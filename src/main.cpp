// FTMS BRIDGE
// Developed by Alessandro Miertschink (with help from Gemini)
// Last update: 2026-03-19

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   48  
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- FTMS/CSC PROTOCOL UUIDS ---
static NimBLEUUID FTMS_SVC_UUID("1826");
static NimBLEUUID FTMS_FEAT_CHR_UUID("2ACC");
static NimBLEUUID FTMS_DATA_CHR_UUID("2AD2");
static NimBLEUUID FTMS_CTRL_CHR_UUID("2AD9");
static NimBLEUUID CSC_SVC_UUID("1816");       
static NimBLEUUID CSC_MEAS_CHR_UUID("2A5B");  

// --- ENUMS ---
enum ControlState { IDLE, SEND_REQ_CTRL, SEND_POWER, SEND_SIMULATION, SEND_GENERIC };

// --- SHARED STATE (Cross-Core) ---
struct SharedBridgeState {
    // Bike Metrics
    volatile float power = 10.0;
    volatile float cadence = 10.0;
    volatile float grade = 10.0;

    // Connection Flags
    volatile bool trainerConnected = false;
    volatile bool zwiftConnected = false;
    volatile bool doConnect = false;

    // Cadence Control (CSC)
    volatile uint32_t lastRevCount = 0;
    volatile uint16_t lastEventTime = 0;
    volatile unsigned long lastCadenceUpdateMillis = 0;

    // Zwift Control Commands
    volatile bool pendingIndicate = false;
    volatile uint8_t pendingOpCode = 0x00;
    volatile ControlState ergState = IDLE;

    // Payloads
    uint8_t targetPowerPayload[2] = {0x00, 0x00};
    uint8_t targetSimPayload[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    uint8_t genericPayload[20];
    size_t genericLen = 0;
};

// --- BLE POINTER CONTEXT ---
struct BLEContext {
    NimBLEClient* client = nullptr;
    NimBLERemoteCharacteristic* remoteControlChar = nullptr;
    NimBLECharacteristic* bikeDataChar = nullptr;
    NimBLECharacteristic* serverControlChar = nullptr;
    NimBLEAdvertisedDevice* targetDevice = nullptr;
};

// Global instantiations
SharedBridgeState state;
BLEContext ble;

// --- LED INDICATOR MODULE ---
class LEDIndicator {
private:
    const int THRESHOLDS[7] = {127, 173, 207, 242, 276, 310, 345};
    const int DEADBAND = 5;
    int currentZone = 0;
    
    const uint8_t zonesRGB[8][3] = {
        {150, 150, 150}, {0, 0, 255}, {0, 255, 0}, {139, 69, 19}, 
        {255, 200, 0}, {255, 100, 0}, {139, 0, 0}, {128, 0, 128}
    };
    
    unsigned long lastBlink = 0;
    bool ledOn = false;
    bool ledWasOff = false;

public:
    void begin() {
        led.begin();
        led.setBrightness(20);
        turnOff();
    }

    void turnOff() {
        if (!ledWasOff) {
            led.setPixelColor(0, 0); 
            led.show();
            ledWasOff = true;
        }
    }

    void update(float power, float cadence) {
        ledWasOff = false;
        int p = (int)power;
        int newZone = 0;

        // Calculate active zone with hysteresis (deadband)
        if      (p > THRESHOLDS[6] + (currentZone == 7 ? -DEADBAND : 0)) newZone = 7;
        else if (p > THRESHOLDS[5] + (currentZone == 6 ? -DEADBAND : 0)) newZone = 6;
        else if (p > THRESHOLDS[4] + (currentZone == 5 ? -DEADBAND : 0)) newZone = 5;
        else if (p > THRESHOLDS[3] + (currentZone == 4 ? -DEADBAND : 0)) newZone = 4;
        else if (p > THRESHOLDS[2] + (currentZone == 3 ? -DEADBAND : 0)) newZone = 3;
        else if (p > THRESHOLDS[1] + (currentZone == 2 ? -DEADBAND : 0)) newZone = 2;
        else if (p > THRESHOLDS[0] + (currentZone == 1 ? -DEADBAND : 0)) newZone = 1;
        else newZone = 0;
        
        currentZone = newZone;

        // Adjust blink speed based on cadence
        unsigned long interval = 60000 / (cadence > 30 ? cadence : 60); 
        if (millis() - lastBlink >= (ledOn ? 100 : interval - 100)) {
            lastBlink = millis();
            ledOn = !ledOn;
            if (ledOn) {
                led.setPixelColor(0, led.Color(zonesRGB[currentZone][0], zonesRGB[currentZone][1], zonesRGB[currentZone][2]));
            } else {
                led.setPixelColor(0, 0); 
            }
            led.show();
        }
    }
};

LEDIndicator statusLed;

// --- TRAINER DISCONNECTION CALLBACKS (CLIENT) ---
class TrainerClientCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* pClient) {
        state.trainerConnected = false;
        state.doConnect = false;
        state.power = 0;
        state.cadence = 0;
        Serial.println("!!! [ALERT] Physical Trainer lost signal. Supervisor will take over.");
    }
};

// --- CONNECTION SUPERVISOR MODULE ---
class TrainerConnectionSupervisor {
private:
    unsigned long lastCheck = 0;
    const unsigned long checkInterval = 3000; // Checks every 3 seconds

public:
    void run() {
        if (millis() - lastCheck < checkInterval) return;
        lastCheck = millis();

        // If not connected and there is no pending connection request
        if (!state.trainerConnected && !state.doConnect) {
            if (!NimBLEDevice::getScan()->isScanning()) {
                Serial.println(">>> [SUPERVISOR] Connection missing. Starting scan...");
                
                // Safe memory cleanup in case a residual client exists
                if (ble.client != nullptr) {
                    NimBLEDevice::deleteClient(ble.client);
                    ble.client = nullptr;
                }
                
                // Restarts scanner to search for the trainer
                NimBLEDevice::getScan()->clearResults();
                NimBLEDevice::getScan()->start(5, false); // Scans for 5 seconds
            }
        }
    }
};

TrainerConnectionSupervisor supervisor;

// --- ZWIFT -> ESP CALLBACK ---
class ServerControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) {
        std::string val = pChar->getValue();
        if (val.length() == 0) return;

        uint8_t opCode = val[0];
        state.pendingOpCode = opCode;
        state.pendingIndicate = true;
        Serial.printf(">>> [ZWIFT] Command received: %02X. Processing scheduled.\n", opCode);

        switch (opCode) {
            case 0x00: 
                state.ergState = SEND_REQ_CTRL; 
                break;
            case 0x05:
                if (val.length() >= 3) {
                    state.targetPowerPayload[0] = val[1]; 
                    state.targetPowerPayload[1] = val[2];
                    state.ergState = SEND_POWER;
                }
                break;
            case 0x11:
                if (val.length() >= 7) {
                    memcpy((void*)state.targetSimPayload, &val[1], 6);
                    state.ergState = SEND_SIMULATION;
                    state.grade = ((int16_t)(val[3] | (val[4] << 8))) * 0.01;
                }
                break;
            default:
                if (val.length() <= 20) {
                    memcpy((void*)state.genericPayload, val.data(), val.length());
                    state.genericLen = val.length();
                    state.ergState = SEND_GENERIC;
                }
                break;
        }
    }
};

class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        state.zwiftConnected = true;
        Serial.println(">>> [CORE 1] Zwift Connected.");
    }
    void onDisconnect(NimBLEServer* pServer) {
        state.zwiftConnected = false;
        Serial.println(">>> [CORE 1] Zwift Disconnected.");
        NimBLEDevice::getAdvertising()->start();
    }
};

class MyScannerCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* device) {
        if (device->getName().find("Cyclo_130") != std::string::npos) {
            ble.targetDevice = new NimBLEAdvertisedDevice(*device);
            NimBLEDevice::getScan()->stop();
            state.doConnect = true; // Notifies TaskTrainer to connect
        }
    }
};

// --- CORE 0: COMMUNICATION WITH TRAINER ---
void TaskTrainer(void * pvParameters) {
    for(;;) {
        // If the supervisor found the trainer and activated the doConnect flag
        if (state.doConnect && ble.targetDevice != nullptr && !state.trainerConnected) {
            Serial.println(">>> [CORE 0] Trying to (re)connect to physical trainer...");
            
            ble.client = NimBLEDevice::createClient();
            ble.client->setClientCallbacks(new TrainerClientCallbacks(), false);
            ble.client->setConnectTimeout(5);

            if (ble.client->connect(ble.targetDevice)) {
                ble.client->discoverAttributes();
                state.trainerConnected = true;
                state.doConnect = false;
                Serial.println(">>> [CORE 0] Success! Connected to Physical Trainer.");
                
                // SUBSCRIBE TO FTMS
                NimBLERemoteService* pSvcFTMS = ble.client->getService(FTMS_SVC_UUID);
                if (pSvcFTMS) {
                    NimBLERemoteCharacteristic* pData = pSvcFTMS->getCharacteristic(FTMS_DATA_CHR_UUID);
                    if (pData) {
                        pData->subscribe(true, [](NimBLERemoteCharacteristic* pC, uint8_t* pD, size_t len, bool isN) {
                            if (len >= 6) state.power = (int16_t)(pD[4] | (pD[5] << 8));
                        });
                        Serial.println(">>> [CORE 0] FTMS Link Established.");
                    }
                    ble.remoteControlChar = pSvcFTMS->getCharacteristic(FTMS_CTRL_CHR_UUID);
                    if (ble.remoteControlChar) {
                        auto pDesc = ble.remoteControlChar->getDescriptor(NimBLEUUID("2902"));
                        if (pDesc) pDesc->writeValue((uint8_t*)("\x02\x00"), 2, true);
                    }
                }

                // SUBSCRIBE TO CSC
                NimBLERemoteService* pSvcCSC = ble.client->getService(CSC_SVC_UUID);
                if (pSvcCSC) {
                    NimBLERemoteCharacteristic* pCadData = pSvcCSC->getCharacteristic(CSC_MEAS_CHR_UUID);
                    if (pCadData) {
                        pCadData->subscribe(true, [](NimBLERemoteCharacteristic* pC, uint8_t* pD, size_t len, bool isN) {
                            if (len >= 5) {
                                uint32_t currentRev = (pD[1] | (pD[2] << 8));
                                uint16_t currentTime = (pD[3] | (pD[4] << 8));
                                if (state.lastRevCount != 0 && currentRev != state.lastRevCount) {
                                    uint16_t timeDiff = currentTime - state.lastEventTime;
                                    uint32_t revDiff = currentRev - state.lastRevCount;
                                    if (currentTime < state.lastEventTime) timeDiff = (65535 - state.lastEventTime) + currentTime;
                                    if (timeDiff > 0) {
                                        state.cadence = (float)(revDiff * 1024.0 * 60.0) / timeDiff;
                                        state.lastCadenceUpdateMillis = millis(); 
                                    }
                                }
                                state.lastRevCount = currentRev; 
                                state.lastEventTime = currentTime;
                            }
                        });
                        Serial.println(">>> [CORE 0] CSC Link Established.");
                    }
                }
            } else {
                Serial.println("!!! [CORE 0] Connection failed. Handing over to Supervisor.");
                NimBLEDevice::deleteClient(ble.client);
                ble.client = nullptr;
                state.doConnect = false;
            }
        }

        // Process Command Queue to Physical Trainer
        if (state.trainerConnected && ble.remoteControlChar) {
            if (state.ergState == SEND_REQ_CTRL) {
                uint8_t req[1] = {0x00};
                if (ble.remoteControlChar->writeValue(req, 1, true)) state.ergState = IDLE;
            } else if (state.ergState == SEND_POWER) {
                uint8_t cmd[3] = {0x05, state.targetPowerPayload[0], state.targetPowerPayload[1]};
                if (ble.remoteControlChar->writeValue(cmd, 3, true)) state.ergState = IDLE;
            } else if (state.ergState == SEND_SIMULATION) {
                uint8_t cmd[7] = {0x11, state.targetSimPayload[0], state.targetSimPayload[1], state.targetSimPayload[2], state.targetSimPayload[3], state.targetSimPayload[4], state.targetSimPayload[5]};
                if (ble.remoteControlChar->writeValue(cmd, 7, true)) state.ergState = IDLE;
            } else if (state.ergState == SEND_GENERIC) {
                if (ble.remoteControlChar->writeValue((uint8_t*)state.genericPayload, state.genericLen, true)) state.ergState = IDLE;
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("--- Starting FTMS BRIDGE (V2 - Auto Reconnect) ---");
    
    statusLed.begin();
    
    uint8_t newMac[6] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x01};
    esp_base_mac_addr_set(newMac);
    NimBLEDevice::init("Cyclo_130_S3");
    
    NimBLEDevice::setMTU(517); 
    
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    NimBLEService* pFTMS = pServer->createService(FTMS_SVC_UUID);
    
    uint8_t feat[8] = {0x0A, 0x40, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00};
    pFTMS->createCharacteristic(FTMS_FEAT_CHR_UUID, NIMBLE_PROPERTY::READ)->setValue(feat, 8);
    
    ble.bikeDataChar = pFTMS->createCharacteristic(FTMS_DATA_CHR_UUID, NIMBLE_PROPERTY::NOTIFY);
    ble.serverControlChar = pFTMS->createCharacteristic(FTMS_CTRL_CHR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    ble.serverControlChar->setCallbacks(new ServerControlCallbacks());
    pFTMS->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(FTMS_SVC_UUID);
    pAdv->start();

    xTaskCreatePinnedToCore(TaskTrainer, "TaskTrainer", 4096, NULL, 1, NULL, 0);
    NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(new MyScannerCallbacks());
}

// --- CORE 1: MAIN EXECUTION ---
void loop() {
    // 1. Calls the Connection Supervisor
    supervisor.run();

    // 2. Manage LEDs based on Zwift Connection
    if (state.zwiftConnected) {
        statusLed.update(state.power, state.cadence);
    } else {        
        statusLed.turnOff();
    }

    // 3. Cadence Drop-off Check
    if (millis() - state.lastCadenceUpdateMillis > 2000) {
        state.cadence = 0;
    }

    // 4. Handle Pending Zwift Indications
    if (state.pendingIndicate && state.zwiftConnected && ble.serverControlChar) {
        uint8_t response[3] = {0x80, state.pendingOpCode, 0x01}; 
        ble.serverControlChar->setValue(response, 3);
        ble.serverControlChar->indicate();
        state.pendingIndicate = false;
        Serial.printf(">>> [ZWIFT] Asynchronous indication sent: 80-%02X-01\n", state.pendingOpCode);
    }

    // 5. Notify Zwift Periodically
    if (state.zwiftConnected && ble.bikeDataChar) {
        static unsigned long lastNotify = 0;
        if (millis() - lastNotify >= 1000) {
            lastNotify = millis();
            uint8_t d[8] = {0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            uint16_t c = (uint16_t)(state.cadence * 2.0);
            int16_t p = (int16_t)state.power;
            d[4] = c & 0xFF; d[5] = (c >> 8) & 0xFF; d[6] = p & 0xFF; d[7] = (p >> 8) & 0xFF;
            
            ble.bikeDataChar->setValue(d, 8); 
            ble.bikeDataChar->notify();
            
            Serial.printf("[STATUS] PWR: %3dW | CAD: %3.0f | INC: %5.2f%% | CONNECTED: %s\n", 
              (int)state.power, 
              (float)state.cadence, 
              (float)state.grade,
              state.trainerConnected ? "YES" : "NO");
        }
    }
    
    vTaskDelay(10);
}