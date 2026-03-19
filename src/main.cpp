// FTMS BRIDGE (Com Supervisor de Conexão)
// Desenvolvido por Alessandro Miertschink
// Última atualização: 2026-03-18

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   48  
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- UUIDS DO PROTOCOLO FTMS/CSC ---
static NimBLEUUID FTMS_SVC_UUID("1826");
static NimBLEUUID FTMS_FEAT_CHR_UUID("2ACC");
static NimBLEUUID FTMS_DATA_CHR_UUID("2AD2");
static NimBLEUUID FTMS_CTRL_CHR_UUID("2AD9");
static NimBLEUUID CSC_SVC_UUID("1816");       
static NimBLEUUID CSC_MEAS_CHR_UUID("2A5B");  

// --- VARIÁVEIS GLOBAIS ---
volatile float livePower = 10.0, liveCadence = 10.0;
volatile float lastGradeReceived = 10.0;
volatile bool trainerConnected = false, zwiftConnected = false;
volatile bool doConnect = false;

// Variáveis de controle de Cadência (CSC)
volatile uint32_t lastRevCount = 0;
volatile uint16_t lastEventTime = 0;
volatile unsigned long lastCadenceUpdateMillis = 0;

volatile bool pendingIndicate = false;
volatile uint8_t pendingOpCode = 0x00;

enum ControlState { IDLE, SEND_REQ_CTRL, SEND_POWER, SEND_SIMULATION, SEND_GENERIC };
volatile ControlState ergState = IDLE;

const int THRESHOLDS[] = {127, 173, 207, 242, 276, 310, 345};
const int DEADBAND = 5;
int currentZone = 0;

const uint8_t zonesRGB[8][3] = {
  {150, 150, 150}, {0, 0, 255}, {0, 255, 0}, {139, 69, 19}, 
  {255, 200, 0}, {255, 100, 0}, {139, 0, 0}, {128, 0, 128}
};

uint8_t targetPowerPayload[2] = {0x00, 0x00};
uint8_t targetSimPayload[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
uint8_t genericPayload[20];
size_t genericLen = 0;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pRemoteControlChar = nullptr;
NimBLECharacteristic *pBikeDataChar, *pServerControlChar;
NimBLEAdvertisedDevice* targetDevice = nullptr;

// --- CALLBACKS DE DESCONEXÃO DO ROLO (CLIENTE) ---
class TrainerClientCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* pClient) {
        trainerConnected = false;
        doConnect = false;
        livePower = 0;
        liveCadence = 0;
        Serial.println("!!! [ALERTA] Rolo Físico perdeu o sinal. O Supervisor assumirá.");
    }
};

// --- MÓDULO SUPERVISOR DE CONEXÃO ---
class TrainerConnectionSupervisor {
private:
    unsigned long lastCheck = 0;
    const unsigned long checkInterval = 3000; // Verifica a cada 3 segundos

public:
    void run() {
        if (millis() - lastCheck < checkInterval) return;
        lastCheck = millis();

        // Se não estiver conectado e não houver um pedido pendente de conexão
        if (!trainerConnected && !doConnect) {
            if (!NimBLEDevice::getScan()->isScanning()) {
                Serial.println(">>> [SUPERVISOR] Conexão ausente. Iniciando varredura...");
                
                // Limpeza segura de memória caso exista um cliente residual
                if (pClient != nullptr) {
                    NimBLEDevice::deleteClient(pClient);
                    pClient = nullptr;
                }
                
                // Reinicia o scanner em busca do rolo
                NimBLEDevice::getScan()->clearResults();
                NimBLEDevice::getScan()->start(5, false); // Busca por 5 segundos
            }
        }
    }
};

// Instância global do supervisor
TrainerConnectionSupervisor supervisor;

// --- LED LOGIC ---
void updateLED() {
    static unsigned long lastBlink = 0;
    static bool ledOn = false;
    
    int p = (int)livePower;
    int newZone = 0;

    if      (p > THRESHOLDS[6] + (currentZone == 7 ? -DEADBAND : 0)) newZone = 7;
    else if (p > THRESHOLDS[5] + (currentZone == 6 ? -DEADBAND : 0)) newZone = 6;
    else if (p > THRESHOLDS[4] + (currentZone == 5 ? -DEADBAND : 0)) newZone = 5;
    else if (p > THRESHOLDS[3] + (currentZone == 4 ? -DEADBAND : 0)) newZone = 4;
    else if (p > THRESHOLDS[2] + (currentZone == 3 ? -DEADBAND : 0)) newZone = 3;
    else if (p > THRESHOLDS[1] + (currentZone == 2 ? -DEADBAND : 0)) newZone = 2;
    else if (p > THRESHOLDS[0] + (currentZone == 1 ? -DEADBAND : 0)) newZone = 1;
    else newZone = 0;
    
    currentZone = newZone;

    unsigned long interval = 60000 / (liveCadence > 30 ? liveCadence : 60); 
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

// --- CALLBACK ZWIFT -> ESP ---
class ServerControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) {
        std::string val = pChar->getValue();
        if (val.length() == 0) return;

        uint8_t opCode = val[0];
        pendingOpCode = opCode;
        pendingIndicate = true;
        Serial.printf(">>> [ZWIFT] Comando recebido: %02X. Processamento agendado.\n", opCode);

        switch (opCode) {
            case 0x00: ergState = SEND_REQ_CTRL; break;
            case 0x05:
                if (val.length() >= 3) {
                    targetPowerPayload[0] = val[1]; targetPowerPayload[1] = val[2];
                    ergState = SEND_POWER;
                }
                break;
            case 0x11:
                if (val.length() >= 7) {
                    memcpy((void*)targetSimPayload, &val[1], 6);
                    ergState = SEND_SIMULATION;
                    lastGradeReceived = ((int16_t)(val[3] | (val[4] << 8))) * 0.01;
                }
                break;
            default:
                if (val.length() <= 20) {
                    memcpy((void*)genericPayload, val.data(), val.length());
                    genericLen = val.length();
                    ergState = SEND_GENERIC;
                }
                break;
        }
    }
};

class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        zwiftConnected = true;
        Serial.println(">>> [CORE 1] Zwift Conectado.");
    }
    void onDisconnect(NimBLEServer* pServer) {
        zwiftConnected = false;
        Serial.println(">>> [CORE 1] Zwift Desconectado.");
        NimBLEDevice::getAdvertising()->start();
    }
};

class MyScannerCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* device) {
        if (device->getName().find("Cyclo_130") != std::string::npos) {
            targetDevice = new NimBLEAdvertisedDevice(*device);
            NimBLEDevice::getScan()->stop();
            doConnect = true; // Avisa a TaskTrainer para conectar
        }
    }
};

// --- CORE 0: COMUNICAÇÃO COM O ROLO ---
void TaskTrainer(void * pvParameters) {
    for(;;) {
        // Se o supervisor encontrou o rolo e ativou a flag doConnect
        if (doConnect && targetDevice != nullptr && !trainerConnected) {
            Serial.println(">>> [CORE 0] Tentando (re)conectar ao rolo físico...");
            
            pClient = NimBLEDevice::createClient();
            pClient->setClientCallbacks(new TrainerClientCallbacks(), false);
            pClient->setConnectTimeout(5);

            if (pClient->connect(targetDevice)) {
                pClient->discoverAttributes();
                trainerConnected = true;
                doConnect = false;
                Serial.println(">>> [CORE 0] Sucesso! Conectado ao Rolo Físico.");
                
                // SUBSCREVE FTMS
                NimBLERemoteService* pSvcFTMS = pClient->getService(FTMS_SVC_UUID);
                if (pSvcFTMS) {
                    NimBLERemoteCharacteristic* pData = pSvcFTMS->getCharacteristic(FTMS_DATA_CHR_UUID);
                    if (pData) {
                        pData->subscribe(true, [](NimBLERemoteCharacteristic* pC, uint8_t* pD, size_t len, bool isN) {
                            if (len >= 6) livePower = (int16_t)(pD[4] | (pD[5] << 8));
                        });
                        Serial.println(">>> [CORE 0] Link FTMS Estabelecido.");
                    }
                    pRemoteControlChar = pSvcFTMS->getCharacteristic(FTMS_CTRL_CHR_UUID);
                    if (pRemoteControlChar) {
                        auto pDesc = pRemoteControlChar->getDescriptor(NimBLEUUID("2902"));
                        if (pDesc) pDesc->writeValue((uint8_t*)("\x02\x00"), 2, true);
                    }
                }

                // SUBSCREVE CSC
                NimBLERemoteService* pSvcCSC = pClient->getService(CSC_SVC_UUID);
                if (pSvcCSC) {
                    NimBLERemoteCharacteristic* pCadData = pSvcCSC->getCharacteristic(CSC_MEAS_CHR_UUID);
                    if (pCadData) {
                        pCadData->subscribe(true, [](NimBLERemoteCharacteristic* pC, uint8_t* pD, size_t len, bool isN) {
                            if (len >= 5) {
                                uint32_t currentRev = (pD[1] | (pD[2] << 8));
                                uint16_t currentTime = (pD[3] | (pD[4] << 8));
                                if (lastRevCount != 0 && currentRev != lastRevCount) {
                                    uint16_t timeDiff = currentTime - lastEventTime;
                                    uint32_t revDiff = currentRev - lastRevCount;
                                    if (currentTime < lastEventTime) timeDiff = (65535 - lastEventTime) + currentTime;
                                    if (timeDiff > 0) {
                                        liveCadence = (float)(revDiff * 1024.0 * 60.0) / timeDiff;
                                        lastCadenceUpdateMillis = millis(); 
                                    }
                                }
                                lastRevCount = currentRev; lastEventTime = currentTime;
                            }
                        });
                        Serial.println(">>> [CORE 0] Link CSC Estabelecido.");
                    }
                }
            } else {
                Serial.println("!!! [CORE 0] Falha na conexão. Repassando para o Supervisor.");
                NimBLEDevice::deleteClient(pClient);
                pClient = nullptr;
                doConnect = false;
            }
        }

        if (trainerConnected && pRemoteControlChar) {
            if (ergState == SEND_REQ_CTRL) {
                uint8_t req[1] = {0x00};
                if (pRemoteControlChar->writeValue(req, 1, true)) ergState = IDLE;
            } else if (ergState == SEND_POWER) {
                uint8_t cmd[3] = {0x05, targetPowerPayload[0], targetPowerPayload[1]};
                if (pRemoteControlChar->writeValue(cmd, 3, true)) ergState = IDLE;
            } else if (ergState == SEND_SIMULATION) {
                uint8_t cmd[7] = {0x11, targetSimPayload[0], targetSimPayload[1], targetSimPayload[2], targetSimPayload[3], targetSimPayload[4], targetSimPayload[5]};
                if (pRemoteControlChar->writeValue(cmd, 7, true)) ergState = IDLE;
            } else if (ergState == SEND_GENERIC) {
                if (pRemoteControlChar->writeValue((uint8_t*)genericPayload, genericLen, true)) ergState = IDLE;
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("--- Iniciando FTMS BRIDGE (V2 - Auto Reconnect) ---");
    
    led.begin();
    led.setBrightness(20);
    
    uint8_t newMac[6] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x01};
    esp_base_mac_addr_set(newMac);
    NimBLEDevice::init("Cyclo_130_S3");
    
    NimBLEDevice::setMTU(517); 
    
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    NimBLEService* pFTMS = pServer->createService(FTMS_SVC_UUID);
    
    uint8_t feat[8] = {0x0A, 0x40, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00};
    pFTMS->createCharacteristic(FTMS_FEAT_CHR_UUID, NIMBLE_PROPERTY::READ)->setValue(feat, 8);
    
    pBikeDataChar = pFTMS->createCharacteristic(FTMS_DATA_CHR_UUID, NIMBLE_PROPERTY::NOTIFY);
    pServerControlChar = pFTMS->createCharacteristic(FTMS_CTRL_CHR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pServerControlChar->setCallbacks(new ServerControlCallbacks());
    pFTMS->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(FTMS_SVC_UUID);
    pAdv->start();

    xTaskCreatePinnedToCore(TaskTrainer, "TaskTrainer", 4096, NULL, 1, NULL, 0);
    NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(new MyScannerCallbacks());
    
    // O primeiro scan agora é feito pelo próprio supervisor no loop,
    // então não precisamos iniciar o scan aqui no setup.
}

// --- CORE 1: EXECUÇÃO PRINCIPAL ---
void loop() {
    // 1. Chama o Supervisor de Conexão
    supervisor.run();

    if (zwiftConnected) {
        updateLED();
    } 
    else {        
        static bool ledWasOff = false;
        if (!ledWasOff) {
            led.setPixelColor(0, 0); 
            led.show();
            ledWasOff = true;
        }
    }

    if (millis() - lastCadenceUpdateMillis > 2000) {
        liveCadence = 0;
    }

    if (pendingIndicate && zwiftConnected) {
        uint8_t response[3] = {0x80, pendingOpCode, 0x01}; 
        pServerControlChar->setValue(response, 3);
        pServerControlChar->indicate();
        pendingIndicate = false;
        Serial.printf(">>> [ZWIFT] Indicação assíncrona enviada: 80-%02X-01\n", pendingOpCode);
    }

    if (zwiftConnected) {
        static unsigned long lastNotify = 0;
        if (millis() - lastNotify >= 1000) {
            lastNotify = millis();
            uint8_t d[8] = {0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            uint16_t c = (uint16_t)(liveCadence * 2.0);
            int16_t p = (int16_t)livePower;
            d[4] = c & 0xFF; d[5] = (c >> 8) & 0xFF; d[6] = p & 0xFF; d[7] = (p >> 8) & 0xFF;
            pBikeDataChar->setValue(d, 8); pBikeDataChar->notify();
            
            Serial.printf("[STATUS] PWR: %3dW | CAD: %3.0f | INC: %5.2f%% | CONECTADO: %s\n", 
              (int)livePower, 
              (float)liveCadence, 
              (float)lastGradeReceived,
              trainerConnected ? "SIM" : "NAO");
        }
    }
    vTaskDelay(10);
}