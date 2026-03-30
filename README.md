# FTMS Bridge (FTMS + CSC)

## Introduction
The **FTMS Bridge** is a high-performance tool designed to act as a bridge between a smart indoor cycling trainer and indoor cycling softwares.
Specifically developed to interface with and support the **CYCLOTRONICS** ecosystem—a premier smart trainer brand **Made-in-Brazil** and **ZWIFT** Cycling App.

To foster a more inclusive indoor cycling ecosystem, I invite fellow developers and enthusiasts to extend this codebase. Whether by adding support for other trainer brands or enhancing existing features, your contributions are welcome to help increase compatibility across the community.

While it can run on standard ESP32 boards, the use of an **ESP32-S3** is highly recommended to handle the specific workload of this project. The S3's superior processing power and optimized Bluetooth stack ensure that the concurrent execution of multiple BLE services and the real-time high-precision mathematical interpolations remain stable without latency, even during intense data throughput.

Developed to support the testing and development of the Cyclo_130_S3 Bridge and compatible apps (like Zwift, Rouvy, etc.), this code uses the `NimBLE` library to broadcast two standard BLE profiles simultaneously:
* **FTMS (Fitness Machine Service):** Broadcasts instantaneous power and speed, and receives ERG mode (target power) and simulated inclination commands.
* **CSC (Cycling Speed and Cadence Service):** Broadcasts highly accurate, anti-jitter crank revolution data to simulate cadence.

---

## Acknowledgments
This source code was developed and optimized with the support of **Google Gemini**, following several weeks of intensive AI training and iterative learning to ensure full protocol compliance and high-precision data handling. 

## Code Architecture & Classes

The code is organized into logical blocks to handle BLE initialization, asynchronous callbacks, and the main data-generation loop.

### 1. Configurations & UUIDs
Defines the standard Bluetooth SIG UUIDs for FTMS (`1826`) and CSC (`1816`), along with their respective characteristics. This ensures the simulator is recognized as a standard fitness device by third-party apps and bridges.

### 2. Telemetry & State Variables
Holds the real-time simulation data (`livePower`, `liveCadence`, `liveSpeed`) and the state of the machine (`isERGMode`, `currentIncline`, `currentTargetPower`). It also includes the accumulator variables used to calculate perfect crank revolution timings.

### 3. `MyServerCallbacks` Class
Inherits from `NimBLEServerCallbacks`.
* **`onConnect`**: Flags the device as connected, pausing advertising.
* **`onDisconnect`**: Safely resets the ERG mode state and automatically restarts BLE advertising so the bridge or app can reconnect seamlessly.

### 4. `ControlPointCallbacks` Class
Inherits from `NimBLECharacteristicCallbacks` and acts as the "listener" for incoming commands.
* Intercepts write requests to the FTMS Control Point (`2AD9`).
* Parses standard `OpCodes` to handle **Target Power (ERG Mode - `0x05`)**, **Inclination (`0x03`)**, and **Indoor Bike Simulation Parameters (`0x11`)**.
* Automatically sends the required `0x80` acknowledgment response back to the client to confirm the command was received and applied.

### 5. `setup()`
Initializes the ESP32 hardware and BLE stack.
* Sets a custom MAC address (useful for overriding default hardware limits or matching known devices).
* Configures TX power (`ESP_PWR_LVL_P9`) for maximum range.
* Creates the NimBLE Server, sets up the FTMS and CSC services, assigns the callbacks, and starts broadcasting the device as `Cyclo_130_SIM`.

### 6. `loop()`
The core engine of the simulator, running at a controlled 500ms interval.
* **FTMS Payload:** Constructs a 6-byte array using the `0x44` flag, packing the simulated speed and power into standard 16-bit integers.
* **CSC Anti-Jitter Logic:** Instead of just sending a raw timestamp, it mathematically calculates the exact millisecond a full crank revolution completes. This fractional timestamping ensures that the receiving bridge calculates a perfectly stable cadence (e.g., exactly 85 RPM) without jumping or oscillating due to loop timing mismatches.

---

## How to Adapt for Your Own Fitness Machine

If you want to use this code to upgrade a "dumb" spin bike, an older treadmill, or a custom smart trainer, you can easily bridge physical hardware to this code:

### 1. Replace Static Variables with Sensor Reads
Currently, variables like `livePower`, `liveCadence`, and `liveSpeed` are hardcoded.
* **Cadence:** Attach a magnetic reed switch or Hall effect sensor to your crank. Use an interrupt in the ESP32 to count the time between revolutions, update `liveCadence`, and feed it to the loop.
* **Speed:** Attach a similar sensor to the flywheel.
* **Power:** If you have load cells on the pedals or a known power curve based on flywheel speed and resistance, calculate the wattage dynamically and update `livePower`.

### 2. Physical Resistance Control (Actuators)
In the `ControlPointCallbacks` class, the code currently just prints the requested ERG power or inclination to the Serial monitor.
* If your bike has a motorized magnetic brake or a stepper motor controlling tension, you can add your motor control logic directly inside `case 0x05:` (Target Power) or `case 0x11:` (Inclination). 
* *Example:* `myStepper.moveTo(mapPowerToPosition(currentTargetPower));`

### 3. Modify FTMS Flags for Extra Data
If your machine measures additional metrics (like Heart Rate or Distance), you need to change the FTMS flag.
* The current flag is `0x44` (Speed + Power).
* Consult the Bluetooth FTMS specification to add bits for Heart Rate (Bit 9) or Total Distance (Bit 8), and expand the `indoorBikeData` array size accordingly.

### 4. Customize Device Identity
In the `setup()` function:
* Change the `NimBLEDevice::init("Your_Bike_Name");` to whatever you want apps like Zwift to display.
* Remove or modify `esp_base_mac_addr_set(newMac);` if you prefer to use the ESP32's native factory MAC address.
