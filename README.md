# CAN Bus Data Transmission (RPM, Speed, Temperature)

## Table of Contents
1. [Introduction](#introduction)
2. [MCP2515 CAN module](#mcp2515-can-module)
    - 2.1 [Interfacing with ESP8266](#interfacing-with-esp8266)
        -2.2  [Pin connections](#pin-connections)
3. [Hardware Implementation](#hardware-implementation)
    - 3.1 [Wiring and Termination Resistors](#wiring-and-termination-resistors)
    - 3.2 [Importance of 120 Ohm Termination Resistor](#importance-of-120-ohm-termination-resistor)
    - 3.3 [Effects of Incorrect Termination Resistance](#effects-of-incorrect-termination-resistance)
4. [Code for Sending RPM, Speed, and Temperature on CAN Bus](#code-for-sending-rpm-speed-and-temperature-on-can-bus)
    - 4.1 [Explanation of the Code](#explaination-of-the-code)
        - 4.1.1 [Libraries and Constants](#1-libraries-and-constants)
        - 4.1.2 [Global Variables and Structs](#2-global-variables-and-structs)
        - 4.1.3 [Functions](#3-functions)
        - 4.1.4 [Setup Function](#4-setup-function)
        - 4.1.5  [Loop Function](#5-loop-function)
        - 4.1.6 [State Logic](#6-state-logic)
        - 4.1.7 [Sending Data via CAN](#7-sending-data-via-can)
5. [Output](#output)


## Introduction
In this section, the data is taken from previously interfaced rotary encoder and tempreature sensors which is then sent on CAN bus using MCP2515 module. 

## MCP2515 CAN module

![image](https://github.com/KetanMe/RPM-speed-and-tempreature-sending-using-CAN/assets/121623546/b1c38cc4-5cc9-46c8-a588-7c804ab830a8)

(read [here](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf))

The MCP2515 is a stand-alone CAN controller developed to simplify applications that require interfacing
with a CAN bus.
The device consists of three main blocks:
1. The CAN module, which includes the CAN
protocol engine, masks, filters, transmit and
receive buffers.
2. The control logic and registers that are used to
configure the device and its operation.
3. The SPI protocol block.

![image](https://github.com/KetanMe/RPM-speed-and-tempreature-sending-using-CAN/assets/121623546/03cae1e3-d9f4-4af4-93d1-3c21feb8069f)

## Interfacing with esp8266
### Pin connections
|MCP2515       |ESP8266 (NodeMCU)|
|--------------|-----------------|
|VCC           |3.3V             |
|GND           |GND              |
|SCK           |D5 (GPIO14)      |
|SO            |D6 (GPIO12)      |
|SI            |D7 (GPIO13)      |
|CS            |Any digital pin (e.g., D8 - GPIO15)|
|INT           |Not connected (or connect to any GPIO for interrupt handling)|


# Hardware Implementation
![CAN](https://github.com/KetanMe/RPM-speed-and-tempreature-sending-using-CAN/assets/121623546/f615a7e2-b91d-47ac-9580-e8a3f7e0c091)

### Wiring and Termination Resistors:

- **Red Wire (CANH)**: Connected to the CAN High (CANH) pin of the MCP2515 module.
  
- **Black Wire (CANL)**: Connected to the CAN Low (CANL) pin of the MCP2515 module.

- **Termination Resistors**: 
  - 120 ohm termination resistors are connected at both ends of the CAN bus. 
  - One end of each resistor connects to CANH and CANL respectively, while the other end is connected to ground.
  - These resistors terminate the CAN bus to prevent signal reflections and ensure proper signal integrity.

### Importance of 120 Ohm Termination Resistor:

- **Preventing Signal Reflections**: 
  - The termination resistors ensure that the CAN signals are properly terminated, preventing signal reflections from occurring on the bus.
  - Signal reflections can lead to data corruption and communication errors, especially in longer CAN networks or high-speed communication.

### Effects of Incorrect Termination Resistance:

- **Less Than 120 Ohm**: 
  - If the termination resistance is less than 120 ohms, it can result in signal overshoot and undershoot.
  - This can cause signal distortion and increase the risk of communication errors.

- **Greater Than 120 Ohm**: 
  - If the termination resistance is greater than 120 ohms, it can weaken the signal levels.
  - This can lead to signal attenuation and decrease the reliability of communication, especially in long CAN networks.

## Code for sending RPM, speed and tempreature on CAN Bus.
```cpp
#include <Wire.h>
#include <mcp2515.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SENDER_CAN_ID 0x124
const int spiCS = D8;
MCP2515 mcp2515Sender(spiCS);

#define ENC_A D0
#define ENC_B D1
#define ENC_Z D2

const int ENC_COUNT_REV = 4096;
const int MAX_SPEED = 5000;
const float radius = 0.05;

volatile long encoderValue = 0;
unsigned long rpmInterval = 1000;
unsigned long rpmPreviousMillis = 0;
float rpm = 0;

#define SENSOR_2_BUS D3
#define SENSOR_3_BUS D4

OneWire sensor2Wire(SENSOR_2_BUS);
OneWire sensor3Wire(SENSOR_3_BUS);

DallasTemperature sensor2(&sensor2Wire);
DallasTemperature sensor3(&sensor3Wire);

struct can_frame canMsg;

enum State {
  WAIT_FOR_DATA,
  PRINT_RPM,
  PRINT_TEMP,
  TEMPORARY_STOP
};

State currentState = WAIT_FOR_DATA;
unsigned long lastTempPrintMillis = 0;
unsigned long rpmStopMillis = 0;
const unsigned long tempPrintInterval = 10000;

void ICACHE_RAM_ATTR updateEncoder() {
  encoderValue++;
}

float readTemperature(DallasTemperature &sensor);

void setup() {
  Serial.begin(9600);
  mcp2515Sender.reset();
  if (mcp2515Sender.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    while (1);
  }
  mcp2515Sender.setNormalMode();
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), updateEncoder, FALLING);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - rpmPreviousMillis > rpmInterval) {
    rpmPreviousMillis = currentMillis;
    rpm = (float)(encoderValue * 60.0 / ENC_COUNT_REV);
    if (rpm > MAX_SPEED) {
      rpm = MAX_SPEED;
    }
    currentState = PRINT_RPM;
    encoderValue = 0;
    if (currentMillis - rpmStopMillis >= tempPrintInterval) {
      currentState = PRINT_TEMP;
      rpmStopMillis = currentMillis;
    }
  }

  switch (currentState) {
    case PRINT_RPM: {
      float speed = (2 * PI * radius * rpm) / 60.0;
      canMsg.can_id = SENDER_CAN_ID;
      canMsg.can_dlc = 8;
      memcpy(&canMsg.data[0], &rpm, sizeof(rpm));
      memcpy(&canMsg.data[4], &speed, sizeof(speed));
      mcp2515Sender.sendMessage(&canMsg);
      break;
    }
    case PRINT_TEMP: {
      float temp1 = readTemperature(sensor2);
      canMsg.can_id = SENDER_CAN_ID;
      canMsg.can_dlc = 4;
      memcpy(&canMsg.data[0], &temp1, sizeof(temp1));
      mcp2515Sender.sendMessage(&canMsg);

      float temp2 = readTemperature(sensor3);
      canMsg.can_id = SENDER_CAN_ID;
      canMsg.can_dlc = 4;
      memcpy(&canMsg.data[0], &temp2, sizeof(temp2));
      mcp2515Sender.sendMessage(&canMsg);

      currentState = PRINT_RPM;
      break;
    }
  }
}

float readTemperature(DallasTemperature &sensor) {
  sensor.requestTemperatures();
  return sensor.getTempCByIndex(0);
}
```

## Explaination of the code.
### 1. Libraries and Constants
```cpp
#include <Wire.h>
#include <mcp2515.h>
#include <OneWire.h>
#include <DallasTemperature.h>
```
These are the libraries used in the code. They include necessary libraries for CAN communication using MCP2515, OneWire protocol for Dallas temperature sensors, and DallasTemperature library for interfacing with DS18B20 temperature sensors.

```cpp
#define SENDER_CAN_ID 0x124
```
This defines the CAN ID for the sender node. In a CAN network, each node needs a unique identifier.

```cpp
const int spiCS = D8;
```
This defines the chip select (CS) pin used for communication with the MCP2515 CAN controller.

```cpp
#define ENC_A D0
#define ENC_B D1
#define ENC_Z D2
```
These define the pins for rotary encoder inputs.

```cpp
const int ENC_COUNT_REV = 4096;
const int MAX_SPEED = 5000;
const float radius = 0.05;
```
These are constants defining encoder specifications such as pulses per revolution, maximum speed in RPM, and the radius of the rotating object.

```cpp
#define SENSOR_2_BUS D3
#define SENSOR_3_BUS D4
```
These define the OneWire bus pins for the temperature sensors.

### 2. Global Variables and Structs
```cpp
MCP2515 mcp2515Sender(spiCS);
OneWire sensor2Wire(SENSOR_2_BUS);
OneWire sensor3Wire(SENSOR_3_BUS);
```
These are instances of the MCP2515 class and OneWire class for communication with the CAN controller and temperature sensors respectively.

```cpp
struct can_frame canMsg;
```
This struct `can_frame` is used to define the structure of CAN messages.

```cpp
volatile long encoderValue = 0;
unsigned long rpmInterval = 1000;
unsigned long rpmPreviousMillis = 0;
float rpm = 0;
```
These variables are used for encoder readings and RPM calculation.

```cpp
enum State {
  WAIT_FOR_DATA,
  PRINT_RPM,
  PRINT_TEMP,
  TEMPORARY_STOP
};

State currentState = WAIT_FOR_DATA;
unsigned long lastTempPrintMillis = 0;
unsigned long rpmStopMillis = 0;
const unsigned long tempPrintInterval = 10000;
```
This enum defines different states for the state machine. `currentState` keeps track of the current state.

### 3. Functions
```cpp
void ICACHE_RAM_ATTR updateEncoder() {...}
```
This function is called whenever there's an interrupt from the encoder. It increments the `encoderValue` variable.

```cpp
float readTemperature(DallasTemperature &sensor) {...}
```
This function reads temperature from a given DallasTemperature sensor instance and returns the temperature value in Celsius.

```cpp
void prepareAndSendTempCANMessage(float temp, const char* sensorName) {...}
```
This function prepares and sends a CAN message containing temperature data from a sensor.

### 4. Setup Function
```cpp
void setup() {...}
```
In the setup function, the serial communication is initialized, MCP2515 is configured, pins are set up for encoder, and interrupts are attached to encoder pins.

### 5. Loop Function
```cpp
void loop() {...}
```
In the loop function, the current state is checked, and based on the state, either RPM data is sent or temperature data is read and sent. The state machine logic handles the transition between sending RPM and temperature data.

### 6. State Logic
The state machine logic switches between sending RPM data and temperature data. If it's time to switch from sending RPM to temperature, it transitions to the temperature state.

### 7. Sending Data via CAN
Data is sent via CAN using the `sendMessage` method of the `MCP2515` class. CAN messages are structured with a CAN ID and data payload. The `can_frame` struct is used to define the structure of CAN messages.

**In this code:**
- Data (RPM, temperature) is packaged into CAN messages and sent over the CAN bus using the MCP2515 controller.
- Each message includes a CAN ID which helps receivers to understand the content and priority of the message.
- The MCP2515 library is used to configure and interact with the MCP2515 CAN controller, facilitating communication over the CAN bus.

**CAN (Controller Area Network)** Read [here](https://github.com/KetanMe/Introduction-to-Control-Area-Network-CAN-)

## Output



https://github.com/KetanMe/RPM-speed-and-tempreature-sending-using-CAN/assets/121623546/e3128858-d432-4184-bfcf-15f44440b171



