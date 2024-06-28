This documentation breaks down our CANBUS and Power Switching Prototype. The entire system consists of 3 different modules:
- **[[#Telemetry]]**
- **Safety Module**
- **Power Switching Circuit**
---
# Overview

In the overview of our electrical systems we are focusing on the LV Box:
![[Top Level Schematic.png]]

Within the LV Box we have the 3 modules:
![[LV Schematic.png]]

Data is first relayed from the **Telemetry** module or from external HV controllers to the **Safety** module by utilizing CANBUS. Within the **Safety** module there are 3 ESP32s that each focus on checking different data. For example one module will check on data from the MCU (Motor Control Unit) while another one checks data from the BMS (Battery Management System). When any of the **Safety** modules determines a fault it will take action and send out a fault code on the CANBUS network. This fault code can be displayed or used to prevent further damage.

This is the flowchart for how data is passed along and used:
![[CANbus.drawio (1).png]]

---
# The Prototype

The prototype is a combination of all 3 modules as described above. The result is this schematic below:
![[Fritzing Schematic.png]]

## Telemetry

Our telemetry system comprises of an Arduino MEGA hooked up to:
- Adafruit GPS Module
- Accelerometer
- MCP2515 CAN Transceiver
- Current Sensor
- Ethernet Breakout Board

For our prototype we are just fetching data from the accelerometer and current sensor. We are fetching values from the current sensor **every 50ms** and from the accelerometer **every 500ms**.

The reason current sensor data is fetched more frequently is to simulate system critical data that would be transmitted by HV controllers such as the BMS. We need to frequently monitor BMS data to ensure that our battery is in a safe state. If it isn't then we would need to swiftly act to prevent damage.

Once these values are fetched then they are sent over CAN through the MCP2515. The **Telemetry** module is hooked up to the **Safety** module via this CAN network.

## Safety

Once data has been transmitted from the **Telemetry** module over CAN we need to make sure that data is in range. For the case of the prototype we are only interested in the current sensor data.

For the prototype this module consists of:
- An ESP32
- MCP2515 CAN Transceiver
- 3 LEDs of different colors to indicate:
	- Nominal State (green)
	- Overcurrent State (red)
	- Undercurrent State (orange)

In our test to make things easy we are assuming the current value we want is **0.00A**. If the current is **> 0.05A** then we assume an over current condition. Conversely if the current is **< -0.05A** then we assume an under current condition. If it determines either one of those fault conditions it will take action and turn off power to the car by diverting power to the **BMS_FAULT** line; which powers components that need to be on when there is a BMS fault. The BMS that we have should be the one that controls the battery and can open the contactors that connects the battery to the HV system.

On top of being able to react to current faults the prototype should be the one controlling the precharge circuit. This feature has not been implemented yet, but the wiring for the MOSFETs that would be controlling the precharge has been included.

## Power Switching Circuit

The power switching circuit is model of the one in our schematic. It consists of:
- 2 SPDT relays
- 3 N-type MOSFETs
- 1 Throw Switch to represent an ESTOP switch
- 3 Push Button Switches
- 5 LEDs to indicate states
- Resistors (they are placed as detailed in the schematic)

The circuit in our prototype was hooked up to 2 power sources: 12V DC Power Supply & 12V Auxiliary Battery. Like the car, our prototype will switch between the 2 sources, as the DC/DC source would not be ready until the precharge circuit charges the entire system. It uses the 12V Battery until the BMS_AUX_EN fault is grounded. Afterwards it switches to be powered by the DC source.

Although the circuit is receiving power, it does not get sent to the rest of the car until the driver switch is flipped ON to connect the system. Once it is switched, electricity from the first relay is passed onto the second relay which diverts the power to the rest of the car. However, when a fault is detected this second relay will divert power to our **BMS_FAULT** line. These faults can be triggered by:
- The ESTOP
- The Safety Module via an electrical output signal
- The IMD (through the IMD_CTRL)
The Red LED will turn on if the circuit reaches the fault state. To reset it the fault needs to be cleared and the Green LED will turn back on. If the driver switch is in the OFF position then neither LED will be on.

# Setup & Installation

To replicate this prototype first recreate the circuit through the Fritzing schematic found above (the fritzing file is linked here). Then download the code to the respective microcontrollers:

- MEGA_Lib_Test.ino should be downloaded to the Arduino MEGA
- esp32_receive_lib_test.ino should be downloaded to the ESP32

**NOTE:** The **canFloat** and **sensorFunctions** libraries may have to be installed in your IDE. Also ensure to select the correct ESP32 board and to have the correct ESP32 library downloaded.

On startup the MEGA will calibrate its current sensor to get a reading of 0A. So make sure that the device that is hooked up to the current sensor is not outputting current. After that startup sequence it will send accelerometer data every 500ms and current data every 50ms.

For the ESP32 it will occasionally send a CAN message with an ID of 0x200 and data (12 & 24). The MEGA should print this out every time it receives this message. Aside from this constant loop the ESP32 will parse the data it receives from the MEGA and act accordingly based on what was described earlier.

In conjunction with the Power Switching circuit the system should act like it was described in the prototype section.