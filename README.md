# Sample Code for Arduino

## Summary
The Arduino Platform allows for easy prototyping with endless possibilities. In
order to enable our customers to use this platform, we provide sample code for
use with Sensiron AG's liquid flow sensors. This application note describes the
examples, as well as the main code components used in the examples. It is meant
as a starting point for setting up communication with the liquid flow sensors
through the I2C interface.

## Compatibility
The code snippets are generally compatible with Sensirion's digital flow
sensor portfolio. The current portfolio is based on two different sensor chips,
the SF04 and the SF06.
More specifically:
### SF04
SLG, SLI, SLS, SLQ-QTxxx, LG16-xxxxD, LS32 and LPG10 series sensors.
### SF06
LD20 and SLF3x series sensors.

## Introduction
This sample code library is aimed at customers who have successfully set up
their Arduino and connected their Sensirion liquid flow sensor. Please see the
[Arduino Quick Start Guide](https://developer.sensirion.com/platforms/arduino/arduino-interface-for-liquid-flow-sensors/)
for help in setting up your Arduino and liquid flow sensor.
Sensirion provides code examples showcasing the most important use cases of I2C
communication with our liquid flow sensors.

## Code Examples
### SF04
|Folder | Number & Name                  | Description                                  |
|-------|--------------------------------|----------------------------------------------|
|SF04   | 01 Simple Measurement          | Reads simple measurement data from the<br>sensor. |
|SF04   | 02 Set Resolution              | Reads measurement data from the sensor and<br>changes the resolution. |
|SF04   | 03 Read Scale Factor and Unit  | Reads the scale factor and measurement unit<br>information from the sensor's EEPROM. |
|SF04   | 04 Read Product Details        | Reads the product details (serial number and product name). |
|SF04   | 05 CRC Checksum                | Reads flow measurement data from the sensor<br>and calculates the checksum of the<br>communication for error detection. |
|SF04   | 10 Calibration Field           | Describes the necessary steps to change the<br>calibration field settings. |
|SF04   | 11 Read Temperature Voltage    | Demonstrates how to read temperature and<br>voltage data. |
|SF04   | 12 Two's Complement            | Demonstrates how to use the signed datatype<br>interpretation directly and how to calculate<br>the two's complement manually. |
|SF04   | 13 DIY Flow Meter              | Expands on example 3 to build a stand-alone<br>flow meter using a graphic LCD. |

### SF06
|Folder | Number & Name                 | Description                                  |
|-------|-------------------------------|----------------------------------------------|
|SF06   | 14 Simple measurement (LD20-2600B) | Reads simple measurement data from the<br>LD20-2600B. |
|SF06   | 15 Simple measurement (SLF3S-1300F) | Reads simple measurement data from the<br>SLF3S-1300F and highlights where<br>changes have to be made for other SF06<br>sensors. |
|SF06   | 16 CANBUS receive | Demonstrates how to forward measurement data over CANBUS.<br>This is useful for some test setups. |
|SF06   | 17 CANBUS send | Demonstrates how to forward measurement data over CANBUS.<br>This is useful for some test setups. |
|SF06   | 18 DIY closed loop volume controller | Demonstrates how an SLF3x can be used<br>together with a pump to dispense a controlled volume. |
|SF06   | 19 DIY flow meter | Stand-alone flow meter using a praphic LCD (SF06 sensors). |
|SF06   | 20 simple measurement multiple SLF3x | Demonstrates how to use an I2C multiplexer<br>(TCA9548A) to communicate with up to 8<br>SLF3x sensors on the same bus.  |

## Prereqites
In order to initiate the serial communication, `Serial.begin(9600)` should be
used to set the baud rate to 9600. The code examples require the use of the
"Wire"-library. This Arduino library contains all the relevant functions for
the communication between your Arduino and the I2C device. The provided code
examples are all based on this library and typically use the following
functions:

| Functions                             | Description
|---------------------------------------|--------------------------------------|
| Wire.begin()                          | Join I2C bus (address optional for<br>master). |
| Wire.beginTransmission(ADDRESS)       | Initiate a transmission transaction <br>with the device at ADDRESS<br>over I2C. |
| Wire.write(value)                     | Queues bytes for transmission from<br>master to slave. |
| Wire.endTransmission()                | Commit the transmission transaction:<br>start, send data, stop. |
| Wire.requestFrom(ADDRESS, LEN)        | Master requests data (LEN bytes)<br>from slave at ADDRESS and<br>store to buffer. |
| Wire.available()                      | Retrieve the length of the I2C buffer. |
| Wire.read()                           | Read data from the I2C buffer.       |

For further details, please refer to Arduino's official
[Wire Documentation](https://www.arduino.cc/en/Reference/Wire).

## SF04 Sensor Commands
The following table shows the commands that are typically used for I2C
communication. The commands need to be transmitted through the
`Wire.write(Value)` function that is part of Arduino's Wire library for I2C
communication.

| Command                   | Value | Description           | See Example(s)   |
|---------------------------|-------|-------------------------------|----------|
| USER\_REG\_W              | 0xE2  | Write user register           | 10, 11   |
| USER\_REG\_R              | 0xE3  | Read user register            | 10, 11   |
| ADV\_USER\_REG\_W         | 0xE4  | Write advanced user register  | 2        |
| ADV\_USER\_REG\_R         | 0xE5  | Read advanced user register   | 2        |
| READ\_ONLY\_REG2\_R       | 0xE9  | Read-only register 2          | 4        |
| TRIGGER\_FLOW\_MEASUREMENT| 0xF1  | Trigger a flow measurement    | 1-13     |
| TRIGGER\_TEMP\_MEASUREMENT| 0xF3  | Trigger a temperature measurement | 11   |
| TRIGGER\_VDD\_MEASUREMENT | 0xF5  | Trigger a supply voltage measurement | 11|
| EEPROM\_R                 | 0xFA  | Read EEPROM                   | 3, 4, 13 |
| SOFT\_RESET               | 0xFE  | Soft reset                    | 1-13     |

## SF06 Sensor Commands
SF06 based sensors have a simplified I2C interface. The measurement commands are
calibration specific and therefore depend on the respective sensor. The
available commands are listed in the sensors' datasheets.

## Two's Complement
The two's complement is a binary number representation. This notation is widely
used in computer science to represent both positive and negative integer
numbers. To get the two's complement notation of a negative integer, first write
the absolute value of that integer in binary representation b = bin(abs(x)).
Then invert the binary bit sequence b (i.e. change every 1 into a 0 and every 0
into a 1). Finally, increment the number represented by b by one.
Note that all negative numbers will thus carry a 1 in their highest bit.

**Example: Represent -5 in 8bit**

1. binary representation of absolute value: b = bin(abs(-5)) -> 0000 0101
2. invert: b = xor(b, 1111 1111) -> 1111 1010
3. Increment by one: b = b + 1 -> 1111 1011
