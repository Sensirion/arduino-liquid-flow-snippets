# Sample Code for Arduino

## Summary
The Arduino Platform allows for easy prototyping with endless possibilities. In
order to enable our customers to use this platform, we provide sample code for
use with Sensiron AG's liquid flow sensors. This application note describes the
examples, as well as the main code components used in the examples. It is meant
as a starting point for setting up communication with the liquid flow sensors
through the I2C interface.

## Compatibility
The sample snippets are generally compatible with Sensirion's digital flow
sensor portfolio. More specifically:

SLG, SLI, SLS, SLQ-QTxxx, LG16-xxxxD, LS32 and LPG10 series sensors.

Exception: Example\_00\_Measurement\_LD20 only works with the LD20 series
sensors based on Sensirion's SF06 chips.

## Introduction
This sample code library is aimed at customers who have successfully set up
their Arduino and connected their Sensirion liquid flow sensor. Please see the
[Arduino Quick Start Guide](https://developer.sensirion.com/platforms/arduino/arduino-interface-for-liquid-flow-sensors/)
for help in setting up your Arduino and liquid flow sensor.
Sensirion provides code examples showcasing the most important use cases of I2C
communication with our liquid flow sensors.

## Code Examples
| Number & Name                 | Description                                  |
|-------------------------------|----------------------------------------------|
| 00 Simple measurement (LD20 only) | Reads simple measurement data from the<br>LD20 sensor series (SF06 flow chip). |
| 01 Simple Measurement             | Reads simple measurement data from the<br>sensor. |
| 02 Set Resolution                 | Reads measurement data from the sensor and<br>changes the resolution. |
| 03 Read Scale Factor and Unit     | Reads the scale factor and measurement unit<br>information from the sensor's EEPROM. |
| 04 Read Product Details           | Reads the product details (serial number and product name). |
| 05 CRC Checksum                   | Reads flow measurement data from the sensor<br>and calculates the checksum of the<br>communication for error detection. |
| 10 Calibration Field              | Describes the necessary steps to change the<br>calibration field settings. |
| 11 Read Temperature Voltage       | Demonstrates how to read temperature and<br>voltage data. |
| 12 Two's Complement               | Demonstrates how to use the signed datatype<br>interpretation directly and how to calculate<br>the two's complement manually. |
| 13 DIY Flow Meter                 | Expands on example 3 to build a stand-alone<br>flow meter using a graphic LCD. |

## Prerequisites
In order to initiate the serial communication, `Serial.begin(9600)` should be
used to set the baud rate to 9600. The code examples require the use of the
"Wire"-library. This Arduino library contains all the relevant functions for
the communication between your Arduino and the I2C device. The provided code
examples are all based on this library and typically use the following
functions:

| Functions                             | Description
|---------------------------------------|--------------------------------------|
| Wire.begin()                          | Join I2C bus (address optional for<br>master). |
| Wire.beginTransmission(ADDRESS)       | Starts transmission with device at<br>ADDRESS over I2C. |
| Wire.write(value)                     | Queues bytes for transmission from<br>master to slave. |
| Wire.endTransmission()                | Stops transmission.                  |
| Wire.requestFrom(ADDRESS, LEN)        | Master requests data (LEN bytes)<br>from slave at ADDRESS. |
| Wire.read()                           | Read data from the I2C bus.          |

For further details, please refer to Arduino's official
[Wire Documentation](https://www.arduino.cc/en/Reference/Wire).

## Sensor Commands
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
| TRIGGER\_FLOW\_MEASUREMENT| 0xF1  | Trigger a flow measurement    | All      |
| TRIGGER\_TEMP\_MEASUREMENT| 0xF3  | Trigger a temperature measurement | 11   |
| TRIGGER\_VDD\_MEASUREMENT | 0xF5  | Trigger a supply voltage measurement | 11|
| EEPROM\_R                 | 0xFA  | Read EEPROM                   | 3, 4, 13 |
| SOFT\_RESET               | 0xFE  | Soft reset                    | All      |

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

