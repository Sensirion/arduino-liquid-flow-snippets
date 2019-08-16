/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * Purpose:  Example code for the I2C communication with Sensirion Liquid
 *           Flow Sensors
 *
 *           Demonstrates the necessary steps to change the calibration field
 *           setting
 ******************************************************************************/

#include <Wire.h>

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors

// ------------------------------------------------------------------------------------------------- 
// Arduino setup routine, just runs once:
// -------------------------------------------------------------------------------------------------
void setup() {
  int ret;
  uint16_t user_reg;
  uint16_t user_reg_CF0;
  uint16_t user_reg_CF1;
  uint16_t user_reg_CF2;
  uint16_t user_reg_CF3;
  uint16_t user_reg_CF4;

  Serial.begin(9600);
  Wire.begin();

  do {
    delay(1000); // Error handling for example: wait a second, then try again

    // Soft reset the sensor
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFE);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      continue;
    }
    delay(50); // wait long enough for reset

    // Read the user register to get the active configuration field
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xE3);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while setting register read mode");
      continue;
    }

    Wire.requestFrom(ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error while reading register settings");
      continue;
    }
    user_reg  = Wire.read() << 8;
    user_reg |= Wire.read();

    // The active calibration field is controlled by the user register setting
    // bits <6:4>:
    //        000: calibration field 0 (default setting for most liquid flow
    //                                  sensors)
    //        001: calibration field 1
    //        010: calibration field 2
    //        011: calibration field 3
    //        100: calibration field 4
    user_reg_CF0 = (user_reg & 0xFF8F) | 0x0000;
    user_reg_CF1 = (user_reg & 0xFF8F) | 0x0010;
    user_reg_CF2 = (user_reg & 0xFF8F) | 0x0020;
    user_reg_CF3 = (user_reg & 0xFF8F) | 0x0030;
    user_reg_CF4 = (user_reg & 0xFF8F) | 0x0040;
    // i.e. (adv_user_reg_heater_off & B1111 1111 1000 1111) |
    //       B0000 0000 0001 0000;

    // Write user register with calibration field 0
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xE2);                           // Send command
    Wire.write((byte)(user_reg_CF0 >> 8));      // Send MSB
    Wire.write((byte)(user_reg_CF0 & 0x00FF));  // Send LSB
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write register settings");
      continue;
    }

    // Switch to measurement mode
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xF1);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }
  } while (ret != 0);
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  int ret;
  uint16_t raw_sensor_value;

  Wire.requestFrom(ADDRESS, 2); // reading 2 bytes ignores the CRC byte
  if (Wire.available() < 2) {
    Serial.println("Error while reading flow measurement");

  } else {
    raw_sensor_value  = Wire.read() << 8; // read the MSB from the sensor
    raw_sensor_value |= Wire.read();      // read the LSB from the sensor
    Serial.print("Sensor reading: ");
    Serial.println((int16_t) raw_sensor_value);
  }

  delay(3000); // milliseconds delay between reads (for demo purposes)
}
