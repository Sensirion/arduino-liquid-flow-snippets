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
 *           Read measurements from the sensor while changing the resolution
 ******************************************************************************/

#include <Wire.h>

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors
const uint8_t MIN_BIT_RESOLUTION = 9;
const uint8_t MAX_BIT_RESOLUTION = 16;
const bool VERBOSE_OUTPUT = true; // set to false for less verbose output

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  do {
    // Soft reset the sensor
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFE);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      delay(1000); // Error handling for example: wait a second, then try again
    }
  } while (ret != 0);

  delay(50); // wait long enough for chip reset to complete
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {

  static unsigned int sensor_resolution = MAX_BIT_RESOLUTION;

  int ret;
  uint16_t raw_sensor_value;
  uint16_t adv_user_reg_original;
  uint16_t adv_user_reg_new;
  uint16_t resolution_mask;

  unsigned long start_time;
  unsigned long stop_time;

  // The resolution setting is controlled by bits 11:9 of the advanced user
  // register.
  // Possible settings for the sensor flow resolution are:
  //    000: 9 bit (flow);
  //    001: 10 bit (flow);
  //    010: 11 bit (flow);
  //    011: 12 bit (flow);
  //    100: 13 bit (flow);
  //    101: 14 bit (flow);
  //    110: 15 bit (flow);
  //    111: 16 bit (flow);

  // Loop through valid resolution settings
  sensor_resolution += 1;
  if (sensor_resolution > MAX_BIT_RESOLUTION) {
    sensor_resolution = MIN_BIT_RESOLUTION;
  }
  resolution_mask = 0xF1FF | ((sensor_resolution - 9) << 9);

  // Change mode to read adv. user register
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xE5);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write register read mode");

  } else {
    // Read the content of the adv user register
    Wire.requestFrom(ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error during read register settings");

    } else {
      adv_user_reg_original  = Wire.read() << 8;
      adv_user_reg_original |= Wire.read();
      adv_user_reg_new = (adv_user_reg_original | 0x0E00) & resolution_mask;

      if (VERBOSE_OUTPUT) {
        // Display register values and settings
        Serial.println();
        Serial.println("----------------");
        Serial.print("New resolution setting:       ");
        Serial.println(sensor_resolution);
        Serial.print("Resolution bit setting (BIN): ");
        Serial.println(sensor_resolution - 9, BIN);
        Serial.print("Resolution mask:     ");
        Serial.println(resolution_mask, BIN);
        Serial.print("Adv. user reg read:   ");
        Serial.println(adv_user_reg_original, BIN);
        Serial.print("Adv. user reg write:  ");
        Serial.println(adv_user_reg_new, BIN);
        Serial.println("----------------");
      }

      // Apply resolution changes:
      // Change mode to write to adv. user register
      Wire.beginTransmission(ADDRESS);
      Wire.write(0xE4);                           // Send command
      Wire.write((byte)(adv_user_reg_new >> 8));      // Send MSB
      Wire.write((byte)(adv_user_reg_new & 0xFF));    // Send LSB
      ret = Wire.endTransmission();
      if (ret != 0) {
        Serial.println("Error during write register settings");
      }
    }

    // Switch to measurement mode
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xF1);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");

    } else {
      // Record the time before the measurements
      start_time = micros();
      for (int i = 0; i < 10; ++i) {
        Wire.requestFrom(ADDRESS, 2); // reading 2 bytes ignores the CRC byte
        if (Wire.available() < 2) {
          Serial.println("Error while reading flow measurement");
        } else {
          raw_sensor_value  = Wire.read() << 8; // read the MSB from the sensor
          raw_sensor_value |= Wire.read();      // read the LSB from the sensor
        }
      }

      // Record the time after the measurement is finished and the result is read
      stop_time = micros();

      Serial.print("Measurement value: ");
      Serial.print((int16_t) raw_sensor_value);

      Serial.print(", resolution: ");
      Serial.print(sensor_resolution);

      Serial.print(", duration for 10 measurments: ");
      Serial.print(stop_time - start_time);
      Serial.println(" usec");
    }
  }

  delay(3000); // milliseconds delay between reads (for demo purposes)
}
