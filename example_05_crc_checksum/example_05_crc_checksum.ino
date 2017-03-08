/*
 * Copyright (c) 2017, Sensirion AG
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
 *           Describes the use of cyclic redundancy check (CRC) with
 *           Sensirion Liquid Flow Sensors
 ******************************************************************************/

#include <Wire.h>         // Arduino library for I2C

#define CRC_POLYNOMIAL 0x131  // P(x)=x^8+x^5+x^4+1 (binary 0001 0011 0001)

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

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
  int b, i;
  byte crc;
  byte calc_crc;
  byte data[2];
  uint16_t raw_sensor_value;
  float sensor_reading;

  Wire.requestFrom(ADDRESS, 3); // reading 2 measurement bytes + the CRC byte
  data[0] = Wire.read(); // read the MSB from the sensor
  data[1] = Wire.read(); // read the LSB from the sensor
  crc = Wire.read();
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error while reading flow measurement");

  } else {
    // compute 8bit checksum (takes 16 micro seconds on the arduino uno)
    calc_crc = 0;
    for (b = 0; b < 2; ++b) {
      calc_crc ^= (data[b]);
      for (i = 8; i > 0; --i) {
        if (calc_crc & 0x80) {
          calc_crc = (calc_crc << 1) ^ CRC_POLYNOMIAL;
        } else {
          calc_crc = (calc_crc << 1);
        }
      }
    }

    if (calc_crc != crc) {
      Serial.print("CRC (expected): ");
      Serial.println(crc);
      Serial.print("CRC (calculated): ");
      Serial.print(calc_crc);
      Serial.println("CRC mismatch while reading flow measurement");

    } else {
      raw_sensor_value = (data[0] << 8) | data[1];

      Serial.print("Raw value from Sensor: ");
      Serial.print(raw_sensor_value);
      Serial.println(" (CRC Check OK)");
    }
  }

  delay(3000); // milliseconds delay between reads (for demo purposes)
}
