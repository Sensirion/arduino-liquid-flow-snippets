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
 *           Reads the sensor's product details
 ******************************************************************************/

#include <Wire.h>

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors

const uint16_t PART_NAME_SIZE                      = 20;
const uint16_t SERIAL_NUMBER_SIZE                  = 4;
const uint16_t PART_NAME_ADDRESS                   = 0x02E8;
const uint16_t SERIAL_NUMBER_ADDRESS               = 0x02F8;

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret, i, j, req_size;

  char     part_name[PART_NAME_SIZE + 1];
  uint32_t serial_number_product;
  byte     crc;

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

    // Read out part name
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFA); // Set EEPROM read mode
    // Write left aligned 12 bit EEPROM address
    Wire.write(PART_NAME_ADDRESS >> 4);
    Wire.write((PART_NAME_ADDRESS << 12) >> 8);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write EEPROM address");
      continue;
    }

    req_size = PART_NAME_SIZE / 2 * 3;
    Wire.requestFrom(ADDRESS, req_size);
    for (i = 0, j = 0; i < req_size; ++i) {
        if ((i + 1) % 3 != 0) {
          part_name[j++] = Wire.read();
        } else {
          crc = Wire.read();
          // Use crc to check integrity of {part_name[j - 2], part_name[j - 1]}
        }
    }
    part_name[j] = '\0';

    // Read out serial number
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFA); // Set EEPROM read mode
    // Write left aligned 12 bit EEPROM address
    Wire.write(SERIAL_NUMBER_ADDRESS >> 4);
    Wire.write((SERIAL_NUMBER_ADDRESS << 12) >> 8);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write EEPROM address");
      continue;
    }

    serial_number_product = 0;
    req_size = SERIAL_NUMBER_SIZE / 2 * 3;
    Wire.requestFrom(ADDRESS, req_size);
    for (i = 0, j = 3; i < req_size; ++i) {
        if ((i + 1) % 3 != 0) {
          serial_number_product |= ((uint32_t) Wire.read()) << (8 * j);
          --j;
        } else {
          crc = Wire.read();
          // Use crc to check integrity of last two read bytes
        }
    }

    Serial.println("Product Details:");
    Serial.print("Part Name:             ");
    Serial.println(part_name);
    Serial.print("Serial Number Product: ");
    Serial.println(serial_number_product) ;
    Serial.println();

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
  raw_sensor_value  = Wire.read() << 8; // read the MSB from the sensor
  raw_sensor_value |= Wire.read();      // read the LSB from the sensor

  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error while reading flow measurement");
  } else {
    Serial.print("Sensor reading: ");
    Serial.println((int16_t) raw_sensor_value);
  }

  delay(3000); // milliseconds delay between reads (for demo purposes)
}
