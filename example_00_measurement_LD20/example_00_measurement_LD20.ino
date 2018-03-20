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
 * Purpose:  Example code for the I2C communication with Sensirion Liquid Flow
 *           Sensor LD20 based on SF06 sensor chip
 *
 *           Read measurements from the sensor
 ******************************************************************************/

#include <Wire.h> // Arduino library for I2C

const int ADDRESS = 0x08; // Standard address for LD20 Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 1200.0;
const float SCALE_FACTOR_TEMP = 200.0;
const char *UNIT_FLOW = " ml/h";
const char *UNIT_TEMP = " deg C";

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  do {
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      delay(500); // wait long enough for chip reset to complete
    }
  } while (ret != 0);

  delay(50); // wait long enough for chip reset to complete
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  int ret;
  uint16_t aux_value;
  uint16_t sensor_flow_value;
  uint16_t sensor_temp_value;
  int16_t signed_flow_value;
  int16_t signed_temp_value;
  float scaled_flow_value;
  float scaled_temp_value;
  byte aux_crc;
  byte sensor_flow_crc;
  byte sensor_temp_crc;

  // To perform a measurement, first send 0x3608 to switch to continuous
  // measurement mode, then read 3x (2 bytes + 1 CRC byte) from the sensor.
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write measurement mode command");

  } else {
    delay(1000);

    for(int i = 0; i < 10; ++i) {
      delay(100);
      if (Wire.requestFrom(ADDRESS, 9) < 9) {
        Serial.println("Error while reading flow measurement");
        continue;
      }

      sensor_flow_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_flow_value |= Wire.read();      // read the LSB from the sensor
      sensor_flow_crc    = Wire.read();
      sensor_temp_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_temp_value |= Wire.read();      // read the LSB from the sensor
      sensor_temp_crc    = Wire.read();
      aux_value          = Wire.read() << 8; // read the MSB from the sensor
      aux_value         |= Wire.read();      // read the LSB from the sensor
      aux_crc            = Wire.read();

      Serial.print("Flow value from Sensor: ");
      Serial.print(sensor_flow_value);

      signed_flow_value = (int16_t) sensor_flow_value;
      Serial.print(", signed value: ");
      Serial.print(signed_flow_value);

      scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
      Serial.print(", scaled value: ");
      Serial.print(scaled_flow_value);
      Serial.print(UNIT_FLOW);

      Serial.print(", Temp value from Sensor: ");
      Serial.print(sensor_temp_value);

      signed_temp_value = (int16_t) sensor_temp_value;
      Serial.print(", signed value: ");
      Serial.print(signed_temp_value);

      scaled_temp_value = ((float) signed_temp_value) / SCALE_FACTOR_TEMP;
      Serial.print(", scaled value: ");
      Serial.print(scaled_temp_value);
      Serial.print(UNIT_TEMP);

      Serial.println("");
    }
    // To stop the continuous measurement, first send 0x3FF9.
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x3F);
    Wire.write(0xF9);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }
  }

  delay(1000); // milliseconds delay between reads (for demo purposes)
}

