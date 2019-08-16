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
 *           Demonstrates how to read temperature and voltage readings
 ******************************************************************************/

#include <Wire.h>

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors

const float SCALEFACTOR_FLOW = 1.0f;
const float SCALEFACTOR_TEMP = 10.0f;
const float SCALEFACTOR_VDD  = 1000.0f;

const byte TRIGGER_FLOW_COMMAND = 0xF1;
const byte TRIGGER_TEMP_COMMAND = 0xF3;
const byte TRIGGER_VDD_COMMAND  = 0xF5;
const byte TRIGGER[] = {TRIGGER_FLOW_COMMAND,
                        TRIGGER_TEMP_COMMAND,
                        TRIGGER_VDD_COMMAND};
const float MEASUREMENT_FACTOR[] = {SCALEFACTOR_FLOW,
                                    SCALEFACTOR_TEMP,
                                    SCALEFACTOR_VDD};
const char *MEASUREMENT_TYPE[] = {"Flow (signed value)", "Temp", "VDD"};
const char *MEASUREMENT_UNIT[] = {""," C"," Volt"};

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;
  uint16_t user_reg;

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
  } while (ret != 0);
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  int i;
  int ret;
  uint16_t reg;
  uint16_t raw_sensor_value;
  float sensor_reading;

  // Loop through the measurement of flow, temperature and VDD
  for (i = 0; i <= 2; ++i) {

    Wire.beginTransmission(ADDRESS);
    Wire.write(TRIGGER[i]);
    ret = Wire.endTransmission();
    if (ret != 0){
      Serial.println("Error during write measurement mode");
      continue;
    }

    Wire.requestFrom(ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error during read");
      continue;
    }
    raw_sensor_value  = Wire.read() << 8;
    raw_sensor_value |= Wire.read();

    sensor_reading = ((int16_t) raw_sensor_value) / MEASUREMENT_FACTOR[i];

    Serial.print(MEASUREMENT_TYPE[i]);
    Serial.print(" Measurement: ");
    Serial.print(sensor_reading);
    Serial.println(MEASUREMENT_UNIT[i]);

    delay(1000);
  }
}
