/*
 * Copyright (c) 2019, Sensirion AG
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
 * Purpose:  Example code for the I2C communication with multiple Sensirion
 *           Liquid Flow Sensors based on the SF06 sensor chip.
 *           For I2C multiplexing the TCA9548A breakout board from Adafriut is used.
 *           (Adafruit 2717)
 *
 *           This Code uses the SLF3S-1300F as an example. Adjust the sensor
 *           specific values as needed, see respective datasheet.
 *           Flow measurements are based on the water calibration
 *           with command: 0x3608. For IPA use 0x3615 instead.
 ******************************************************************************/

#include <Wire.h> // Arduino library for I2C

// -----------------------------------------------------------------------------
// TCA multiplexer specific settings, adjust if needed:
// -----------------------------------------------------------------------------

#define TCAADDR 0x70
int active_sensors[8];

void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// Sensor specific settings, adjust if needed:
// -----------------------------------------------------------------------------

const int ADDRESS = 0x08; // Address for SLF3x Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
const char *UNIT_FLOW = " ml/min"; //physical unit of the flow rate measurement
int ret;
int16_t signed_flow_value;
float scaled_flow_value;
byte sensor_flow_crc;

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  // Try to reset the sensor via each of the 8 TCA's
  // ports to check for sensor presence.
  for(int i = 0; i < 8; i++) {
    tcaselect(i);
    // Soft reset the sensor to check
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.print("No sensor detected on TCA port #");
      Serial.println(i);
      active_sensors[i] = 0;

    } else {
      Serial.print("Sensor detected on TCA port #");
      Serial.println(i);
      active_sensors[i] = 1;
    }
  }

  delay(50); // wait long enough for chip reset to complete

  for(int i = 0; i < 8; i++) {
    if (active_sensors[i] == 0) {
      continue; // dismiss if this port is not active.
    }
    tcaselect(i);

    // Send 0x3608 to switch to continuous measurement mode (H20 calibration).
    // Check datasheet for available measurement commands.
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x36);
    Wire.write(0x08);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.print("Error during write measurement mode command on TCA port #");
      Serial.print(i);
      Serial.println("!");
    } else {
      Serial.print("Measurement on TCA port #");
      Serial.print(i);
      Serial.println(" started!");
    }
  }
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  for (int i = 0; i < 8; i++) {
    if (active_sensors[i] == 0) {
      continue; // dismiss if this port is not active.
    }
    tcaselect(i);

    Wire.requestFrom(ADDRESS, 3);
    if (Wire.available() < 3) {
      Serial.println("Error while reading flow measurement");
      continue;
    }

    signed_flow_value  = Wire.read() << 8; // read the MSB from the sensor
    signed_flow_value |= Wire.read();      // read the LSB from the sensor
    sensor_flow_crc    = Wire.read();

    scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
    Serial.print("Flow measurement on TCA port #");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(scaled_flow_value);
    Serial.println(UNIT_FLOW);
  }

  delay(1000); // milliseconds delay between reads (for demo purposes)
}
