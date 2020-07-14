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
 * Purpose: Based on Example code for the I2C communication with Sensirion
 *          Liquid Flow Sensors
 *
 *          Reads the flow measurement from the sensor and
 *          displays it on the LCD
 ******************************************************************************/

#include <Wire.h>

// -----------------------------------------------------------------------------
// TCA multiplexer specific settings, adjust if needed: (just in case)
// -----------------------------------------------------------------------------

#define TCAADDR 0x70
int active_sensors[8];

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// LCD specific settings and variables, adjust if needed:
// -----------------------------------------------------------------------------
#include <LiquidCrystal.h>

// initialize the library with the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Use for LCD KeyPad Shield,
// for example: ADAFRUIT 772
// LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // Use for LCD Display,
// for example: ADAFRUIT 1447

// -----------------------------------------------------------------------------
// Sensor specific settings and variables, adjust if needed:
// -----------------------------------------------------------------------------

const int ADDRESS = 0x08; // Address for SLF3x Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 500; // Scale Factor for flow rate measurement
const char *UNIT_FLOW = " ml/min"; // physical unit of the flow rate measurement
int ret;
int16_t signed_flow_value;
float scaled_flow_value;
byte sensor_flow_crc;

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  tcaselect(0);       // only relevant if the I2C Multiplexer is used

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

  // Begin measurement
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error while sending start measurement command, retrying...");
  }

  // Setup the LCD, a 16x2 (columnsxrows) in the case of the arduino starter kit
  lcd.begin(16, 2);

  // Writing the first line to the LCD:
  lcd.print("DIY Flow Meter");
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  Wire.requestFrom(ADDRESS, 3);
  if (Wire.available() < 3) {
    Serial.println("Error while reading flow measurement");
  }

  signed_flow_value  = Wire.read() << 8; // read the MSB from the sensor
  signed_flow_value |= Wire.read();      // read the LSB from the sensor
  sensor_flow_crc    = Wire.read();

  scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;

  // Serial.print("Sensor reading: ");
  // Serial.println(scaled_flow_value);

  // Displaying flow on the LCD
  // move the LCD cursor to the second row
  lcd.setCursor(0, 1);
  lcd.print(scaled_flow_value);
  lcd.print(UNIT_FLOW);

  delay(250); // milliseconds delay between reads (for demo purposes)
}
