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
 * Purpose: Example code for the I2C communication with Sensirion Liquid
 *          Flow Sensors
 *
 *          Reads the scale factor and measurement unit information from the
 *          sensor's EEPROM
 ******************************************************************************/

#include <Wire.h>

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors

const bool VERBOSE_OUTPUT = true; // set to false for less verbose output

const uint16_t ACTIVE_CONFIG_FIELD_SIZE = 0x300;
const uint16_t WRDADR_OFFSET_SCALEFACTOR = 0x02B6;

const char *DIMENSIONS[] = {"X","X","X","n","u","m","c","d",
                            "","-","h","k","M","G","", ""} ;
const char *UNITS[] = {"nl","sl","sl@15C","sl@25C","X","X","X","X","l","g",
                       "X","X","X","X","X","X","Pa","bar","m H2O","in H2O"};
const char *TIME_BASES[] = {"-","us","ms","s","Min","h","day",
                            "","","","","","","","",""};

const char *dimension;
const char *unit;
const char *time_base;
uint16_t raw_scale_factor;

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;
  uint16_t user_reg;
  uint16_t active_config_field;
  uint16_t active_config_field_address;
  uint16_t base_address;

  uint16_t measurement_unit_code;
  uint16_t dimension_code;
  uint16_t unit_code;
  uint16_t time_base_code;

  byte crc1;
  byte crc2;

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

    // Read scale factor and measurement unit
    // Read the user register to get the active configuration field
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xE3);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while setting register read mode");
      continue;
    }

    Wire.requestFrom(ADDRESS, 2);
    user_reg  = Wire.read() << 8;
    user_reg |= Wire.read();
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while reading register settings");
      continue;
    }

    // To determine the base EEPROM address for actual sensor information,
    // bit <6:4> of User Register (called active configuration field) must be
    // multiplied by the field size 0x300.
    active_config_field = ((user_reg & 0x0070) >> 4);
    base_address = active_config_field * ACTIVE_CONFIG_FIELD_SIZE;
    active_config_field_address = WRDADR_OFFSET_SCALEFACTOR + base_address;

    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFA); // Set EEPROM read mode
    // Write left aligned 12 bit EEPROM address
    Wire.write(active_config_field_address >> 4);
    Wire.write((active_config_field_address << 12) >> 8);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write EEPROM address");
      continue;
    }

    // Read the scale factor and the adjacent unit
    Wire.requestFrom(ADDRESS, 6);
    raw_scale_factor       = Wire.read() << 8;
    raw_scale_factor      |= Wire.read();
    crc1                   = Wire.read();
    measurement_unit_code  = Wire.read() << 8;
    measurement_unit_code |= Wire.read();
    crc2                   = Wire.read();
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while reading EEPROM");
      continue;
    }

    dimension_code = (measurement_unit_code & 0x000F);
    time_base_code = (measurement_unit_code & 0x00F0) >> 4;
    unit_code      = (measurement_unit_code & 0x1F00) >> 8;

    dimension      = DIMENSIONS[dimension_code];
    unit           = UNITS[unit_code];
    time_base      = TIME_BASES[time_base_code];

    if (VERBOSE_OUTPUT) {
      Serial.println();
      Serial.println("----------------");
      Serial.print("Scale factor:           ");
      Serial.println(raw_scale_factor);
      Serial.print("Measurement unit code:  ");
      Serial.println(measurement_unit_code);
      Serial.print("Dimension:    ");
      Serial.print(dimension);
      Serial.print("  code:  ");
      Serial.println(dimension_code);
      Serial.print("Unit:         ");
      Serial.print(unit);
      Serial.print("  code:  ");
      Serial.println(unit_code);
      Serial.print("Time base:    ");
      Serial.print(time_base);
      Serial.print(" code: ");
      Serial.println(time_base_code);
      Serial.println("----------------");
      Serial.println();
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
  float sensor_reading;

  Wire.requestFrom(ADDRESS, 2); // reading 2 bytes ignores the CRC byte
  raw_sensor_value  = Wire.read() << 8; // read the MSB from the sensor
  raw_sensor_value |= Wire.read();      // read the LSB from the sensor
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error while reading flow measurement");

  } else {
    sensor_reading = ((int16_t) raw_sensor_value) / (float)raw_scale_factor;

    Serial.print("Sensor reading: ");
    Serial.print(sensor_reading);
    Serial.print(" ");
    Serial.print(dimension);
    Serial.print(unit);
    Serial.print("/");
    Serial.println(time_base); 
  }

  delay(3000); // milliseconds delay between reads (for demo purposes)
}

