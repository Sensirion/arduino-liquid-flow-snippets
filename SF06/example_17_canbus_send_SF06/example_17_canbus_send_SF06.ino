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
#include <mcp_can.h>
#include <SPI.h>

// -----------------------------------------------------------------------------
// Settings and variables, adjust if needed:
// -----------------------------------------------------------------------------

const int ADDRESS = 0x08; // Standard address for SLF3x Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 500;
const char *UNIT_FLOW = " ml/h";
const char *UNIT_TEMP = " deg C";
int ret;
uint16_t aux_value;
uint16_t sensor_flow_value;
uint8_t flow_high;
uint8_t flow_low;
uint8_t temp_high;
uint8_t temp_low;
uint8_t aux_high;
uint8_t aux_low;
uint16_t sensor_temp_value;
int16_t signed_flow_value;
byte aux_crc;
byte sensor_flow_crc;
byte sensor_temp_crc;
unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};

#define CAN0_INT 2   // Set INT to Pin 2
MCP_CAN CAN0(9);     // Set CS to pin 9


// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(CAN_500KBPS) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  // CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

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

  // Begin continuous measurement
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever
// -----------------------------------------------------------------------------
void loop() {
  Wire.requestFrom(ADDRESS, 9);
  if (Wire.available() < 9) {
    Serial.println("Error while reading flow measurement");
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

  // For simplicity the CRC is not checked here. This should, however, be done in any real project.

  // CAN variables
  flow_high = highByte(sensor_flow_value);
  flow_low = lowByte(sensor_flow_value);

  temp_high = highByte(sensor_temp_value);
  temp_low = lowByte(sensor_temp_value);

  aux_high = highByte(aux_value);
  aux_low = lowByte(aux_value);

  // CAN data
  byte data[8] = {flow_high, flow_low, temp_high, temp_low, aux_high, aux_low, 0x00, 0x00};

  // send data:  ID = 0x351, Standard CAN Frame, Data length = 8 bytes,
  // 'data' is the array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(0x007, 0, 8, data);

  delay(100); // milliseconds delay between reads (for demo purposes only)
}
