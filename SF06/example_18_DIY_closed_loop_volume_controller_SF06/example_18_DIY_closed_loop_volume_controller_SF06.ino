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
 * Purpose: Demonstration of how the sensor measurement of a
 *          Sensirion flow sensor (e.g. SLF3S-1300F) can be used to dose a
 *          volume of liquid correctly with a low cost pump.
 *
 *          Controlls a DC pump (e.g. peristaltic pump, valve, etc.) via the
 *          Arduino Motor Shield. The pump is run until the sensor
 *          has measured the desired volume.
 *
 *          Overflow (Tail) is corrected online.
 ******************************************************************************/

#include <Wire.h>
bool verbose = false;

// -----------------------------------------------------------------------------
// TCA multiplexer specific settings, adjust if needed:
// Delete if you don't use a multiplexer.
// -----------------------------------------------------------------------------

#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// Control specific settings and variables
// -----------------------------------------------------------------------------

// Modes:
// 0: Set Target Vol
// 1: Dose
// 2: Tail (for correction)

int mode = 0;
bool tail = false;
long tail_time;

double target_volume = 0.0;
double totalizer_volume = 0.0;
float duty_rate = 100.0; // percentage
float tail_correction = 0.0;
double delta_t;

double this_micros;
double last_micros;

uint16_t key_value;
uint16_t counter = 0;
uint16_t buffer_counter = 0;

// -----------------------------------------------------------------------------
// LCD specific settings and variables, adjust if needed:
// -----------------------------------------------------------------------------
#include <LiquidCrystal.h>

// initialize the library with the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Use for LCD KeyPad Shield
//for example: ADAFRUIT 772
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // Use for LCD Display
//for example: ADAFRUIT 1447

// -----------------------------------------------------------------------------
// Sensor specific settings and variables, adjust if needed:
// -----------------------------------------------------------------------------

const int ADDRESS = 0x08; // Standard address for LD20 Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 500;
const char *UNIT_FLOW = "ml/min";
const char *UNIT_VOLUME = "ml";
const char *UNIT_TEMP = " deg C";
int ret;
int16_t signed_flow_value;
float sensor_reading;
byte sensor_flow_crc;
float measurement_buffer[10];
float average;

// -----------------------------------------------------------------------------
// Measurement routine
// -----------------------------------------------------------------------------
void measure_flow(){
  Wire.requestFrom(ADDRESS, 3);
  if (Wire.available() < 3) {
    Serial.println("Error while reading flow measurement");
  }

  signed_flow_value  = Wire.read() << 8; // read the MSB from the sensor
  signed_flow_value |= Wire.read();      // read the LSB from the sensor
  sensor_flow_crc    = Wire.read();

  sensor_reading = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
}

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  tcaselect(0);       // choose to read from sensor zero on the multiplexer

  // Setup the Motor Shield
  // Direction and break are not implemented.
  // Using Channel B, break and SENS0 is disconnected by cutting the pads.
  // (for compatibility with the LCD KeypadShield)
  pinMode(11, OUTPUT); // PWM controll for the motor shield's B Por
  pinMode(0, INPUT);   // Input for Current Sensing. Not used in this example.

  // Setup the LCD, a 16x2 (columnsxrows) in the case of the arduino starter kit
  lcd.begin(16, 2);

  // Writing the first line to the LCD:
  lcd.setCursor(0, 0);
  lcd.print("Closed Loop Vol.");
  lcd.setCursor(0, 1);
  lcd.print("Controller SF06");
  delay(1000);

  do {
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      lcd.setCursor(0, 0);
      lcd.print("Controller:     ");
      lcd.setCursor(0, 1);
      lcd.print("No Sensor!      ");
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

  // Write the welcome to the LCD
  lcd.setCursor(0, 0);
  lcd.print("Closed Loop Vol.");
  lcd.setCursor(0, 1);
  lcd.print("Controller SF06");
  delay(2000);
}

// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  //
  // Set Target Volume Menu Mode
  //
  while (true) {
    counter++;

    // Check input Keys approximately every 140 ms
    if (counter % 7 == 0) {
      key_value = analogRead(0);
      //Serial.println(key_value);
      if (900 < key_value && key_value < 1100) {
      } else if (500 < key_value && key_value < 700) {
        // don't continue until the key is released.
        do {
          key_value = analogRead(0);
        } while (500 < key_value && key_value < 700);
        break; // select

      } else if (300 < key_value && key_value < 500) {
        target_volume -= 0.01; // left

      } else if (50 < key_value && key_value < 120) {
        target_volume = target_volume + 1; // up

      } else if (key_value <  50) {
        target_volume += 0.01; // right

      } else if (200 < key_value && key_value < 300) {
        target_volume -= 1; // down
      }

      if (target_volume < 0.0) {
        target_volume = 0.0;
      }
    }

    // Displaying flow on the LCD approximatelly every 500 ms.
    if (counter % 25 == 0) {
    // move the LCD cursor to the first row
      lcd.setCursor(0, 0);
      lcd.print("Set tar. volume:  ");
      lcd.setCursor(0, 1);
      lcd.print("                  ");
      lcd.setCursor(1, 1);
      lcd.print(target_volume,3);
      lcd.setCursor(14, 1);
      lcd.print(UNIT_VOLUME);
    }

    delay(20);
  }

  //
  // Dispense mode
  //
  this_micros = micros();
  totalizer_volume = 0.0;
  tail = false;

  while (true) {
    counter++;

    // Check input Keys approximately every 140 ms
    if (counter % 14 == 0) {
      key_value = analogRead(0);
      //Serial.println(key_value);
      if (500 < key_value && key_value < 700){
        analogWrite(11, 0); // stop
        // don't continue until the key is released.
        do {
          key_value = analogRead(0);
        } while (500 < key_value && key_value < 700);
        break; // select
      }
    }

    // measure the flow and check the volume, approximatelly every 10 ms
    if (counter % 1 == 0) {
      measure_flow();
      last_micros = this_micros;
      this_micros = micros();
      delta_t = ((this_micros - last_micros)) / 1000000.0 / 60.0; // since the unit of flow is ml/min
      totalizer_volume = totalizer_volume + sensor_reading * delta_t;

      if (target_volume == 0.0) {
        // nothing to do
        break;

      } else if (target_volume <= totalizer_volume + tail_correction) {
        // stop once we have dosed enough
        analogWrite(11, (int) (0.0));
        tail = true;
        tail_time = millis() + 1000;
        break;

      } else if (target_volume > totalizer_volume + tail_correction) {
        // start or keep running the pump as long the volume is not reached ye
        analogWrite(11, (int) (duty_rate / 100.0 * 255.0));
      }

      // Print the status every 100 loops
      if (verbose && (counter % 100 == 0)) {
        Serial.print("Sensor reading: ");   Serial.print(sensor_reading);
        Serial.print(" dalta t: ");         Serial.print(delta_t);
        Serial.print(" Target: ");          Serial.print(target_volume);
        Serial.print(" Tail correction: "); Serial.print(tail_correction);
        Serial.print(" Totalizer: ");       Serial.println(totalizer_volume);
      }
    }

    // Displaying flow on the LCD approximatelly every 500 ms.
    if (counter % 50 == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Set:           ");
      lcd.setCursor(4, 0);
      lcd.print(target_volume,3);
      lcd.setCursor(14, 0);
      lcd.print(UNIT_VOLUME);

      lcd.setCursor(0, 1);
      lcd.print("Sen:           ");
      lcd.setCursor(4, 1);
      lcd.print(totalizer_volume,3);
      lcd.setCursor(14, 1);
      lcd.print(UNIT_VOLUME);
    }

    delay(10); // milliseconds delay between rounds
  }

  //
  // Tail mode
  // After the pump has been stopped some more liquid will flow though the sensor.
  // This "tail" is determined and substracted from consequtive dosings.
  //

  while (true) {
    counter++;

    // Check input Keys approximately every 140 ms
    if (counter % 14 == 0) {
      key_value = analogRead(0);
      // Serial.println(key_value);
      if (500 < key_value && key_value < 700){
        analogWrite(11, (int) (0.0)); // stop
        // don't continue until the key is released.
        do {
          key_value = analogRead(0);
        } while (500 < key_value && key_value < 700);
        break; // select
      }
    }

    // measure the flow and check the volume, approximatelly every 10 ms
    if (counter % 1 == 0) {
      measure_flow();
      last_micros = this_micros;
      this_micros = micros();
      delta_t = ((this_micros - last_micros)) / 1000000.0 / 60.0; // since the unit of flow is ml/min
      totalizer_volume = totalizer_volume + sensor_reading * delta_t;

      // Serial.print(tail_time); Serial.println(millis());
      if (tail && (tail_time <= millis())) {
        tail_correction = tail_correction + totalizer_volume - target_volume;
        tail = false;
      }

      if (verbose && (counter % 100 == 0)) {
        Serial.print("Sensor reading: ");   Serial.print(sensor_reading);
        Serial.print(" delta t: ");         Serial.print(delta_t);
        Serial.print(" Target: ");          Serial.print(target_volume);
        Serial.print(" Tail correction: "); Serial.print(tail_correction);
        Serial.print(" Totalizer: ");       Serial.println(totalizer_volume);
      }
    }

    // Displaying flow on the LCD approximatelly every 500 ms.
    if (counter % 50 == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Set:           ");
      lcd.setCursor(4, 0);
      lcd.print(target_volume,3);
      lcd.setCursor(14, 0);
      lcd.print(UNIT_VOLUME);

      lcd.setCursor(0, 1);
      lcd.print("Sen:           ");
      lcd.setCursor(4, 1);
      lcd.print(totalizer_volume,3);
      lcd.setCursor(14, 1);
      lcd.print(UNIT_VOLUME);
    }

    delay(10); // milliseconds delay between rounds
  }
}
