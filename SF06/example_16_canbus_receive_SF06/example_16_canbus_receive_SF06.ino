
/*******************************************************************************
 * Purpose:  Example code for the I2C communication with Sensirion Liquid Flow
 *           Sensors based on the SF06 sensor chip.
 *
 *           This Code uses the SLF3S-1300F as an example. Adjust the sensor
 *           specific values as needed, see respective datasheet.
 *           Flow measurements are based on the water calibration
 *           with command: 0x3608. For IPA use 0x3615 instead.
 ******************************************************************************/

#include <SPI.h>
#include "mcp_can.h"

// -----------------------------------------------------------------------------
// CAN BUS Shield settings
// (uses Seeed Studio example code)
// -----------------------------------------------------------------------------
const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);                                    // Set CS pin

// -----------------------------------------------------------------------------
// Sensor specific settings and variables, adjust if needed:
// -----------------------------------------------------------------------------

const float SCALE_FACTOR_FLOW = 500.0; // Scale Factor for flow rate measurement
const float SCALE_FACTOR_TEMP = 200.0; // Scale Factor for temperature measurement

const char *UNIT_FLOW = " ml/h";
const char *UNIT_TEMP = " deg C";

uint16_t sensor_flow_value;
uint16_t sensor_temp_value;
uint16_t aux_value;
int16_t signed_flow_value;
int16_t signed_temp_value;
float scaled_flow_value;
float scaled_temp_value;

// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup()
{
    Serial.begin(9600);

    while (CAN_OK != CAN0.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

void loop()
{
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN0.checkReceive()) // check for incoming data
    {
        CAN0.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf

        unsigned long canId = CAN0.getCanId();

        Serial.println("-----------------------------");
        Serial.print("Get data from ID: 0x");
        Serial.println(canId, HEX);

        // See individual bits:
        // for(int i = 0; i < len; i++)    // print the data
        // {
        //     Serial.print(buf[i], HEX);
        //     Serial.print("\t");
        // }
        // Serial.println();

        sensor_flow_value = (uint16_t) word(buf[0], buf[1]);
        sensor_temp_value = (uint16_t) word(buf[2], buf[3]);
        aux_value = (uint16_t) word(buf[4], buf[5]);

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

        Serial.print(", aux value: ");
        Serial.print(aux_value);

        Serial.println("");
    }
}
