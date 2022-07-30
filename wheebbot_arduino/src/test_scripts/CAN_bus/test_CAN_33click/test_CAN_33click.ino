//Using Seedstudio CAN library
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>


const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 8;

mcp2515_can CAN(SPI_CS_PIN);
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Available bitrates with 10MHz and MCP2515:
// 5-10-20-40-50-100-125-200-250-500-1000 kBPS

void setup() {
  while (!Serial);
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  while (CAN_OK != CAN.begin(CAN_1000KBPS,MCP_10MHz)) {             // the higher the bitrate, the lower is the delay between two consecutive packages
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    Serial.println("Init ok");
}

void loop() {
  // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    stmp[7] = stmp[7] + 1;
    if (stmp[7] == 100) {
        stmp[7] = 0;
        stmp[6] = stmp[6] + 1;

        if (stmp[6] == 100) {
            stmp[6] = 0;
            stmp[5] = stmp[5] + 1;
        }
    }

    CAN.sendMsgBuf(0x00, 0, 8, stmp);
    CAN.sendMsgBuf(0x01, 0, 8, stmp);// to check delay 
    delay(100);                       // send data per 100ms
    SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");

}
