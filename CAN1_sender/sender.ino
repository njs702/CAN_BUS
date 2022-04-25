#include <SPI.h>
#include <mcp_can.h>

MCP_CAN CAN(10);

void setup(){
    Serial.begin(9600);
    while(CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)){
        Serial.println("CAN BUS init failed!");
        delay(100);
    }
    Serial.println("CAN BUS init Success!");
}

void loop(){
    Serial.println("Sending");
    unsigned char stmp[8] = {0,1,2,3,4,5,6,7};
    CAN.sendMsgBuf(0x11,0,8,stmp);
    delay(1000);
}