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
    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == CAN.checkReceive()){
        CAN.readMsgBuf(&len,buf);
        unsigned long canId = CAN.getCanId();

        Serial.print("\nData from ID: 0x");
        Serial.println(canId,HEX);
        for(int i=0;i<len;i++){
            Serial.print(buf[i]);
            Serial.print("\t");
        }
    }
}