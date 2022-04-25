#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10); // SPI CS pin 10

void setup()
{
    SPI.begin(); // SPI 통신 시작
    Serial.begin(9600); // 시리얼 통신 9600 Baud rate
    mcp2515.reset();
    // CAN 통신 속도 500KBPS, 클락 속도 8MHZ
    mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);
    mcp2515.setNormalMode(); // CAN 노말 모드
}

void loop()
{
	digitalWrite(LED_BUILTIN,HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
}
