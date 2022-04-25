#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);

void setup()
{   
    SPI.begin(); // Begin SPI communication
	mcp2515.reset();
    // CAN 통신 속도를 500KBPS로, 클락 속도를 8MHZ로 맞춘다
    mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);
    mcp2515.setNormalMode();
}

void loop()
{
    canMsg.can_id = 0x036; // CAN id = 0x036
    canMsg.can_dlc = 8; // CAN data length = 8
    canMsg.data[0] = 125; // value in [0] (max=255)
    canMsg.data[1] = 203; // value in [1] (max=255)
    canMsg.data[2] = 255; // value in [2] (max=255)
    canMsg.data[3] = 0x00; // 나머지는 모두 0
    canMsg.data[4] = 0x00; 
    canMsg.data[5] = 0x00; 
    canMsg.data[6] = 0x00; 
    canMsg.data[7] = 0x00; 
    
    mcp2515.sendMessage(&canMsg); // CAN 메시지 전송
    delay(1000);
}
