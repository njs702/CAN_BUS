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

}
