#include <SPI.h>
#include <mcp_can.h>

// spi cs핀 연결 포트 : 10
MCP_CAN CAN(10);

void setup(){
    Serial.begin(9600);
    while(CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)){
        Serial.println("CAN BUS init Failed");
        delay(100);
    }
    Serial.println("CAN BUS Init Success");

    // 2번 핀 인터럽트 설정(falling 때), 인터럽트 들어오면 CAN_int 함수 실행
    attachInterrupt(digitalPinToInterrupt(2),CAN_int,FALLING);
}

void CAN_int(){
    unsigned char len = 0;
    unsigned char buf[8];

    CAN.readMsgBuf(&len,buf); // CAN 데이터 가져오기
    unsigned long canId = CAN.getCanId(); // CAN ID 얻기
    Serial.print("\nData from ID : 0x");
    Serial.println(canId,HEX); // 16진수로 ID 출력
    for(int i=0;i<len;i++){
        Serial.print(buf[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}

void loop(){
    Serial.println("Sending");
    unsigned char stmp[8] = {7,6,5,4,3,2,1,0};
    CAN.sendMsgBuf(0x11,0,8,stmp); // 데이터 전송
    delay(1700); // 1.7초마다 전송
}