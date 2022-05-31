#include <SPI.h>
#include <mcp_can.h>

// spi cs핀 연결 포트 : 10
MCP_CAN CAN(10);
int btn_pin = 7;

void setup(){
    Serial.begin(115200);
    while(CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)){
        Serial.println("CAN BUS init Failed");
        delay(100);
    }
    Serial.println("CAN BUS Init Success");

    // 2번 핀 인터럽트 설정(falling 때), 인터럽트 들어오면 CAN_int 함수 실행
    attachInterrupt(digitalPinToInterrupt(2),CAN_int,FALLING);

    // 7번 핀 INPUT MODE로 설정
    pinMode(btn_pin,INPUT);
}

void CAN_int(){
    unsigned char len = 0;
    unsigned char buf[8];

    CAN.readMsgBuf(&len,buf); // CAN 데이터 가져오기
    unsigned long cnaId = CAN.getCanId(); // CAN ID 얻기
    Serial.print("\nData from ID : 0x");
    Serial.println(cnaId); // 16진수로 ID 출력
    for(int i=0;i<len;i++){
        Serial.print(buf[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}

void loop(){
    unsigned char stmp[8] = {0,1,2,3,4,5,6,7};
    if(digitalRead(btn_pin) == HIGH){
        Serial.println("Sending(from right)");
        CAN.sendMsgBuf(0x11,0,8,stmp); // 데이터 전송
        delay(1000);
    }
    
     // delay(1700); // 1.7초마다 전송
}