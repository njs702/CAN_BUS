// 3대의 단말이 하나의 BUS LINE으로 통신하는 프로그램 연습

#include <SPI.h>
#include <mcp_can.h>

MCP_CAN CAN(10);
int led_pin = 7;
int btn_pin = 6;

void CAN_INT(){
    unsigned char len = 0;
    unsigned char buf[8];


    CAN.readMsgBuf(&len,buf); // CAN 데이터 가져오기
    unsigned long canId = CAN.getCanId(); // CAN ID 얻기
    switch (canId)
    {
    case 0x90:
        Serial.print("\nData from ID : 0x");
        Serial.println(canId,HEX); // 16진수로 ID 출력
        for(int i=0;i<len;i++){
            Serial.print(buf[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
        digitalWrite(led_pin,HIGH);
        delay(1000);
        digitalWrite(led_pin,LOW);
        break;
    
    case 0x92:
        Serial.print("\nData from ID : 0x");
        Serial.println(canId,HEX); // 16진수로 ID 출력
        for(int i=0;i<len;i++){
            Serial.print(buf[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
        digitalWrite(led_pin,HIGH);
        delay(1000);
        digitalWrite(led_pin,LOW);
        break;
    
    case 0x93:
        Serial.print("\nData from ID : 0x");
        Serial.println(canId,HEX); // 16진수로 ID 출력
        for(int i=0;i<len;i++){
            Serial.print(buf[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
        digitalWrite(led_pin,HIGH);
        delay(1000);
        digitalWrite(led_pin,LOW);
        break;
    
    default:
        break;
    }
    
}

void setup(){
    Serial.begin(115200);
    pinMode(led_pin,OUTPUT);
    pinMode(btn_pin,INPUT);
    while(CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)){
        Serial.println("CAN BUS init failed!");
        delay(100);
    }
    Serial.println("CAN BUS init Success!");
    // 2번 핀 인터럽트 설정(falling 때), 인터럽트 들어오면 CAN_int 함수 실행
    attachInterrupt(digitalPinToInterrupt(2),CAN_INT,FALLING);
}

void loop(){
    unsigned char data[8] = {7,6,5,4,3,2,1,0};
    
    if(digitalRead(btn_pin)==HIGH){
        CAN.sendMsgBuf(0x91,0,8,data);
        CAN.sendMsgBuf(0x90,0,8,data);
        Serial.println("Button pushed, Data send");
        delay(1000);
    }
    
    // delay(1000);
}