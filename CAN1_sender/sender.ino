#include <SPI.h>
#include <mcp_can.h>
#include <dht.h>

typedef struct{
    float humid; // store humidity value
    float temp; // store temperature value
}temp_humid_data;

union temp_humid_union {
    temp_humid_data first;
    unsigned char second[8];
};

MCP_CAN CAN(10);
static temp_humid_data temp_humid;

dht DHT;
int DHT_11_PIN = 3;

void CAN_int(){
    unsigned char len = 0;
    unsigned char buf[8];

    CAN.readMsgBuf(&len,buf); // CAN 데이터 가져오기
    unsigned long cnaId = CAN.getCanId(); // CAN ID 얻기
    Serial.print("\nData from ID : 0x");
    Serial.println(cnaId,HEX); // 16진수로 ID 출력
    for(int i=0;i<len;i++){
        Serial.print(buf[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}

void setup(){
    Serial.begin(115200);
    while(CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)){
        Serial.println("CAN BUS init failed!");
        delay(100);
    }
    Serial.println("CAN BUS init Success!");
    // 2번 핀 인터럽트 설정(falling 때), 인터럽트 들어오면 CAN_int 함수 실행
    attachInterrupt(digitalPinToInterrupt(2),CAN_int,FALLING);
}

void loop(){
    //Serial.println("Sending");
    DHT.read11(DHT_11_PIN);
    temp_humid.humid = DHT.humidity;
    temp_humid.temp = DHT.temperature;

    temp_humid_union a;
    a.first = temp_humid;

    unsigned char stmp[8];

    for (int i = 0; i < 8; ++i)
        stmp[i] = a.second[i];

    CAN.sendMsgBuf(0x11,0,8,stmp);
    delay(1000);
}