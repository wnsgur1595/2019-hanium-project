#include <SoftwareSerial.h>
#define trig 8    // 트리거 핀 선언
#define echo 9    // 에코 핀 선언

SoftwareSerial hc06(2,3); //Tx, Rx

long distance2 = 0;
char x = '2', y = '3'; // 이 주차공간의 좌표

void setup()
{
   Serial.begin(9600);
   Serial.println("초음파 센서 시작");
   hc06.begin(9600);

   pinMode(trig, OUTPUT);    // 트리거 핀 출력으로 선언
   pinMode(echo, INPUT);     // 에코 핀 입력으로 선언
}
void loop()
{
   long duration, distance;    // 거리 측정을 위한 변수 선언
   bool Parked = false;
   // 트리거 핀으로 10us 동안 펄스 출력
   digitalWrite(trig, LOW);        // Trig 핀 Low
   delayMicroseconds(2);            // 2us 딜레이
   digitalWrite(trig, HIGH);    // Trig 핀 High
   delayMicroseconds(10);            // 10us 딜레이
   digitalWrite(trig, LOW);        // Trig 핀 Low
 
    /*
        에코핀에서 받은 펄스 값을 pulseIn함수를 호출하여
        펄스가 입력될 때까지의 시간을 us단위로 duration에 저장
        pulseln() 함수는 핀에서 펄스(HIGH or LOW)를 읽어서 마이크로초 단위로 반환
   */
 
   duration = pulseIn(echo, HIGH);  
 
   /*
        음파의 속도는 초당 340m, 왕복하였으니 나누기 2를하면 170m이고,
        mm단위로 바꾸면 170,000mm.
        duration에 저장된 값은 us단위이므로 1,000,000으로 나누어 주고,
        정리해서 distance에 저장 
   */
 
   distance = duration * 170 / 1000;

   if(distance < 500){
    distance2 = distance;
   }
   else if(distance2 < 500){
    hc06.write(x);
    hc06.write(y);
    distance2 = distance;
   }
   delay(1000);
}
