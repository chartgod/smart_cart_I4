#include <SoftwareSerial.h>
#include <string.h>
//**********ros**********
#include <ros.h>
#include <std_msgs/Float64.h>
ros::NodeHandle nh;
std_msgs::Float64 Distance;
ros::Publisher chatter("chatter", &Distance);
//***********************
float rssi = 0;
String list = "";
const char * str1 = "RSSI:";
const char * str2 = "태우의 S20 Ultra";
int index_n = 0;

//필터
int N = 0;
float rssiN = 0;
float rssi_n1 = 0;
float rssi_n2 = 0;
float a = 0.5;
float N1 = 0;
float N2 = 0;

//거리
float txpower = -59; //def:-59 비콘이 1m 에서 측정되는 RSSI 값
float n = 4; //경로 손실 지수 2~4 주변장애물 많으면 4
float d = 0; //거리

SoftwareSerial I4_HM10_1(3, 2); // RX, TX 핀 설정 I4_HM10_1

void setup() {
  //**********ros**********
  nh.initNode();
  nh.advertise(chatter);
  //***********************  
  Serial.begin(9600);
  I4_HM10_1.begin(9600);
  I4_HM10_1.println("AT+DISC?"); // AT 명령 모드에서 RSSI 값을 측정하는 명령어 전송
  Serial.println("Start!");
}

void loop() {
  
  string();
  
  if (Serial.available()){
    I4_HM10_1.write(Serial.read());
  }

  //**********ros**********
  Distance.data = d;
  chatter.publish(&Distance);
  nh.spinOnce();
  delay(10);  
  //***********************
}

void string(){
  while(I4_HM10_1.available()){
    if (I4_HM10_1.available()) { // HM-10 모듈로부터 응답을 받을 때까지 기다림
      char c = I4_HM10_1.read();
      Serial.write(c); //시리얼창 모니터링
      list += c; //모듈에서 읽어오는 문자 list에 추가
    }
    
    index_n = list.indexOf("태우의 S20 Ultra",0);//list내에 문자열"태우의 S20 Ultra"의 위치, 인덱스 검출
    if (index_n > 1 && list.indexOf("OK+DISCE",index_n)>-1){
      Serial.println("");
      Serial.println(" -> done!");
      //Serial.print("RSSI:");     Serial.print(list[index_n-14]);      Serial.print(list[index_n-13]);      Serial.print(list[index_n-12]);      Serial.println(list[index_n-11]);
      
      distance(); //rssi자료형 변환 및 거리 계산 함수
     
      list = "";
      I4_HM10_1.println("AT+DISC?");
    }
    if (list.indexOf("OK+DISCE",0)>-1){
      list = "";
      Serial.println("");
      Serial.println("-> Rescan!");
      I4_HM10_1.println("AT+DISC?");
    }       
  }
}

void distance(){
  rssi = (list[index_n-13]-'0') * 100; //아스키코드 0~9 순서대로 1씩 증가하는 형태 이므로 '0'을 뺀다
  rssi += (list[index_n-12]-'0') * 10;
  rssi += (list[index_n-11]-'0');
  rssi = -rssi;
  Serial.print("RSSI:");
  Serial.println(rssi);
  //RSSI Feedback 필터링
  if (N == 1) {
    rssi_n2 = rssi;
    rssiN = (a*(rssi_n2)) + ((1-a)*(rssi_n1));
    rssi = rssiN;    
  }
  if (N == 0){ 
    N = 1;
  }
  
  // 거리 계산
  d = pow(10,((txpower-rssi)/(10*n))); //pow 10^N 거듭 제곱
  Serial.print("거리: ");
  Serial.print(d);
  Serial.println("m");
  N = 2;
  if(N == 2){
    rssi_n1 = rssi; 
    N = 1;
  }
}
