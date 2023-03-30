#include <SoftwareSerial.h>
#include <string.h>

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String Str0;
ros::Publisher chatter("chatter",&Str0);

int rssi = 0;
int i = 0;
//char list1[200];
String list = "";
const char * str1 = "RSSI:";
const char * str2 = "태우의 S20 Ultra";

SoftwareSerial BTserial(3, 2); // RX, TX 핀 설정


void setup() {
  nh.initNode();
  nh.advertise(chatter);
  
  Serial.begin(9600);
  BTserial.begin(9600);
  BTserial.println("AT+DISC?"); // AT 명령 모드에서 RSSI 값을 측정하는 명령어 전송
}

void loop() {
  
  if (BTserial.available()) { // HM-10 모듈로부터 응답을 받을 때까지 기다림
    char c = BTserial.read();
    Serial.write(c);
    list += c;
    //list = BTserial.read();
    i++;    
  }
  //i = 0;
  //String str = list1;
  //const char * str3 = list;
  
  //int index = list.indexOf("S20 Ultra",0); // 수정중*****************************************
  //if (index > 1) {
  //  Serial.println(list[9,index]);
  //}
  
  
  //Serial.find('+');
    
  
  //Str0.data = str3;
  //chatter.publish(&Str0);
  //nh.spinOnce();

  if (Serial.available()){
    BTserial.write(Serial.read());
  }
}
