#include <SoftwareSerial.h>
#include <string.h>

//#include <ros.h>
//#include <std_msgs/String.h>

//ros::NodeHandle nh;
//std_msgs::String Str0;
//ros::Publisher chatter("chatter",&Str0);

int rssi = 0;
String list = "";
const char * str1 = "RSSI:";
const char * str2 = "태우의 S20 Ultra";
int index_n = 0;
SoftwareSerial BTserial(3, 2); // RX, TX 핀 설정


void setup() {
  //nh.initNode();
  //nh.advertise(chatter);
  
  Serial.begin(9600);
  BTserial.begin(9600);
  BTserial.println("AT+DISC?"); // AT 명령 모드에서 RSSI 값을 측정하는 명령어 전송
}

void loop() {
  
  string();
  
  if (Serial.available()){
    BTserial.write(Serial.read());
  }
}

void string(){
  while(BTserial.available()){
    if (BTserial.available()) { // HM-10 모듈로부터 응답을 받을 때까지 기다림
      char c = BTserial.read();
      //Serial.write(c); //시리얼창 모니터링
      list += c; //모듈에서 읽어오는 문자 list에 추가
    }
    
    // 수정중*****************************************
    index_n = list.indexOf("태우의 S20 Ultra",0);//list내에 문자열"태우의 S20 Ultra"의 위치, 인덱스 검출
    if (index_n > 1 && list.indexOf("OK+DISCE",index_n)>-1){
      Serial.println("");
      Serial.println(" -> done!");
      Serial.print("RSSI:");
      Serial.print(list[index_n-14]);
      Serial.print(list[index_n-13]);
      Serial.print(list[index_n-12]);
      Serial.print(list[index_n-11]);
      list = "";
      BTserial.println("AT+DISC?");
    }
    if (list.indexOf("OK+DISCE",0)>-1){
      list = "";
      Serial.println("");
      Serial.println("-> Rescan!");
      BTserial.println("AT+DISC?");
    }
    
    //Str0.data = c;
    //chatter.publish(&Str0);
    //nh.spinOnce();    
  }
}
