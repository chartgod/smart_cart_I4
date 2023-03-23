//#include <ros.h>
//#include <std_msgs/Float64.h>

//ros::NodeHandle nh;
//std_msgs::Float64 Distance;
//ros::Publisher chatter("chatter",&Distance);

const int trigPin = 6;
const int echoPin = 7;

const int red = 13;
const int grn = 12;

  long duration;
  float distance;

void setup() {
//  nh.initNode();
//  nh.advertise(chatter);
  
  Serial.begin(9600);       // 시리얼 속도 설정

  pinMode(echoPin, INPUT);   // echoPin 입력
  pinMode(trigPin, OUTPUT);  // trigPin 출력
  
  pinMode(red, OUTPUT);
  pinMode(grn, OUTPUT);

}

void loop() {

  digitalWrite(trigPin, HIGH);                        // trigPin에서 초음파 발생(echoPin도 HIGH)

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);                  // echoPin 이 HIGH를 유지한 시간을 저장 한다.

  distance = ((float)(340 * duration) / 10000) / 2; 
  
//  Distance.data = distance;
//  chatter.publish(&Distance);
//  nh.spinOnce();
  
  Serial.print("Duration:");
  Serial.print(duration);
  Serial.print("\nDistance:");                          // 물체와 초음파 센서간 거리를 표시
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 10){
    digitalWrite(red, HIGH);
    digitalWrite(grn, LOW);
  }
  else{
    digitalWrite(red, LOW);
    digitalWrite(grn, HIGH);
  }
  //delay(500);


  //delay(200);

}

