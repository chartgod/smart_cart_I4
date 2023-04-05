int red = 4;
int yellow = 2;
int blue = 5;
int trig = 7;
int echo = 6;
int buzzer= 8;
long duration;
float distance;

void setup() {
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(yellow, OUTPUT);
}

void loop() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  duration = pulseIn(echo, HIGH);

  delay(1000);
  distance = duration / 29 / 2;
  
  Serial.print(" 거리: ");
  Serial.print(distance);
  Serial.println("cm");
  delay(1000);

  if (distance <= 10) {
    tone(8,100,10);    //수동부저
    Serial.print("위험합니다!! -> ");
    digitalWrite(blue,LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(red, HIGH);
  }
  else if(distance <= 20) {
    tone(8,100,10);
    digitalWrite(blue, LOW);
    digitalWrite(red,LOW);
    digitalWrite(yellow, HIGH);
  }
  else{
    noTone(buzzer);       //수동부저
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(blue, HIGH);
  }
}
