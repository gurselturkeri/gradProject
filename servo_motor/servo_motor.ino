
#include <Servo.h>

Servo myservo;


int pos = 0;  

void setup(){
Serial.begin(115200);
  Serial.setTimeout(1);  
  myservo.attach(7);  
}

void loop() {
  while (!Serial.available());
  pos = Serial.readString().toInt();
  myservo.write(pos);              
  Serial.print(pos);
}
