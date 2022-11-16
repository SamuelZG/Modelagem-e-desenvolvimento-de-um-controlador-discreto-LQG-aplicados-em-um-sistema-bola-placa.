#include <Servo.h>

const int SERVO1_PIN = 9;   //x
const int SERVO2_PIN = 10;  //y

Servo servo1;
Servo servo2;

String inData; 

float servo1_angle = 90; 
float servo2_angle = 90;

void setup() 
{
  Serial.begin(115200);
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);

  servo1.writeMicroseconds((servo1_angle * 1500)/90);//servo1.writeMicroseconds((servo1_angle * 1500)/90);
  servo2.writeMicroseconds((servo2_angle * 1500)/90);
} 
 
void loop() 
{
  if (Serial.available() > 0) {

    inData = Serial.readStringUntil(':');
    servo1_angle = inData.toInt();

    inData = Serial.readStringUntil('$');
    servo2_angle = inData.toInt();

    servo1.writeMicroseconds((servo1_angle * 1500)/90);
    servo2.writeMicroseconds((servo2_angle * 1500)/90);
    
    //servo1.write((servo1_angle));
    //servo2.write((servo2_angle));


  }
}
