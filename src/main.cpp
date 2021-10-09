#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#define SERVO1_PIN 18
#define SERVO2_PIN 23
#define SERVO3_PIN 17
#define SERVO4_PIN 16
#define SERVO5_PIN 22
#define SERVO6_PIN 15
#define SERVO7_PIN 21

Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7;
BluetoothSerial SerialBT;
int servo1Angle = 90, servo23Angle = 20, servo4Angle = 160, servo5Angle = 90, servo6Angle = 90, servo7Angle = 90;
int servo1AngleOffset = 5, servo23AngleOffset = 0, servo4AngleOffset = 0, servo5AngleOffset = 0, servo6AngleOffset = 10, servo7AngleOffset = 10;

void rotateJoint1(int angle)
{
  angle = constrain(angle, 0, 180);
  servo1.write(angle);
}

void rotateJoint2(int angle)
{
  angle = constrain(angle, 20, 160);
  servo2.write(angle);
  servo3.write(180 - angle);
}

void rotateJoint3(int angle)
{
  angle = constrain(angle, 40, 170);
  servo4.write(angle);
}

void rotateJoint4(int angle)
{
  angle = constrain(angle, 0, 180);
  servo5.write(angle);
}

void rotateJoint5(int angle)
{
  angle = constrain(angle, 0, 180);
  servo6.write(angle);
}

void rotateJoint6(int angle)
{
  angle = constrain(angle, 0, 180);
  servo7.write(angle);
}

// void initPose()
// {
//   rotateJoint1(90);
//   rotateJoint2(20);
//   rotateJoint3(90);
//   rotateJoint4(90);
//   rotateJoint5(90);
//   rotateJoint6(90);
// }

void setJointAngles()
{
  rotateJoint1(servo1Angle - servo1AngleOffset);
  rotateJoint2(servo23Angle - servo23AngleOffset);
  rotateJoint3(servo4Angle - servo4AngleOffset);
  rotateJoint4(servo5Angle - servo5AngleOffset);
  rotateJoint5(servo6Angle - servo6AngleOffset);
  rotateJoint6(servo7Angle - servo7AngleOffset);
}

void getJointAnglesFromSerial()
{
  if (Serial.available())
  {
    servo1Angle = Serial.parseInt();
    servo23Angle = Serial.parseInt();
    servo4Angle = Serial.parseInt();
    servo5Angle = Serial.parseInt();
    servo6Angle = Serial.parseInt();
    servo7Angle = Serial.parseInt();
    Serial.println("Angles received: " + String(servo1Angle) + " " + String(servo23Angle) + " " +
                   String(servo4Angle) + " " + String(servo5Angle) + " " + String(servo6Angle) + " " + String(servo7Angle));
    Serial.flush();
  }
}

void getJointAnglesFromBT()
{
  if (SerialBT.available())
  {
    servo1Angle = SerialBT.parseInt();
    servo23Angle = SerialBT.parseInt();
    servo4Angle = SerialBT.parseInt();
    servo5Angle = SerialBT.parseInt();
    servo6Angle = SerialBT.parseInt();
    servo7Angle = SerialBT.parseInt();
    SerialBT.println("Angles received: " + String(servo1Angle) + " " + String(servo23Angle) + " " +
                     String(servo4Angle) + " " + String(servo5Angle) + " " + String(servo6Angle) + " " + String(servo7Angle));
    SerialBT.flush();
  }
}

void setup()
{
  Serial.begin(9600);
  SerialBT.begin("ESP32RobotArm6Dof"); //Bluetooth device name
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);            // Standard 50hz servo
  servo1.attach(SERVO1_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object
                                        // using SG90 servo min/max of 500us and 2400us
                                        // for MG995 large servo, use 1000us and 2000us,
                                        // which are the defaults, so this line could be
                                        // "myservo.attach(servoPin);"
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo3.attach(SERVO3_PIN, 500, 2400);
  servo4.attach(SERVO4_PIN, 500, 2400);
  servo5.attach(SERVO5_PIN, 500, 2400);
  servo6.attach(SERVO6_PIN, 500, 2400);
  servo7.attach(SERVO7_PIN, 500, 2400);
  // initPose();
  Serial.println("Setup complete");
}

void loop()
{
  setJointAngles();
  getJointAnglesFromBT();
  getJointAnglesFromSerial();
}
