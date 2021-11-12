#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#define SERVO1_PIN 18
#define SERVO2_PIN 23
#define SERVO3_PIN 17
#define SERVO4_PIN 16
#define SERVO5_PIN 22
#define SERVO6_PIN 15
#define SERVO7_PIN 21


class Joint
{
private:
    Servo servo;
    byte pin;
    int minAngle;
    int maxAngle;
    int startAngle;
    int angleOffset;
    int currentAngle;

public:
    Joint(byte pin, int minAngle = 0, int maxAngle = 180, int startAngle = 90, int angleOffset = 0)
    {
        this->pin = pin;
        this->minAngle = minAngle;
        this->maxAngle = maxAngle;
        this->startAngle = startAngle;
        this->angleOffset = angleOffset;
        this->currentAngle = startAngle + angleOffset;
        // servo.attach(pin, 500, 2400);
    }

    void init()
    {
        servo.attach(pin, 500, 2400);
        rotateJoint(currentAngle);
    }

    int getCurrentAngle()
    {
        return currentAngle;
    }

    void rotateJoint(int angle)
    {
        angle = constrain(angle, minAngle, maxAngle);
        servo.write(angle);
    }

    bool rotateJointWithStep(int angle, int step)
    {
        angle = constrain(angle, minAngle, maxAngle);
        if (angle > currentAngle)
        {
            Serial.println("+" + String(currentAngle));
            if ((currentAngle + step) > angle)
            {
                Serial.println("++" + String(currentAngle));
                currentAngle++;
            }
            else
            {
                Serial.println("+++" + String(currentAngle));
                currentAngle += step;
            }
        }
        else if (angle < currentAngle)
        {
            Serial.println("-" + String(currentAngle));
            if ((currentAngle - step) < angle)
            {
                Serial.println("--" + String(currentAngle));
                currentAngle--;
            }
            else
            {
                Serial.println("---" + String(currentAngle));
                currentAngle -= step;
            }
        }
        else
        {
            return true;
        }
        servo.write(currentAngle);
        return false;
    }
};

class Joint2M
{
private:
    Servo servo1;
    Servo servo2;
    byte pin1;
    byte pin2;
    int minAngle;
    int maxAngle;
    int startAngle;
    int angleOffset;
    int currentAngle;

public:
    Joint2M(byte pin1, byte pin2, int minAngle = 0, int maxAngle = 180, int startAngle = 90, int angleOffset = 0)
    {
        this->pin1 = pin1;
        this->pin2 = pin2;
        this->minAngle = minAngle;
        this->maxAngle = maxAngle;
        this->startAngle = startAngle;
        this->angleOffset = angleOffset;
        this->currentAngle = startAngle + angleOffset;
    }

    void init()
    {
        servo1.attach(pin1, 500, 2400);
        servo2.attach(pin2, 500, 2400);
        rotateJoint(currentAngle);
    }

    int getCurrentAngle()
    {
        return currentAngle;
    }

    void rotateJoint(int angle)
    {
        angle = constrain(angle, minAngle, maxAngle);
        servo1.write(angle);
        servo2.write(180 - angle);
    }

    bool rotateJointWithStep(int angle, int step)
    {
        angle = constrain(angle, minAngle, maxAngle);
        if (angle > currentAngle)
        {
            Serial.println("+" + String(currentAngle));
            if ((currentAngle + step) > angle)
            {
                Serial.println("++" + String(currentAngle));
                currentAngle++;
            }
            else
            {
                Serial.println("+++" + String(currentAngle));
                currentAngle += step;
            }
        }
        else if (angle < currentAngle)
        {
            Serial.println("-" + String(currentAngle));
            if ((currentAngle - step) < angle)
            {
                Serial.println("--" + String(currentAngle));
                currentAngle--;
            }
            else
            {
                Serial.println("---" + String(currentAngle));
                currentAngle -= step;
            }
        }
        else
        {
            return true;
        }
        servo1.write(angle);
        servo2.write(180 - angle);
        return false;
    }
};


int servo1Angle = 90, servo23Angle = 20, servo4Angle = 160, servo5Angle = 90, servo6Angle = 90, servo7Angle = 90, speed = 75;
BluetoothSerial SerialBT;
Joint joint1(SERVO1_PIN, 0, 180, 90, -5);
Joint2M joint2(SERVO2_PIN, SERVO3_PIN, 20, 160, 20, 0);
Joint joint3(SERVO4_PIN, 40, 170, 160, 0);
Joint joint4(SERVO5_PIN, 0, 180, 90, 0);
Joint joint5(SERVO6_PIN, 0, 180, 90, -10);
Joint joint6(SERVO7_PIN, 0, 180, 90, -10);

// void rotateJoint1(int angle)
// {
//     angle = constrain(angle, 0, 180);
//     servo1.write(angle);
// }

// void rotateJoint2(int angle)
// {
//     angle = constrain(angle, 20, 160);
//     servo2.write(angle);
//     servo3.write(180 - angle);
// }

// void rotateJoint3(int angle)
// {
//     angle = constrain(angle, 40, 170);
//     servo4.write(angle);
// }

// void rotateJoint4(int angle)
// {
//     angle = constrain(angle, 0, 180);
//     servo5.write(angle);
// }

// void rotateJoint5(int angle)
// {
//     angle = constrain(angle, 0, 180);
//     servo6.write(angle);
// }

// void rotateJoint6(int angle)
// {
//     angle = constrain(angle, 0, 180);
//     servo7.write(angle);
// }

// // void initPose()
// // {
// //   rotateJoint1(90);
// //   rotateJoint2(20);
// //   rotateJoint3(90);
// //   rotateJoint4(90);
// //   rotateJoint5(90);
// //   rotateJoint6(90);
// // }

// void setJointAngles()
// {
//     rotateJoint1(servo1Angle - servo1AngleOffset);
//     rotateJoint2(servo23Angle - servo23AngleOffset);
//     rotateJoint3(servo4Angle - servo4AngleOffset);
//     rotateJoint4(servo5Angle - servo5AngleOffset);
//     rotateJoint5(servo6Angle - servo6AngleOffset);
//     rotateJoint6(servo7Angle - servo7AngleOffset);
// }


void initJoints()
{
    joint1.init();
    delay(2000);
    joint2.init();
    delay(2000);
    joint3.init();
    delay(2000);
    joint4.init();
    delay(2000);
    joint5.init();
    delay(2000);
    joint6.init();
}


void rotateJoints()
{
    joint1.rotateJointWithStep(servo1Angle, 1);
    joint2.rotateJointWithStep(servo23Angle, 1);
    joint3.rotateJointWithStep(servo4Angle, 1);
    joint4.rotateJointWithStep(servo5Angle, 1);
    joint5.rotateJointWithStep(servo6Angle, 1);
    joint6.rotateJointWithStep(servo7Angle, 1);
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
        speed = Serial.parseInt();
        Serial.println("Angles received: " + String(servo1Angle) + " " + String(servo23Angle) + " " +
                       String(servo4Angle) + " " + String(servo5Angle) + " " + String(servo6Angle) + " " + String(servo7Angle) + " " + String(speed));
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
        speed = SerialBT.parseInt();
        SerialBT.println("Angles received: " + String(servo1Angle) + " " + String(servo23Angle) + " " +
                         String(servo4Angle) + " " + String(servo5Angle) + " " + String(servo6Angle) + " " + String(servo7Angle) + " " + String(speed));
        SerialBT.flush();
    }
}

void setup()
{
    Serial.begin(9600);
    SerialBT.begin("ESP32RobotArm6Dof");
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    initJoints();
    Serial.println("Setup complete");
}

void loop()
{
    getJointAnglesFromBT();
    static unsigned long previousMillis = 0;
    if (millis() - previousMillis >= speed)
    {
        previousMillis = millis();
        rotateJoints();
    }
    delay(1);
}
