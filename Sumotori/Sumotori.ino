// ME EN 3220   Mechanical-Basho Arduino Mega 2560 Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb


// Libraries to be used
#include <DualVNH5019MotorShieldMod3.h>
#include <PWMServo.h>


// Pin defintions
#define MOTOR_1_DIRECTION_A_PIN 2
#define MOTOR_1_DIRECTION_B_PIN 4
#define MOTOR_1_SPEED_PIN 9
#define MOTOR_1_ENABLE_PIN 6
#define MOTOR_1_CURRENT_SENSE_PIN A0

#define MOTOR_2_DIRECTION_A_PIN 7
#define MOTOR_2_DIRECTION_B_PIN 8
#define MOTOR_2_SPEED_PIN 10
#define MOTOR_2_ENABLE_PIN 12
#define MOTOR_2_CURRENT_SENSE_PIN A1

#define MOTOR_3_DIRECTION_A_PIN 24
#define MOTOR_3_DIRECTION_B_PIN 26
#define MOTOR_3_SPEED_PIN 45
#define MOTOR_3_ENABLE_PIN 22
#define MOTOR_3_CURRENT_SENSE_PIN A3

#define MOTOR_4_DIRECTION_A_PIN 25
#define MOTOR_4_DIRECTION_B_PIN 27
#define MOTOR_4_SPEED_PIN 46
#define MOTOR_4_ENABLE_PIN 23
#define MOTOR_4_CURRENT_SENSE_PIN A4

#define SERVO_1_PIN 11
#define SERVO_2_PIN 13


// Global objects
DualVNH5019MotorShieldMod3 md(MOTOR_3_DIRECTION_A_PIN, MOTOR_3_DIRECTION_B_PIN, MOTOR_3_ENABLE_PIN, MOTOR_3_CURRENT_SENSE_PIN, MOTOR_3_SPEED_PIN, MOTOR_4_DIRECTION_A_PIN, MOTOR_4_DIRECTION_B_PIN, MOTOR_4_ENABLE_PIN, MOTOR_4_CURRENT_SENSE_PIN, MOTOR_4_SPEED_PIN);
PWMServo servo1;
PWMServo servo2;

// Global state set to inital values
int recievedData[6];
int mode = 1; // 1 is teleoperation mode
int motor1Speed = 0;
int motor2Speed = 0;
int motor3Speed = 0;
int motor4Speed = 0;

int servoPosition = 100;
int servoIncrement = 0;

long lastDataRequestTime = millis(); // milliseconds
long packetWaitTime = 100; // milliseconds

long lastServoMoveTime = 0; // milliseconds
long servoWaitTime = 50; // milliseconds


void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module, the Xbee will use pins 16 and 17 which correspond to Serial2 on the Mega
    Serial2.begin(9600);
    
    // Initialize the two DualVNH5019 motor shields
    md.init();

    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);
    servo1.write(servoPosition);
    servo2.write(servoPosition);
    delay(1000);
    servo1.detach();
    servo2.detach();

    // Setup is complete
    Serial.println("Setup Complete");

    // Ask for first state
    Serial2.write(255);
    lastDataRequestTime = millis();
    // Serial.println("ASKING FOR INITIAL DATA");
}



void loop()
{
    if(Serial2.available() > 0)
    {
        if(Serial2.read() == 255) // Extract starting byte
        {
            // Serial.println("DATA PACKET RECEIVED");
            int integersRecieved = 0;
            while (integersRecieved < 6)
            {
                if(Serial2.available() > 1)
                {
                    byte high = Serial2.read();
                    byte low = Serial2.read();
                    int reconstructedData = high << 8 | low;
                    recievedData[integersRecieved] = reconstructedData;
                    integersRecieved++;
                    // Serial.println(reconstructedData);
                }
            }
            // Serial.println();
            // IMPLEMENT UPDATE MODE
            mode = recievedData[0];

            // Ask for more state
            Serial2.write(255);
            // Serial.println("ASKING FOR DATA");
            // Set lastDataRequestTime to current time
            lastDataRequestTime = millis();
        }

        // Flush any extra data in the buffer
        while(Serial2.available() > 0)
        {
            Serial2.read();
        }
    }



    // IMPLEMENT CHECK MODE
    if(mode == 1)
    {
        servoIncrement = recievedData[1];
        motor1Speed = recievedData[2];
        motor2Speed = recievedData[3];
        motor3Speed = recievedData[4];
        motor4Speed = recievedData[5];
    }



    // WRITE CURRENT STATE TO MOTORS AND SERVOS
    // Serial.print("Setting motor 1 speed to ");
    // Serial.println(motor1Speed);
    md.setM1Speed(motor1Speed);
    // Serial.print("Setting motor 2 speed to ");
    // Serial.println(motor2Speed);
    md.setM2Speed(motor2Speed);
    // Serial.print("Setting motor 3 speed to ");
    // Serial.println(motor3Speed);
    md.setM3Speed(motor3Speed);
    // Serial.print("Setting motor 4 speed to ");
    // Serial.println(motor4Speed);
    md.setM4Speed(motor4Speed);

    if((millis() - lastServoMoveTime) > servoWaitTime)
    {
        servoPosition += servoIncrement;
        if(servoPosition > 180)
        {
            servoPosition = 180;
        }
        if(servoPosition < 100)
        {
            servoPosition = 100;
        }
        // if(servoPosition > 180)
        // {
        //     servoPosition = 180;
        // }
        // if(servoPosition < 0)
        // {
        //     servoPosition = 0;
        // }
        servo1.attach(SERVO_1_PIN);
        servo2.attach(SERVO_2_PIN);
        servo1.write(servoPosition);
        servo2.write(map(servoPosition, 100, 180, 180, 100));
        delay(100);
        servo1.detach();
        servo2.detach();
        Serial.println(servo1.read());
        Serial.println(servo2.read());
        Serial.println();
        lastServoMoveTime = millis();

    }





    // If no new state has been recieved for a while then ask again
    if ((millis() - lastDataRequestTime) > packetWaitTime)
    {
        Serial2.write(255);
        lastDataRequestTime = millis();
        // Serial.println("ASKING FOR DATA BECAUSE OF WAIT TIME");
    }

}


