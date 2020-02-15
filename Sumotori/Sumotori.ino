// ME EN 3220   Mechanical-Basho Arduino Mega 2560 Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb


// Libraries to be used
#include <DualVNH5019MotorShieldMod3.h>


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


// Global objects
DualVNH5019MotorShieldMod3 md(MOTOR_3_DIRECTION_A_PIN, MOTOR_3_DIRECTION_B_PIN, MOTOR_3_ENABLE_PIN, MOTOR_3_CURRENT_SENSE_PIN, MOTOR_3_SPEED_PIN, MOTOR_4_DIRECTION_A_PIN, MOTOR_4_DIRECTION_B_PIN, MOTOR_4_ENABLE_PIN, MOTOR_4_CURRENT_SENSE_PIN, MOTOR_4_SPEED_PIN);


// Global state set to inital values
int recievedData[6];
int mode = 1; // 1 is teleoperation mode
int motor1Speed = 0;
int motor2Speed = 0;
int motor3Speed = 0;
int motor4Speed = 0;
int servoValue = 0;

long timeSinceLastPacket = millis(); // millis
long packetWaitTime = 50000; // millis



void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module, the Xbee will use pins 16 and 17 which correspond to Serial2 on the Mega
    Serial2.begin(9600);
    
    // Initialize the two DualVNH5019 motor shields
    md.init();

    // Setup is complete
    Serial.println("Setup Complete");

    // Ask for first state
    Serial2.write(255);
    // Serial.println("ASKING FOR INITIAL DATA");
}

void loop()
{
    if(Serial2.available())
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
            // Set timeSinceLastPacket to current time
            timeSinceLastPacket = millis();
        }
    }

    // IMPLEMENT CHECK MODE
    if(mode == 1)
    {
        servoValue = recievedData[1];
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

    // If no new state has been recieved for a while then ask again
    // if (timeSinceLastPacket - millis() > packetWaitTime)
    // {
    //     Serial2.write(255);
    //     Serial.println("ASKING FOR DATA BECAUSE OF WAIT TIME");
    // }

}


