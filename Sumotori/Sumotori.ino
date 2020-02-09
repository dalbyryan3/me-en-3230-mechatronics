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


DualVNH5019MotorShieldMod3 md(MOTOR_3_DIRECTION_A_PIN, MOTOR_3_DIRECTION_B_PIN, MOTOR_3_ENABLE_PIN, MOTOR_3_CURRENT_SENSE_PIN, MOTOR_3_SPEED_PIN, MOTOR_4_DIRECTION_A_PIN, MOTOR_4_DIRECTION_B_PIN, MOTOR_4_ENABLE_PIN, MOTOR_4_CURRENT_SENSE_PIN, MOTOR_4_SPEED_PIN);

void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module, the Xbee will use pins 16 and 17 which correspond to Serial2 on the Mega
    Serial2.begin(9600);
    
    // Initialize the two DualVNH5019 motor shields
    md.init();

    Serial.println("Setup Complete");

}

void loop()
{
    while(Serial2.available())
    {
        int speed = Serial2.read();
        Serial.print("Setting all motor speeds to: ");
        Serial.println(speed);
        md.setM1Speed(speed);
        md.setM2Speed(speed);
        md.setM3Speed(speed);
        md.setM4Speed(speed);
    }
}