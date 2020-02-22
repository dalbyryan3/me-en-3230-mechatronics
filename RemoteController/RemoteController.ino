// ME EN 3220   Mechanical-Basho Arduino Uno Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb

// Libraries to be used
#include <SoftwareSerial.h>

// Pin defintions
#define XBEE_SERIAL_RX_PIN 2
#define XBEE_SERIAL_TX_PIN 3

#define JOYSTICK_1_X_PIN A0
#define JOYSTICK_1_Y_PIN A1
#define JOYSTICK_2_Y_PIN A2

// Global state
int16_t mode = 1; // 1 is teleoperation mode
int16_t xVelocity = 0; // Will be a value between -10 and 10 (No actual units, just a relative speed value where 0 is not moving and 10 is maximum speed)
int16_t yVelocity = 0; // Will be a value between -10 and 10
int16_t servoIncrement = 0; // Will be avalue between -10 and 10 which indicates how many degrees to change the angle of the servos

SoftwareSerial xbeeSerial(XBEE_SERIAL_RX_PIN,XBEE_SERIAL_TX_PIN); // RX TX

void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module
    xbeeSerial.begin(9600);
    Serial.println("Setup Complete");

}

void loop()
{
    // delay(3000);
    if(dataRequested())
    {
        // Determine state
        // mode = mode; // IMPLEMENT READ MODE FROM MODE SWITCH
        servoIncrement = map(analogRead(JOYSTICK_2_Y_PIN), 0, 1000, -10, 10); 
        xVelocity = map(analogRead(JOYSTICK_1_X_PIN), 0, 1024, -10, 10);
        yVelocity = map(analogRead(JOYSTICK_1_Y_PIN), 0, 1024, -10, 10); 

        // motor1Speed = (motorSpeedVert + motorSpeedHorz);
        // motor3Speed = (motorSpeedVert - motorSpeedHorz);
        // if(motor1Speed > 400)
        // {
        //     motor1Speed = 400;
        // }
        // if(motor1Speed < -400)
        // {
        //     motor1Speed = -400;
        // }
        // if(motor3Speed > 400)
        // {
        //     motor3Speed = 400;
        // }
        // if(motor3Speed < -400)
        // {
        //     motor3Speed = -400;
        // }
        // motor2Speed = motor1Speed;
        // motor4Speed = motor3Speed;

        // Send state in packetized order
        xbeeSerial.write(255); // Starting byte
        // Send each state
        xbeeSerialWriteInt(mode);
        xbeeSerialWriteInt(servoIncrement);
        xbeeSerialWriteInt(xVelocity);
        xbeeSerialWriteInt(yVelocity);
        Serial.println("PACKET SENT");
        Serial.println();
    }
}

// Method will return true if the Mega is asking for the current state
bool dataRequested()
{
    // If Mega is asking for data then read the request and return true
    if(xbeeSerial.available())
    {
        xbeeSerial.read(); 
        return true;
    }
    else
    {
        return false;
    }
}

void xbeeSerialWriteInt(int16_t value)
{
    byte high = highByte(value);
    byte low = lowByte(value);
    Serial.println(high << 8 | low);
    xbeeSerial.write(high);
    xbeeSerial.write(low);
}