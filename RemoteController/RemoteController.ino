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
int mode = 1; // 1 is teleoperation mode
int motor1Speed = -80;
int motor2Speed = 200;
int motor3Speed = 400;
int motor4Speed = -400;
int servoValue = 205; 

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
        // servoValue = map(analogRead(JOYSTICK_2_Y_PIN), 0, 1024, 0, 255); // Will be written using PWM to servos, Arduino only has 0 to 255 range for PWM
        motor1Speed = map(analogRead(JOYSTICK_1_Y_PIN), 0, 1024, -400, 400); // Will be written using the motor driver library directly to the motor with a speed between -400 and 400
        motor2Speed = motor1Speed;
        motor3Speed = motor1Speed;
        motor4Speed = motor1Speed;

        // Send state in packetized order

        xbeeSerial.write(255); // Starting byte
        // Send each state
        xbeeSerialWriteInt(mode);
        xbeeSerialWriteInt(servoValue);
        xbeeSerialWriteInt(motor1Speed);
        xbeeSerialWriteInt(motor2Speed);
        xbeeSerialWriteInt(motor3Speed);
        xbeeSerialWriteInt(motor4Speed);
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

void xbeeSerialWriteInt(int value)
{
    byte high = highByte(value);
    byte low = lowByte(value);
    Serial.println(high << 8 | low);
    xbeeSerial.write(high);
    xbeeSerial.write(low);
}