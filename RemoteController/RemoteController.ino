// ME EN 3220   Mechanical-Basho Arduino Uno Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb

// Libraries to be used
#include <SoftwareSerial.h>

// Pin defintions
static const int XBEE_SERIAL_RX_PIN = 2;
static const int XBEE_SERIAL_TX_PIN = 3;
static const int JOYSTICK_1_X_PIN = A0;
static const int JOYSTICK_1_Y_PIN = A1;
static const int JOYSTICK_2_Y_PIN = A2;

// Global objects
SoftwareSerial xbeeSerial(XBEE_SERIAL_RX_PIN,XBEE_SERIAL_TX_PIN); // RX TX

// Global state set to initial values
int16_t mode = 1; // 1 is teleoperation mode
int16_t xVelocity = 0; // Will be a value between -10 and 10 (No actual units, just a relative speed value where 0 is not moving and 10 is maximum speed)
int16_t yVelocity = 0; // Will be a value between -10 and 10
int16_t servoIncrement = 0; // Will be avalue between -10 and 10 which indicates how many degrees to change the angle of the servos



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
        // Determine state remote controller 
        // mode = mode; // IMPLEMENT READ MODE FROM MODE SWITCH
        servoIncrement = map(analogRead(JOYSTICK_2_Y_PIN), 0, 1000, -10, 10); 
        xVelocity = map(analogRead(JOYSTICK_1_X_PIN), 0, 1024, -10, 10);
        yVelocity = map(analogRead(JOYSTICK_1_Y_PIN), 0, 1024, -10, 10); 

        // Send state in packetized order
        // Will be sending 4 signed integers of data, mode, servoIncrement, xVelocity, and yVelocity
        // [{start byte}, {mode byte 1}{mode byte 2}, {servo increment byte 1}{servo increment byte 2}, {xVelocity byte 1}{xVelocity byte 2}, {yVelocity byte 1}{yVelocity byte 2}]
        xbeeSerial.write(255); // Starting byte
        // Send each state
        xbeeSerialWriteInt(mode);
        xbeeSerialWriteInt(servoIncrement);
        xbeeSerialWriteInt(xVelocity);
        xbeeSerialWriteInt(yVelocity);
        // Serial.println("PACKET SENT");
        // Serial.println();
    }
}



// Methods which help abstract code above

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