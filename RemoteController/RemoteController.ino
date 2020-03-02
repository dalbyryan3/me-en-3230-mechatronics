// ME EN 3220   Mechanical-Basho Arduino Uno Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb

// Libraries to be used
#include <SoftwareSerial.h>

// Pin defintions
static const uint8_t XBEE_SERIAL_RX_PIN = 2;
static const uint8_t XBEE_SERIAL_TX_PIN = 3;
static const uint8_t JOYSTICK_1_X_PIN = A0;
static const uint8_t JOYSTICK_1_Y_PIN = A1;
static const uint8_t JOYSTICK_1_BUTTON_PIN = 4;
static const uint8_t JOYSTICK_2_Y_PIN = A2;
static const uint8_t JOYSTICK_2_BUTTON_PIN = 5;
static const uint8_t NEUTRAL_BUTTON_PIN = 6;

// Global objects
SoftwareSerial xbeeSerial(XBEE_SERIAL_RX_PIN,XBEE_SERIAL_TX_PIN); // RX TX

// Global state set to initial values
int16_t mode = 3; // 3 is neutral mode
int16_t xVelocity = 0; // Will be a value between -100 and 100 (No actual units, just a relative speed value where 0 is not moving and 10 is maximum speed)
int16_t yVelocity = 0; // Will be a value between -100 and 100
int16_t servoIncrement = 0; // Will be avalue between -10 and 10 which indicates how many degrees to change the angle of the servos



void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module
    xbeeSerial.begin(9600);

    // Setup digital pins
    pinMode(JOYSTICK_1_BUTTON_PIN, INPUT);
    pinMode(JOYSTICK_2_BUTTON_PIN, INPUT);
    pinMode(NEUTRAL_BUTTON_PIN, INPUT);

    // Serial.println("Setup Complete");

}

void loop()
{
    // delay(3000);
    if(dataRequested())
    {
        // Determine state remote controller 
        // Determine mode
        if (digitalRead(NEUTRAL_BUTTON_PIN))
        {
            // Change mode to netural
            mode = 3;
        }
        else if (digitalRead(JOYSTICK_1_BUTTON_PIN))
        {
            // Change mode to autonomous
            mode = 1;

        }
        else if (digitalRead(JOYSTICK_2_BUTTON_PIN))
        {
            // Change mode to teleoperation
            mode = 2;
        }

        // Determine servo position
        int16_t servoAnalogValue = analogRead(JOYSTICK_2_Y_PIN); // Should have range of 0 to 1023
        if (servoAnalogValue > 800)
        {
            servoIncrement = 80; // degrees
        }
        else if (servoAnalogValue < 200)
        {
            servoIncrement = -80; // degrees
        }

        // Determine velocity values
        xVelocity = map(analogRead(JOYSTICK_1_X_PIN), 0, 1023, -100, 100);
        yVelocity = map(analogRead(JOYSTICK_1_Y_PIN), 0, 1023, -100, 100); 

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
        Serial.println();
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
    Serial.println(high << 8 | low); // How high and low will be recombined on Mega
    xbeeSerial.write(high);
    xbeeSerial.write(low);
}