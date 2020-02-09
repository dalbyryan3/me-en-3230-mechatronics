// ME EN 3220   Mechanical-Basho Arduino Uno Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb

// Libraries to be used
#include <SoftwareSerial.h>

// Pin defintions
#define XBEE_SERIAL_RX_PIN 2
#define XBEE_SERIAL_TX_PIN 3

#define JOYSTICK_1_X_PIN A0
#define JOYSTICK_1_Y_PIN A1
#define JOYSTICK_2_Y_PIN A2


SoftwareSerial unoSerial(XBEE_SERIAL_RX_PIN,XBEE_SERIAL_TX_PIN); // RX TX

void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module
    unoSerial.begin(9600);

}

void loop()
{
    
}