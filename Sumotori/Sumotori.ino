// ME EN 3220   Mechanical-Basho Arduino Mega 2560 Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb

// Libraries to be used
#include <DualVNH5019MotorShieldMod3.h>
#include <PWMServo.h>
#include <QTRSensors.h>

// Pin defintions
static const uint8_t MOTOR_1_DIRECTION_A_PIN = 2;
static const uint8_t MOTOR_1_DIRECTION_B_PIN = 4;
static const uint8_t MOTOR_1_SPEED_PIN = 9;
static const uint8_t MOTOR_1_ENABLE_PIN = 6;
static const uint8_t MOTOR_1_CURRENT_SENSE_PIN = A0;
static const uint8_t MOTOR_2_DIRECTION_A_PIN = 7;
static const uint8_t MOTOR_2_DIRECTION_B_PIN = 8;
static const uint8_t MOTOR_2_SPEED_PIN = 10;
static const uint8_t MOTOR_2_ENABLE_PIN = 12;
static const uint8_t MOTOR_2_CURRENT_SENSE_PIN = A1;
static const uint8_t MOTOR_3_DIRECTION_A_PIN = 24;
static const uint8_t MOTOR_3_DIRECTION_B_PIN = 26;
static const uint8_t MOTOR_3_SPEED_PIN = 45;
static const uint8_t MOTOR_3_ENABLE_PIN = 22;
static const uint8_t MOTOR_3_CURRENT_SENSE_PIN = A3;
static const uint8_t MOTOR_4_DIRECTION_A_PIN = 25;
static const uint8_t MOTOR_4_DIRECTION_B_PIN = 27;
static const uint8_t MOTOR_4_SPEED_PIN = 46;
static const uint8_t MOTOR_4_ENABLE_PIN = 23;
static const uint8_t MOTOR_4_CURRENT_SENSE_PIN = A4;

// Servo pins
static const uint8_t SERVO_1_PIN = 11;
static const uint8_t SERVO_2_PIN = 13;

// Reflectance sensor pins
static const uint8_t REFLECTANCE_SENSOR_COUNT = 8;
static const uint8_t REFLECTANCE_SENSOR_EMITTER_PIN = 36;
static const uint8_t REFLECTANCE_SENSOR_PINS[REFLECTANCE_SENSOR_COUNT] = {28,29,30,31,32,33,34,35};

// Global objects
DualVNH5019MotorShieldMod3 md(MOTOR_3_DIRECTION_A_PIN, MOTOR_3_DIRECTION_B_PIN, MOTOR_3_ENABLE_PIN, MOTOR_3_CURRENT_SENSE_PIN, MOTOR_3_SPEED_PIN, MOTOR_4_DIRECTION_A_PIN, MOTOR_4_DIRECTION_B_PIN, MOTOR_4_ENABLE_PIN, MOTOR_4_CURRENT_SENSE_PIN, MOTOR_4_SPEED_PIN);
PWMServo servo1;
PWMServo servo2;
QTRSensors reflectanceSensor;

// Global state set to inital values
int16_t recievedData[4];
int16_t mode = 1; // 1 is teleoperation mode

// Motor state variables
static const int16_t MOTOR_UPPER_LIMIT = 400;
static const int16_t MOTOR_LOWER_LIMIT = -400;
int16_t xVelocity = 0;
int16_t yVelocity = 0;

// Serial communication state variables
unsigned long lastDataRequestTime = millis(); // milliseconds
unsigned long packetWaitTime = 100;           // milliseconds

// Servo state variables
static const int16_t SERVO_UPPER_LIMIT = 180; // degrees
static const int16_t SERVO_LOWER_LIMIT = 100; // degrees
int16_t servoPosition = 100;
int16_t servoIncrement = 0;
unsigned long lastServoMoveTime = 0; // milliseconds
unsigned long servoWaitTime = 50;    // milliseconds

// Reflectance sensor state variables
uint16_t reflectanceSensorValues[REFLECTANCE_SENSOR_COUNT];
static const uint16_t reflectanceSensorBias[REFLECTANCE_SENSOR_COUNT] = {250,250,250,250,250,250};
static const float reflectanceSensorCenter = (REFLECTANCE_SENSOR_COUNT + 1.0) / 2.0;

void setup()
{
    // Begin serial communication over the USB bus
    Serial.begin(9600);

    // Begin serial communication over the Xbee module, the Xbee will use pins 16 and 17 which correspond to Serial2 on the Mega
    Serial2.begin(9600);

    // Initialize the two DualVNH5019 motor shields
    md.init();

    // Initialize servo
    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);
    servo1.write(servoPosition);
    servo2.write(servoPosition);
    delay(1000);
    servo1.detach();
    servo2.detach();

    // Initialize QTR-8RC reflectance sensor
    reflectanceSensor.setTypeRC();
    reflectanceSensor.setSensorPins(REFLECTANCE_SENSOR_PINS, REFLECTANCE_SENSOR_COUNT);
    reflectanceSensor.setEmitterPin(REFLECTANCE_SENSOR_EMITTER_PIN);

    // Setup is complete
    Serial.println("Setup Complete");

    // Ask for first state
    Serial2.write(255);
    lastDataRequestTime = millis();
    // Serial.println("ASKING FOR INITIAL DATA");
}

void loop()
{
    // Recieve data if available
    if (Serial2.available() > 0)
    {
        if (Serial2.read() == 255) // Extract starting byte
        {
            // Serial.println("DATA PACKET RECEIVED");
            int16_t integersRecieved = 0;
            while (integersRecieved < 6)
            {
                if (Serial2.available() > 1)
                {
                    byte high = Serial2.read();
                    byte low = Serial2.read();
                    int16_t reconstructedData = high << 8 | low;
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
        while (Serial2.available() > 0)
        {
            Serial2.read();
        }
    }

    // IMPLEMENT CHECK MODE
    // Determine mode and modify global sumotori state
    if (mode == 1)
    {
        servoIncrement = recievedData[1];
        xVelocity = recievedData[2];
        yVelocity = recievedData[3];
    }
    // For now this mode is PM7 mode
    if (mode == 2)
    {
        // Read raw sensor values
        reflectanceSensor.read(reflectanceSensorValues);

        // Determine how far sumotori is from center
        float sum = 0;
        float weightedSum = 0;
        for (uint8_t i = 0; i < REFLECTANCE_SENSOR_COUNT; i++)
        {
            uint16_t biasedValue = 0;
            if(reflectanceSensorValues[i] > reflectanceSensorBias[i])
            {
                biasedValue = reflectanceSensorValues[i] - reflectanceSensorBias[i];
            }
            sum = sum + biasedValue;
            weightedSum = weightedSum + (biasedValue*(i+1));
        }

        float lineLocation = weightedSum / sum;
        float distanceFromCenter = lineLocation - reflectanceSensorCenter;

        // Adjust velocity according to how far sumotori is from center
        // Can change below to drive proportionally to how for from center the robot is
        if(distanceFromCenter > 1)
        {
            // drive to the left
        }
        else if(distanceFromCenter < -1)
        {
            // drive to the right
        }

    }

    // Write current motor state
    writeVelocityToMotors();
    // Write current servo state
    if ((millis() - lastServoMoveTime) > servoWaitTime)
    {
        writeServoIncrementToServos();
        // Serial.println(servo1.read());
        // Serial.println(servo2.read());
        // Serial.println();
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

// Methods which help abstract code above

// Used translate and write the velocity global state into motor commands
// Will limit the written motor commands to possible values
void writeVelocityToMotors()
{
    // Will use differential steering between right and left motors to turn sumotori

    int16_t motorSpeedVert = map(yVelocity, -10, 10, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    int16_t motorSpeedHorz = map(xVelocity, -10, 10, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    int16_t rightMotorSpeed = (motorSpeedVert + motorSpeedHorz); // Motors 1 and 2
    int16_t leftMotorSpeed = (motorSpeedVert - motorSpeedHorz);  // Motors 3 and 4

    // Apply upper and lower limits to motor values
    if (rightMotorSpeed > MOTOR_UPPER_LIMIT)
    {
        rightMotorSpeed = MOTOR_UPPER_LIMIT;
    }
    if (rightMotorSpeed < MOTOR_LOWER_LIMIT)
    {
        rightMotorSpeed = MOTOR_LOWER_LIMIT;
    }
    if (leftMotorSpeed > MOTOR_UPPER_LIMIT)
    {
        leftMotorSpeed = MOTOR_UPPER_LIMIT;
    }
    if (leftMotorSpeed < MOTOR_LOWER_LIMIT)
    {
        leftMotorSpeed = MOTOR_LOWER_LIMIT;
    }

    // Write motor speeds
    md.setM1Speed(rightMotorSpeed);
    // Serial.print("Setting motor 2 speed to ");
    // Serial.println(motor2Speed);
    md.setM2Speed(rightMotorSpeed);
    // Serial.print("Setting motor 3 speed to ");
    // Serial.println(motor3Speed);
    md.setM3Speed(leftMotorSpeed);
    // Serial.print("Setting motor 4 speed to ");
    // Serial.println(motor4Speed);
    md.setM4Speed(leftMotorSpeed);
}

// Used to translate and write the servo global state into servo write commands
// Will limit the written servo commands to possible values
void writeServoIncrementToServos()
{
    servoPosition += servoIncrement;

    // Apply upper and lower limits to servo position
    if (servoPosition > SERVO_UPPER_LIMIT)
    {
        servoPosition = 180;
    }
    if (servoPosition < SERVO_LOWER_LIMIT)
    {
        servoPosition = 100;
    }

    // Write servo values
    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);
    servo1.write(servoPosition);
    servo2.write(map(servoPosition, 100, 180, 180, 100));
    delay(100);
    servo1.detach();
    servo2.detach();
}
