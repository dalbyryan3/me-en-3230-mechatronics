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
static const uint8_t REFLECTANCE_SENSOR_PINS[REFLECTANCE_SENSOR_COUNT] = {28, 29, 30, 31, 32, 33, 34, 35};

// Rangefinder sensor pins
static const uint8_t RANGEFINDER_SENSOR_SIDE_RIGHT_PIN = A5;


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
uint16_t reflectanceSensorBias[REFLECTANCE_SENSOR_COUNT] = {1730, 1790, 1450, 1550, 1600, 1500, 1400, 1580}; // For inlab red carpet
// static const uint16_t reflectanceSensorBias[REFLECTANCE_SENSOR_COUNT] = {500, 500, 500, 500, 500, 500, 500, 500};
// static const uint16_t reflectanceSensorBias[REFLECTANCE_SENSOR_COUNT] = {1900, 1900,1900,1900,1900,1900,1900,1900};
static const float REFLECTANCE_SENSOR_DISTANCE_BETWEEN_SENSORS = 1.0; // cm
static const float REFLECTANCE_SENSOR_CENTER = (REFLECTANCE_SENSOR_COUNT + 1.0) / 2.0;
static const float REFLECTANCE_SENSOR_SPEED_MULTIPLIER = 25.0;

// Rangefinder sensor state variables
static const float RANGEFINDER_SENSOR_A_CONSTANT = 12.5;
static const float RANGEFINDER_SENSOR_B_CONSTANT = -1.0;
static const float RANGEFINDER_NOMINAL_WALL_DISTANCE = 20.0; // cm
static const float RANGEFINDER_NOMINAL_WALL_MULTIPLIER = 25.0;



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
    writeServoIncrementToServos(); // Writes intital servo positions

    delay(1000);

    // Initialize QTR-8RC reflectance sensor
    reflectanceSensor.setTypeRC();
    reflectanceSensor.setSensorPins(REFLECTANCE_SENSOR_PINS, REFLECTANCE_SENSOR_COUNT);
    reflectanceSensor.setEmitterPin(REFLECTANCE_SENSOR_EMITTER_PIN);
    // IMPLEMENT READ INTITAL VALUES AND MAKE IT BIAS
    for(int i = 0; i < REFLECTANCE_SENSOR_COUNT; i++) // Add to bias to guarentee the "zero" value stays the zero value even with noise
    {
        reflectanceSensorBias[i] = reflectanceSensorBias[i] + 200;
    }

    // Setup is complete
    // Serial.println("Setup Complete");

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
            while (integersRecieved < 4)
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
    // For now this mode is PM7 line tracking mode
    if (mode == 2)
    {
        // Read raw sensor values
        reflectanceSensor.read(reflectanceSensorValues);
        // Determine how far sumotori is from center
        float sum = 0;
        float weightedSum = 0;
        for (uint8_t i = 0; i < REFLECTANCE_SENSOR_COUNT; i++)
        {
            // Serial.print(reflectanceSensorValues[i]); 
            // Serial.print("    "); 
            uint16_t biasedValue = 0;
            if (reflectanceSensorValues[i] > reflectanceSensorBias[i])
            {
                biasedValue = reflectanceSensorValues[i] - reflectanceSensorBias[i];
            }
            sum = sum + biasedValue;
            weightedSum = weightedSum + (biasedValue * (i + 1));
        }

        // Serial.println();
        yVelocity = 50;
        if (sum == 0) // This means we are not over a line
        {
            xVelocity = 0;
        }
        else
        {
            float lineLocation = weightedSum / sum;
            float distanceFromCenter = (lineLocation - REFLECTANCE_SENSOR_CENTER) * REFLECTANCE_SENSOR_DISTANCE_BETWEEN_SENSORS;
            float normalizedDistanceFromCenter = distanceFromCenter / 3.5; // Normalize distance from center to -1 to 1 since can be -3.5 to 3.5
            int16_t newXVelocity = normalizedDistanceFromCenter * REFLECTANCE_SENSOR_SPEED_MULTIPLIER * -1; // Will scale and flip our normalizedDistanceFromCenter
            // Serial.println(newXVelocity);
            xVelocity = newXVelocity;
            if(abs(distanceFromCenter) > .25 && abs(distanceFromCenter) < 1.0)
            {
                yVelocity = yVelocity *  (3.0 / 4.0);
            }
            else if(abs(distanceFromCenter) > 1.5 && abs(distanceFromCenter) < 2.0)
            {
                yVelocity = yVelocity / 2.0;
            }
            else if(abs(distanceFromCenter) > 2.0 && abs(distanceFromCenter) < 3.0)
            {
                yVelocity = yVelocity / 4.0;
            }
            else if(abs(distanceFromCenter) > 3.0)
            {
                yVelocity = 0.0;
            }
        }

    }

    // For now this mode is PM7 wall tracking mode
    if (mode == 3)
    {
        yVelocity = 25;
        float distance = rangefinderSensorPowerFunction(map(analogRead(RANGEFINDER_SENSOR_SIDE_RIGHT_PIN), 0, 1024, 0, 5000)/1000.0);
        // Serial.println(distance);
        float normalizedDistanceFromNominal = (distance - RANGEFINDER_NOMINAL_WALL_DISTANCE) / 5.0;
        int16_t newXVelocity = normalizedDistanceFromNominal * RANGEFINDER_NOMINAL_WALL_MULTIPLIER * -1;
        // Serial.println(newXVelocity);
        // Serial.println();
        xVelocity = newXVelocity;
        if(xVelocity > 15)
        {
            xVelocity = 15;
        }
        else if(xVelocity < -15)
        {
            xVelocity = -15;
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
        // Serial.println(servo1.read());
        // Serial.println(servo2.read());
        // Serial.println();
}

// Methods which help abstract code above

// Used translate and write the velocity global state into motor commands
// Will limit the written motor commands to possible values
void writeVelocityToMotors()
{
    // Will use differential steering between right and left motors to turn sumotori

    int16_t motorSpeedVert = map(yVelocity, -100, 100, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    int16_t motorSpeedHorz = map(xVelocity, -100, 100, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
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
    // Serial.println(rightMotorSpeed);
    md.setM2Speed(rightMotorSpeed);
    // Serial.print("Setting motor 3 speed to ");
    // Serial.println(rightMotorSpeed);
    md.setM3Speed(leftMotorSpeed);
    // Serial.print("Setting motor 4 speed to ");
    // Serial.println(leftMotorSpeed);
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
    servo1.write(servoPosition);
    servo2.write(map(servoPosition, 100, 180, 180, 100));
    delay(100);
}

// Translates rangefinder voltage in volts to distance in centimeters using the rangefinder power function derived from experimental data
float rangefinderSensorPowerFunction(float voltage)
{
    return RANGEFINDER_SENSOR_A_CONSTANT * pow(voltage,RANGEFINDER_SENSOR_B_CONSTANT);
}
