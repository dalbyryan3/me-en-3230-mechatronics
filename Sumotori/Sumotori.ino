// ME EN 3220   Mechanical-Basho Arduino Mega 2560 Code     Ryan Dalby, Colby Smith, Aaron Fowlks Landon O'Camb

// Libraries to be used
#include <DualVNH5019MotorShieldMod3.h>
#include <PWMServo.h>
#include <QTRSensors.h>
#include <Encoder.h>
#include <math.h>

// Pin defintions
// Motor Pins
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

// Hall effect sensor pins
static const uint8_t HALL_EFFECT_SENSOR_PIN = A2;

// Encoder pins
static const uint8_t MOTOR_2_ENCODER_PIN_1 = 18;
static const uint8_t MOTOR_2_ENCODER_PIN_2 = 19;

static const uint8_t MOTOR_3_ENCODER_PIN_1 = 20;
static const uint8_t MOTOR_3_ENCODER_PIN_2 = 21;

// Global objects
DualVNH5019MotorShieldMod3 md(MOTOR_3_DIRECTION_A_PIN, MOTOR_3_DIRECTION_B_PIN, MOTOR_3_ENABLE_PIN, MOTOR_3_CURRENT_SENSE_PIN, MOTOR_3_SPEED_PIN, MOTOR_4_DIRECTION_A_PIN, MOTOR_4_DIRECTION_B_PIN, MOTOR_4_ENABLE_PIN, MOTOR_4_CURRENT_SENSE_PIN, MOTOR_4_SPEED_PIN);
PWMServo servo1;
PWMServo servo2;
QTRSensors reflectanceSensor;
Encoder rightEnc(MOTOR_2_ENCODER_PIN_1, MOTOR_2_ENCODER_PIN_2); // Corresponds to motor 2 (right motors)
Encoder leftEnc(MOTOR_3_ENCODER_PIN_1, MOTOR_3_ENCODER_PIN_2);  // Corresponds to motor 3 (left motors)

// Global state set to inital values
int16_t recievedData[4];
int16_t mode = 3; // 3 is neutral mode

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
unsigned long lastServoMoveTime = millis(); // milliseconds
unsigned long servoWaitTime = 50;           // milliseconds

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

// Hall effect sensor state variables
static const int16_t HALL_EFFECT_SENSOR_LOW_THRESHOLD = 430;  // On a 0 to 1023 scale which maps to 0 to 5 V sensor output
static const int16_t HALL_EFFECT_SENSOR_HIGH_THRESHOLD = 878; // On a 0 to 1023 scale which maps to 0 to 5 V sensor output
bool isIntroductoryMode = false;

// Motor encoder state variables
static const double MOTOR_GEAR_RATIO = 131; // 131:1
static const int16_t ENCODER_COUNTS_PER_REV = 64;
static const double WHEEL_RADIUS = 3.5; // cm
int16_t rightEncoderCount = 0;
int16_t leftEncoderCount = 0;
unsigned long lastRightEncoderReadTime = millis();
unsigned long lastLeftEncoderReadTime = millis();
double prevRightMotorAngularPosition = 0;
double prevLeftMotorAngularPosition = 0;
double rightMotorAngularPosition = 0;
double leftMotorAngularPosition = 0;
double rightMotorAngularVelocity = 0;
double leftMotorAngularVelocity = 0;
double leftMotorPosition = 0;
double rightMotorPosition = 0;

// PM 9 demo variables
bool isStraightDemo = false;
double straightLineGain = 5.0;
double radiusGain = 3.5;
static const int16_t FINAL_STRAIGHT_LINE_POSITION = 16.0; // cm
static const int16_t TURNING_RADIUS = 20.0;               // cm
static const int16_t WHEEL_BASE = 20.75;                  // cm

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
    for (int i = 0; i < REFLECTANCE_SENSOR_COUNT; i++) // Add to bias to guarentee the "zero" value stays the zero value even with noise
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
            if (recievedData[0] != mode)
            {
                mode = recievedData[0];
                isIntroductoryMode = false; // If we updated the mode want to change out of introductory mode
            }

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

    // Determine mode and modify global sumotori state
    // Check if in introductory mode (For now this is PM8 Mode which in the end would include parts of the PM9 demo mode, the hall effect triggering mechanism already implementedin PM8, and the PM7 line and wall tracking)
    if (isIntroductoryMode)
    {
        // PM 8 mode
        yVelocity = 25;

        // Introductory procedure
        // Determine if over line, if in range of wall
        // if not over line and wall is in range: track wall // step 1
        // else if over line then follow line // step 2
        // else we are at final part of intro ceremony so turn slightly right, straighten out, and turn to face center(using wheel odometrey to control), once this is done set isIntroductoryMode to false which will return to netural mode // step 3
    }
    // PM 9 demo mode
    else if (mode == 1)
    {
        // NOTE: encoders and servos should not be used concurrently since when the servo and encoder library function calls are used at the same time there becomes timing issues for determing last encoder read time
        int32_t rightEncReading = rightEnc.read() * -1; // multiply by negative 1 to make positive values mean forward
        if (rightEncReading != rightEncoderCount)       // Only calculate next velocity value when the value has changed to prevent erroneous 0 velocity values since we can possibly loop back before the next value is ready
        {
            rightEncoderCount = rightEncReading / MOTOR_GEAR_RATIO;
            rightMotorAngularPosition = rightEncoderCount * (2 * M_PI) / ENCODER_COUNTS_PER_REV;
            rightMotorAngularVelocity = 1000.0 * ((rightMotorAngularPosition - prevRightMotorAngularPosition) / ((double)millis() - (double)lastRightEncoderReadTime));
            prevRightMotorAngularPosition = rightMotorAngularPosition;
            // lastRightEncoderReadTime = millis();
            // // Serial.print(rightEncoderCount);
            // // Serial.print("\t");
            // // Serial.print(rightMotorAngularPosition);
            // // Serial.print("\t");
            Serial.print(rightMotorAngularPosition * WHEEL_RADIUS); // distance traveled in centimeters
            rightMotorPosition = rightMotorAngularPosition * WHEEL_RADIUS;
            // // Serial.print("\t");
            // // Serial.print(rightMotorAngularVelocity);
            // // Serial.print("\t");
            // // Serial.println(rightMotorAngularVelocity * WHEEL_RADIUS);
        }

        int32_t leftEncReading = leftEnc.read();
        if (leftEncReading != leftEncoderCount) // Only calculate next velocity value when the value has changed to prevent erroneous 0 velocity values since we can possibly loop back before the next value is ready
        {
            leftEncoderCount = leftEncReading / MOTOR_GEAR_RATIO;
            leftMotorAngularPosition = leftEncoderCount * (2 * M_PI) / ENCODER_COUNTS_PER_REV;
            leftMotorAngularVelocity = 1000.0 * ((leftMotorAngularPosition - prevLeftMotorAngularPosition) / ((double)millis() - (double)lastLeftEncoderReadTime));
            prevLeftMotorAngularPosition = leftMotorAngularPosition;
            lastLeftEncoderReadTime = millis();
            // Serial.print(leftEncoderCount);
            // Serial.print("\t");
            // Serial.print(leftMotorAngularPosition);
            Serial.print("\t");
            Serial.println(leftMotorAngularPosition * WHEEL_RADIUS); // distance traveled in centimeters
            leftMotorPosition = leftMotorAngularPosition * WHEEL_RADIUS;
            // Serial.print("\t");
            // Serial.print(leftMotorAngularVelocity);
            // Serial.print("\t");
            // Serial.println(leftMotorAngularVelocity * WHEEL_RADIUS);
        }

        if (isStraightDemo) // If we are in straight line driving mode
        {
            // Control Law
            double error = (rightMotorPosition - leftMotorPosition);
            xVelocity = straightLineGain * error * -1;
            yVelocity = 20;
            if (leftMotorPosition > FINAL_STRAIGHT_LINE_POSITION || rightMotorPosition > FINAL_STRAIGHT_LINE_POSITION)
            {
                xVelocity = 0;
                yVelocity = 0;
                mode = 3; // Send out of this mode into neutral mode
            }
        }
        else // Then in circle driving mode
        {
            double error = (leftMotorAngularPosition / rightMotorPosition) - (WHEEL_BASE + WHEEL_RADIUS)/WHEEL_RADIUS;
            // Serial.println(error);
            xVelocity = radiusGain * error;
            yVelocity = 40;
            
        }
    }
    // Remote control mode
    else if (mode == 2)
    {
        servoIncrement = recievedData[1];
        xVelocity = recievedData[2];
        yVelocity = recievedData[3];
    }
    // Neutral mode- Will do nothing other than wait for Hall Effect sensor to be triggered to begin introductory mode
    else if (mode == 3)
    {
        xVelocity = 0;
        yVelocity = 0;
        servoIncrement = 0;
        uint16_t hallEffectReading = analogRead(HALL_EFFECT_SENSOR_PIN);
        // Serial.println(hallEffectReading);
        if (hallEffectReading <= HALL_EFFECT_SENSOR_LOW_THRESHOLD || hallEffectReading >= HALL_EFFECT_SENSOR_HIGH_THRESHOLD)
        {
            isIntroductoryMode = true;
            // Serial.println(" TRIGGERED ");
        }
    }

    // For now this mode is PM7 line tracking mode (Will be part of final introductory mode)
    else if (mode == 5)
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
            float normalizedDistanceFromCenter = distanceFromCenter / 3.5;                                  // Normalize distance from center to -1 to 1 since can be -3.5 to 3.5
            int16_t newXVelocity = normalizedDistanceFromCenter * REFLECTANCE_SENSOR_SPEED_MULTIPLIER * -1; // Will scale and flip our normalizedDistanceFromCenter
            // Serial.println(newXVelocity);
            xVelocity = newXVelocity;
            if (abs(distanceFromCenter) > .25 && abs(distanceFromCenter) < 1.0)
            {
                yVelocity = yVelocity * (3.0 / 4.0);
            }
            else if (abs(distanceFromCenter) > 1.5 && abs(distanceFromCenter) < 2.0)
            {
                yVelocity = yVelocity / 2.0;
            }
            else if (abs(distanceFromCenter) > 2.0 && abs(distanceFromCenter) < 3.0)
            {
                yVelocity = yVelocity / 4.0;
            }
            else if (abs(distanceFromCenter) > 3.0)
            {
                yVelocity = 0.0;
            }
        }
    }

    // For now this mode is PM7 wall tracking mode (Will be part of final introductory mode)
    else if (mode == 6)
    {
        yVelocity = 25;
        float distance = rangefinderSensorPowerFunction(map(analogRead(RANGEFINDER_SENSOR_SIDE_RIGHT_PIN), 0, 1024, 0, 5000) / 1000.0);
        // Serial.println(distance);
        float normalizedDistanceFromNominal = (distance - RANGEFINDER_NOMINAL_WALL_DISTANCE) / 5.0;
        int16_t newXVelocity = normalizedDistanceFromNominal * RANGEFINDER_NOMINAL_WALL_MULTIPLIER * -1;
        // Serial.println(newXVelocity);
        // Serial.println();
        xVelocity = newXVelocity;
        if (xVelocity > 15)
        {
            xVelocity = 15;
        }
        else if (xVelocity < -15)
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
// Will only write to servos if servoIncrement is not 0 (This prevents interference with encoders)
// Will limit the written servo commands to possible values
void writeServoIncrementToServos()
{
    if (servoIncrement == 0) // Only write to servos if servo position has changed
    {
        return;
    }
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
    return RANGEFINDER_SENSOR_A_CONSTANT * pow(voltage, RANGEFINDER_SENSOR_B_CONSTANT);
}
