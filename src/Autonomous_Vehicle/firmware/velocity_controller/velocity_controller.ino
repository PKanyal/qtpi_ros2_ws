#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

// =====================================================
// 1. CYTRON MDD10A MOTOR DRIVER PINS (sign-magnitude: PWM + DIR)
//    (L298N version: firmware/velocity_controller_l298n)
// =====================================================

// Left motors: MDD10A channel 1 (M1A / M1B)
#define LEFT_PWM    5     // -> MDD10A PWM1
#define LEFT_DIR    12    // -> MDD10A DIR1

// Right motors: MDD10A channel 2 (M2A / M2B)
#define RIGHT_PWM   6     // -> MDD10A PWM2
#define RIGHT_DIR   7     // -> MDD10A DIR2

// DIR level that drives each side FORWARD. If a side spins backwards
// on the first test, flip its value here (or swap that side's two motor
// wires on M1A/M1B or M2A/M2B) - do NOT touch the encoder wiring.
#define LEFT_FORWARD_DIR    HIGH   // flipped 2026-11-19: left ran backwards
#define RIGHT_FORWARD_DIR   LOW

// =====================================================
// 2. ENCODER PINS - ORIGINAL WORKING WIRING
// =====================================================

#define LEFT_ENC_A    2
#define LEFT_ENC_B    4

#define RIGHT_ENC_A   3
#define RIGHT_ENC_B   9

// =====================================================
// 3. ROBOT PARAMETERS
// =====================================================

const float WHEEL_RADIUS = 0.04f;
// Effective skid-steer wheel base (spin test 2026-11-18; measured track is
// 0.25 m but the wheels scrub in turns). Keep equal to wheel_base in
// robot_bringup/launch/bringup.launch.py.
const float WHEEL_SEPARATION = 0.372f;
const float COUNTS_PER_REV = 4684.0f;

const float MAX_WHEEL_SPEED = 0.20f;

// =====================================================
// 4. MOTOR CONTROL PARAMETERS
// =====================================================

// MDD10A has almost no voltage drop (the L298N lost ~2 V), so the same PWM
// now gives the motors more voltage. Start lower and tune on the robot:
// MIN_PWM = lowest value at which a wheel reliably starts turning.
const int MAX_PWM = 150;
const int MIN_PWM = 70;    // L298N needed 100

const float KP = 250.0f;
const float KI = 80.0f;

const float INTEGRAL_LIMIT = 1.0f;

const unsigned long CONTROL_INTERVAL = 20;
const unsigned long REPORT_INTERVAL = 50;
const unsigned long COMMAND_TIMEOUT = 300;

// =====================================================
// 5. ENCODER VARIABLES
// =====================================================

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

long previousLeftCount = 0;
long previousRightCount = 0;

float leftMeasuredSpeed = 0.0f;
float rightMeasuredSpeed = 0.0f;

float leftIntegral = 0.0f;
float rightIntegral = 0.0f;

float leftTargetSpeed = 0.0f;
float rightTargetSpeed = 0.0f;

// =====================================================
// 6. TIMING VARIABLES
// =====================================================

unsigned long lastControlTime = 0;
unsigned long lastReportTime = 0;
unsigned long lastCommandTime = 0;

bool commandReceived = false;

// =====================================================
// 7. ENCODER INTERRUPTS
// =====================================================

// Left encoder: D2 = Channel A, D4 = Channel B

void leftEncoderISR() {

    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);

    if (a == b)
        leftEncoderCount++;
    else
        leftEncoderCount--;
}

// Right encoder: D3 = Channel A, D9 = Channel B

void rightEncoderISR() {

    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);

    if (a == b)
        rightEncoderCount++;
    else
        rightEncoderCount--;
}

// =====================================================
// 8. MOTOR OUTPUT FUNCTIONS
// =====================================================

// command: signed PWM (-MAX_PWM..MAX_PWM); sign = direction.
void driveMotor(int pwmPin, int dirPin, int forwardLevel, float command) {

    if (command == 0.0f) {
        analogWrite(pwmPin, 0);     // MDD10A: PWM 0 = motor off (brake)
        return;
    }

    int pwm = constrain((int)fabs(command), 0, MAX_PWM);

    if (pwm > 0 && pwm < MIN_PWM)
        pwm = MIN_PWM;

    int reverseLevel = (forwardLevel == HIGH) ? LOW : HIGH;
    digitalWrite(dirPin, command > 0.0f ? forwardLevel : reverseLevel);
    analogWrite(pwmPin, pwm);
}

void setLeftMotor(float command) {
    driveMotor(LEFT_PWM, LEFT_DIR, LEFT_FORWARD_DIR, command);
}

void setRightMotor(float command) {
    driveMotor(RIGHT_PWM, RIGHT_DIR, RIGHT_FORWARD_DIR, command);
}

// =====================================================
// 9. STOP MOTORS
// =====================================================

void stopMotors() {

    // MDD10A: PWM 0 stops the motor regardless of DIR
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);

    leftIntegral = 0.0f;
    rightIntegral = 0.0f;

    leftTargetSpeed = 0.0f;
    rightTargetSpeed = 0.0f;
}

// =====================================================
// 10. SERIAL COMMAND PROCESSING
//
// Format: V,linear_velocity,angular_velocity
// Example: V,0.10,0.0
// =====================================================

char serialBuffer[48];
byte serialIndex = 0;

void processCommand(char *command) {

    if (command[0] != 'V')
        return;

    char *token = strtok(command, ",");

    token = strtok(NULL, ",");

    if (token == NULL)
        return;

    float linear = atof(token);

    token = strtok(NULL, ",");

    if (token == NULL)
        return;

    float angular = atof(token);

    if (!isfinite(linear) || !isfinite(angular))
        return;

    // Differential drive inverse kinematics

    float left =
        linear - angular * WHEEL_SEPARATION / 2.0f;

    float right =
        linear + angular * WHEEL_SEPARATION / 2.0f;

    // Limit wheel speeds proportionally

    float maxSpeed = fmaxf(
        fabs(left),
        fabs(right)
    );

    if (maxSpeed > MAX_WHEEL_SPEED) {

        float scale = MAX_WHEEL_SPEED / maxSpeed;

        left *= scale;
        right *= scale;
    }

    leftTargetSpeed = left;
    rightTargetSpeed = right;

    lastCommandTime = millis();
    commandReceived = true;
}

// =====================================================
// 11. SERIAL INPUT
// =====================================================

void readSerialCommands() {

    while (Serial.available()) {

        char c = Serial.read();

        if (c == '\n' || c == '\r') {

            if (serialIndex > 0) {

                serialBuffer[serialIndex] = '\0';

                processCommand(serialBuffer);

                serialIndex = 0;
            }
        }
        else if (serialIndex < sizeof(serialBuffer) - 1) {

            serialBuffer[serialIndex++] = c;
        }
        else {

            serialIndex = 0;
        }
    }
}

// =====================================================
// 12. ENCODER SPEED MEASUREMENT
// =====================================================

void updateEncoderSpeeds(float dt) {

    long leftCount;
    long rightCount;

    noInterrupts();

    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;

    interrupts();

    long leftDelta =
        leftCount - previousLeftCount;

    long rightDelta =
        rightCount - previousRightCount;

    previousLeftCount = leftCount;
    previousRightCount = rightCount;

    float circumference =
        2.0f * PI * WHEEL_RADIUS;

    leftMeasuredSpeed =
        fabs((float)leftDelta) /
        COUNTS_PER_REV *
        circumference / dt;

    rightMeasuredSpeed =
        fabs((float)rightDelta) /
        COUNTS_PER_REV *
        circumference / dt;
}

// =====================================================
// 13. PI CONTROLLER
// =====================================================

float calculatePWM(
    float target,
    float measured,
    float &integral,
    float dt
) {

    if (fabs(target) < 0.001f) {

        integral = 0.0f;
        return 0.0f;
    }

    float error = fabs(target) - measured;

    integral += error * dt;

    integral = constrain(
        integral,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
    );

    float output =
        KP * error + KI * integral;

    output = constrain(
        output,
        0.0f,
        (float)MAX_PWM
    );

    if (output > 0.0f && output < MIN_PWM)
        output = MIN_PWM;

    return output;
}

// =====================================================
// 14. MOTOR CONTROL UPDATE
// =====================================================

void updateMotorControl(float dt) {

    if (!commandReceived ||
        millis() - lastCommandTime > COMMAND_TIMEOUT) {

        stopMotors();
        return;
    }

    float leftPWM = calculatePWM(
        leftTargetSpeed,
        leftMeasuredSpeed,
        leftIntegral,
        dt
    );

    float rightPWM = calculatePWM(
        rightTargetSpeed,
        rightMeasuredSpeed,
        rightIntegral,
        dt
    );

    static unsigned long lastDebug = 0;

    if (millis() - lastDebug >= 500) {
        lastDebug = millis();

        Serial.print("LT:");
        Serial.print(leftTargetSpeed, 3);

        Serial.print(" RT:");
        Serial.print(rightTargetSpeed, 3);

        Serial.print(" LPWM:");
        Serial.print(leftPWM);

        Serial.print(" RPWM:");
        Serial.println(rightPWM);
    }

    setLeftMotor(
        leftTargetSpeed > 0.0f ? leftPWM :
        (leftTargetSpeed < 0.0f ? -leftPWM : 0.0f)
    );

    setRightMotor(
        rightTargetSpeed > 0.0f ? rightPWM :
        (rightTargetSpeed < 0.0f ? -rightPWM : 0.0f)
    );
}

// =====================================================
// 15. ENCODER REPORTING
//
// Format: E,left_count,right_count
// =====================================================

void reportEncoders() {

    long leftCount;
    long rightCount;

    noInterrupts();

    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;

    interrupts();

    Serial.print("E,");
    Serial.print(leftCount);
    Serial.print(",");
    Serial.println(rightCount);
}

// =====================================================
// 16. SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    // Motor pins

    // Motors off before anything else, so the MDD10A never sees a
    // floating PWM pin while the Uno boots.
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);

    pinMode(LEFT_DIR, OUTPUT);
    pinMode(RIGHT_DIR, OUTPUT);
    digitalWrite(LEFT_DIR, LEFT_FORWARD_DIR);
    digitalWrite(RIGHT_DIR, RIGHT_FORWARD_DIR);

    // Encoder pins

    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);

    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // External interrupts:
    // Arduino Uno D2 = INT0
    // Arduino Uno D3 = INT1

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENC_A),
        leftEncoderISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENC_A),
        rightEncoderISR,
        CHANGE
    );

    stopMotors();

    lastControlTime = millis();
    lastReportTime = millis();
    lastCommandTime = millis();

    Serial.println("Velocity Controller Ready");
}

// =====================================================
// 17. MAIN LOOP
// =====================================================

void loop() {

    readSerialCommands();

    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL) {

        float dt =
            (now - lastControlTime) / 1000.0f;

        lastControlTime = now;

        updateEncoderSpeeds(dt);

        updateMotorControl(dt);
    }

    if (now - lastReportTime >= REPORT_INTERVAL) {

        lastReportTime = now;

        reportEncoders();
    }
}
