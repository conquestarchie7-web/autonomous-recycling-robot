/*
 * Autonomous Recycling Robot - Main Controller
 *
 * Arduino sketch for the primary robot board.
 * Responsibilities:
 *   - Line following using three IR sensors
 *   - Front and side obstacle/object detection using ultrasonic sensors
 *   - Autonomous object approach and collection
 *   - Servo-actuated gripper control
 *   - Ground colour detection
 *   - Communication with a secondary colour-sensor/display board
 */

#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SoftwareSerial.h>

// -------------------- Pin definitions --------------------

// Motors
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 3
#define MOT_B2_PIN 11

// Ultrasonic distance sensors
#define TRIGGER_PIN_FRONT 7
#define ECHO_PIN_FRONT 8
#define TRIGGER_PIN_RIGHT_SIDE 12
#define ECHO_PIN_RIGHT_SIDE 13

// RGB sensor (I2C on Arduino Uno: SDA=A4, SCL=A5)
#define RGB_SENSOR_SDA_PIN A4
#define RGB_SENSOR_SCL_PIN A5

// Gripper servos
#define SERVO_HINGE_PIN 9
#define SERVO_ARM 10

// Inter-board serial communication
SoftwareSerial mySerial(2, 4);  // RX=2, TX=4

// Three IR line sensors
const int irPins[3] = {A0, A1, A2};

// -------------------- Hardware objects --------------------

#define MAX_DISTANCE 200

NewPing sonar(
    TRIGGER_PIN_FRONT,
    ECHO_PIN_FRONT,
    MAX_DISTANCE
);

NewPing sonar_side(
    TRIGGER_PIN_RIGHT_SIDE,
    ECHO_PIN_RIGHT_SIDE,
    MAX_DISTANCE
);

Adafruit_TCS34725 tcs =
    Adafruit_TCS34725(
        TCS34725_INTEGRATIONTIME_50MS,
        TCS34725_GAIN_4X
    );

Servo servoHinge;
Servo servoArm;

// -------------------- Robot state --------------------

int distance = 0;
int side_distance = 0;

int counter = 0;

int irSensors = B000;
int irSensorDigital[3] = {0, 0, 0};
int threshold = 500;

const int defaultSpeed = 150;
int error = 0;
int errorLast = 0;
int leftServoSpeed = 0;
int rightServoSpeed = 0;

const int HINGE_UP = 0;
const int HINGE_DOWN = 65;
const int ARM_OPEN = 5;
const int ARM_CLOSED = 48;

bool objGrabbed = false;

String groundColour;
String incoming;
String serialBuffer = "";
String lastSentMessage = "";

// -------------------- Communication --------------------

/*
 * Send a status message to the secondary board.
 * Avoids repeatedly transmitting the same state.
 */
void sendMessage(String msg) {
    if (msg != lastSentMessage) {
        mySerial.println(msg);
        lastSentMessage = msg;
    }
}

// -------------------- Gripper control --------------------

void GripperInit() {
    servoHinge.write(HINGE_UP);
    servoArm.write(ARM_OPEN);
    delay(400);
}

/*
 * Lower the gripper, close around the object, then lift it.
 */
void GrabCanSequence() {
    sendMessage("GRABBING");

    servoArm.write(ARM_OPEN);
    delay(600);

    servoHinge.write(HINGE_DOWN);
    delay(600);

    servoArm.write(ARM_CLOSED);
    delay(600);

    servoHinge.write(HINGE_UP);

    objGrabbed = true;
}

/*
 * Lower the gripper, release the object, then raise it again.
 */
void DropCanSequence() {
    sendMessage("DEPOSITING");

    servoHinge.write(HINGE_DOWN);
    delay(600);

    servoArm.write(ARM_OPEN);
    delay(600);

    servoHinge.write(HINGE_UP);
    delay(600);

    objGrabbed = false;
    counter++;

    // Stop after three successful deposits.
    if (counter >= 3) {
        sendMessage("FINISHED");
        set_motor_pwm(0, 0, 0);

        while (true) {
        }
    }
}

// -------------------- Motor control --------------------

/*
 * Drive one motor using a signed PWM value (-255 to +255).
 */
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN) {
    if (pwm < 0) {
        analogWrite(IN1_PIN, -pwm);
        digitalWrite(IN2_PIN, LOW);
    } else {
        digitalWrite(IN1_PIN, LOW);
        analogWrite(IN2_PIN, pwm);
    }
}

/*
 * Drive both motors using signed PWM values.
 */
void set_motor_currents(int pwm_A, int pwm_B) {
    set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
    set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);
}

/*
 * Drive both motors for a fixed duration.
 */
void spin_and_wait(int pwm_A, int pwm_B, int duration) {
    set_motor_currents(pwm_A, pwm_B);
    delay(duration);
}

// -------------------- Distance sensing --------------------

bool isObjectInFront(int checkDist) {
    int d = sonar.ping_cm();
    return (d > 0 && d < checkDist);
}

/*
 * Handle obstacles detected directly ahead.
 * The robot pauses, waits for the obstacle to clear, then performs
 * a manoeuvre and returns to the line.
 */
void SenseDistance() {
    distance = sonar.ping_cm();

    if (distance > 0 && distance < 5) {
        set_motor_currents(0, 0);
        sendMessage("OBJECT IN FRONT");

        delay(4000);

        if (!isObjectInFront(7)) {
            return;
        }

        sendMessage("MOVING AROUND");

        // Reverse and turn away from the obstacle.
        spin_and_wait(-150, 0, 1500);
        spin_and_wait(150, 135, 250);

        // Follow an arc until the line is reacquired.
        while (true) {
            set_motor_currents(160, 100);
            delay(20);

            Scan();

            if (irSensors == B010 || irSensors == B111) {
                break;
            }
        }

        set_motor_currents(0, 0);
        delay(300);

        error = 0;
        errorLast = 0;
    }
}

/*
 * Search for an object beside the robot.
 * The robot pivots toward a detected object, approaches it,
 * grabs it, and then returns to the line.
 */
void SenseSide() {
    // Do not search while carrying an object.
    if (objGrabbed) {
        return;
    }

    sendMessage("SEARCHING");

    side_distance =
        sonar_side.convert_cm(
            sonar_side.ping_median(3)
        );

    if (side_distance > 0 && side_distance < 21) {
        set_motor_currents(0, 0);
        delay(500);

        // Pivot until the target is detected in front.
        while (!isObjectInFront(25)) {
            set_motor_currents(150, 0);
            delay(50);
        }

        spin_and_wait(120, 0, 300);

        // Approach the target slowly.
        while (!isObjectInFront(5)) {
            set_motor_currents(100, 80);
            delay(20);
        }

        set_motor_currents(0, 0);
        delay(500);

        GrabCanSequence();

        // Return to the line after collecting the object.
        ReturnToTrack(1);

        error = 0;
        errorLast = 0;
    }
}

// -------------------- Line following --------------------

/*
 * Read the three IR sensors and encode their states as a
 * three-bit value:
 *
 *   B100 -> line left
 *   B010 -> line centred
 *   B001 -> line right
 *
 * Sensor threshold is controlled by 'threshold'.
 */
void Scan() {
    irSensors = B000;

    for (int i = 0; i < 3; i++) {
        int sensorValue = analogRead(irPins[i]);

        irSensorDigital[i] =
            (sensorValue >= threshold) ? 1 : 0;

        int b = 2 - i;
        irSensors += (irSensorDigital[i] << b);
    }
}

/*
 * Convert the IR sensor pattern into steering error and
 * adjust motor speeds accordingly.
 */
void UpdateDirection() {
    errorLast = error;

    switch (irSensors) {
        case B000:
            if (errorLast < 0) {
                error = -130;
            } else if (errorLast > 0) {
                error = 130;
            }
            break;

        case B100:
            error = -80;
            break;

        case B110:
            error = -40;
            break;

        case B010:
            error = 0;
            break;

        case B011:
            error = 40;
            break;

        case B001:
            error = 80;
            break;

        case B111:
            error = 0;
            break;

        default:
            error = errorLast;
    }

    if (error >= 0) {
        leftServoSpeed = defaultSpeed;
        rightServoSpeed = defaultSpeed - error;
    } else {
        leftServoSpeed = defaultSpeed + error;
        rightServoSpeed = defaultSpeed;
    }
}

/*
 * Reacquire the line after collecting or depositing an object.
 *
 * pivotDir =  1  -> return after grabbing
 * pivotDir = -1  -> return after dropping
 */
void ReturnToTrack(int pivotDir) {
    int revA = (pivotDir == 1) ? -140 : -100;
    int revB = (pivotDir == 1) ? -100 : -140;

    // Reverse until the line is detected.
    while (true) {
        Scan();

        if (irSensors != B000) {
            break;
        }

        set_motor_currents(revA, revB);
        delay(20);
    }

    set_motor_currents(0, 0);
    delay(500);

    // Pivot until the centre of the line is detected.
    while (true) {
        Scan();

        if (irSensors == B010 || irSensors == B111) {
            break;
        }

        set_motor_currents(
            pivotDir == -1 ? 125 : 40,
            pivotDir == 1 ? 110 : 40
        );

        delay(10);
    }

    set_motor_currents(0, 0);
    delay(500);
}

// -------------------- Colour-based sorting --------------------

/*
 * Read the ground colour while carrying an object.
 * The ground colour determines which deposit zone the object
 * should be delivered to.
 */
void detectColour() {
    if (!objGrabbed) {
        return;
    }

    groundColour = "NONE";

    float red, green, blue;
    tcs.getRGB(&red, &green, &blue);

    if (red > (green + 30) &&
        red > (blue + 30) &&
        red > 120) {

        groundColour = "RED";

    } else if (green > red &&
               green > blue &&
               green > 90) {

        groundColour = "GREEN";

    } else if (blue > (red + 15) &&
               blue > (green + 15) &&
               blue > 90) {

        groundColour = "BLUE";
    }

    // Receive the object colour from the secondary board.
    if (mySerial.available()) {
        char c = mySerial.read();

        if (c == '\n') {
            incoming = serialBuffer;
            incoming.trim();
            serialBuffer = "";
        } else {
            serialBuffer += c;
        }
    }

    // Deposit when object colour matches the ground zone.
    if (incoming == groundColour) {
        set_motor_currents(0, 0);
        delay(500);

        while (!isObjectInFront(15)) {
            set_motor_currents(0, 180);
            delay(50);
        }

        while (!isObjectInFront(4)) {
            set_motor_currents(100, 80);
            delay(20);
        }

        set_motor_currents(0, 0);
        delay(500);

        DropCanSequence();

        ReturnToTrack(-1);

        error = 0;
        errorLast = 0;

        incoming = "";
    }
}

// -------------------- Arduino setup --------------------

void setup(void) {
    pinMode(MOT_A1_PIN, OUTPUT);
    pinMode(MOT_A2_PIN, OUTPUT);
    pinMode(MOT_B1_PIN, OUTPUT);
    pinMode(MOT_B2_PIN, OUTPUT);

    digitalWrite(MOT_A1_PIN, LOW);
    digitalWrite(MOT_A2_PIN, LOW);
    digitalWrite(MOT_B1_PIN, LOW);
    digitalWrite(MOT_B2_PIN, LOW);

    servoHinge.attach(SERVO_HINGE_PIN);
    servoArm.attach(SERVO_ARM);

    GripperInit();

    Serial.begin(9600);
    mySerial.begin(57600);

    if (!tcs.begin()) {
        Serial.println(
            "No TCS34725 found - check connections."
        );
    }

    delay(1000);
}

// -------------------- Main control loop --------------------

void loop() {
    SenseDistance();    // Safety / obstacle handling
    SenseSide();        // Object search and collection
    Scan();             // Read line sensors
    UpdateDirection();  // Update steering correction
    detectColour();     // Check for matching deposit zone

    // Apply motor speeds for a short control interval.
    spin_and_wait(
        leftServoSpeed * 1.15,
        rightServoSpeed,
        10
    );
}
