#include <Servo.h>

Servo servo;

// Define pin connections
const int trigPin = 13;  // Trigger pin for ultrasonic sensor
const int echoPin = A0;  // Echo pin for ultrasonic sensor (Analog pin used as digital)

const int servoPin = 11; // Servo pin to control sensor direction

// Motor driver pins for L298N
const int enAPin = 6;    // PWM control for left motor speed
const int in1Pin = 7;    // Left motor direction pin 1
const int in2Pin = 5;    // Left motor direction pin 2
const int in3Pin = 4;    // Right motor direction pin 1
const int in4Pin = 2;    // Right motor direction pin 2
const int enBPin = 3;    // PWM control for right motor speed

enum Motor { LEFT, RIGHT };

void go(Motor m, int speed) {
    digitalWrite(m == LEFT ? in1Pin : in3Pin, speed > 0 ? HIGH : LOW);
    digitalWrite(m == LEFT ? in2Pin : in4Pin, speed <= 0 ? HIGH : LOW);
    analogWrite(m == LEFT ? enAPin : enBPin, abs(speed));
    Serial.print(m == LEFT ? "Left" : "Right");
    Serial.print(" motor speed set to ");
    Serial.println(speed);
}

void testMotors() {
    Serial.println("Testing Motors:");
    go(LEFT, 255);
    delay(200);
    go(LEFT, -255);
    delay(200);
    go(LEFT, 0);
    go(RIGHT, 255);
    delay(200);
    go(RIGHT, -255);
    delay(200);
    go(RIGHT, 0);
}

unsigned int readDistance() {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long period = pulseIn(echoPin, HIGH);
    return period * 0.343 / 2;
}

#define NUM_ANGLES 7
int sensorAngles[NUM_ANGLES] = {60, 70, 80, 90, 100, 110, 120};
unsigned int distances[NUM_ANGLES];

void readNextDistance() {
    static int angleIndex = 0;
    static bool increasing = true;
    distances[angleIndex] = readDistance();
    servo.write(sensorAngles[angleIndex]);
    Serial.print("Servo at angle ");
    Serial.print(sensorAngles[angleIndex]);
    Serial.print(": ");
    Serial.print(distances[angleIndex]);
    Serial.println(" mm");
    if (increasing) {
        angleIndex++;
        if (angleIndex == NUM_ANGLES - 1) {
            increasing = false;
        }
    } else {
        angleIndex--;
        if (angleIndex == 0) {
            increasing = true;
        }
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);  // Set the analog pin as a digital input
    pinMode(enAPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);
    pinMode(enBPin, OUTPUT);

    servo.attach(servoPin);
    servo.write(90);  // Center the servo
    go(LEFT, 0);      // Ensure motors are off
    go(RIGHT, 0);     // Ensure motors are off
    testMotors();     // Perform a quick motor test

    for (int i = 0; i < NUM_ANGLES; i++) {
        readNextDistance();
        delay(200);
    }
}

void loop() {
    readNextDistance();
    bool tooClose = false;
    for (int i = 0; i < NUM_ANGLES; i++) {
        if (distances[i] < 300) { // Check if any objects are closer than 300 mm
            tooClose = true;
            Serial.print("Obstacle too close at angle ");
            Serial.println(sensorAngles[i]);
            break;
        }
    }

    if (tooClose) {
        go(LEFT, -180); // Move left motor in reverse
        go(RIGHT, 80);  // Move right motor slowly forward to turn
    } else {
        go(LEFT, 255);  // Move forward
        go(RIGHT, 255); // Move forward
    }

    delay(50); // Small delay to manage control frequency
}
