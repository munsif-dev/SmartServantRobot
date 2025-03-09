/*
 * Firmware for the "2WD Ultrasonic Motor Robot Car Kit"
 *
 * Stephen A. Edwards
 *
 * Hardware configuration:
 * - A pair of DC motors driven by an L298N H-bridge motor driver
 * - An HC-SR04 ultrasonic range sensor mounted atop a small hobby servo
 */

#include <Servo.h>

Servo servo;

// Ultrasonic Module pins
const int trigPin = 13; // 10-microsecond high pulse causes chirp, wait 50us
const int echoPin = 12; // Width of high pulse indicates distance

// Servo motor that aims ultrasonic sensor
const int servoPin = 11; // PWM output for hobby servo

// Motor control pins: L298N H-bridge
const int enAPin = 6;  // Left motor PWM speed control
const int in1Pin = 7;  // Left motor Direction 1
const int in2Pin = 5;  // Left motor Direction 2
const int in3Pin = 4;  // Right motor Direction 1
const int in4Pin = 2;  // Right motor Direction 2
const int enBPin = 3;  // Right motor PWM speed control

enum Motor { LEFT, RIGHT };

// Set motor speed: 255 full ahead, -255 full reverse, 0 stop
void go(Motor m, int speed) {
    digitalWrite(m == LEFT ? in1Pin : in3Pin, speed > 0 ? HIGH : LOW);
    digitalWrite(m == LEFT ? in2Pin : in4Pin, speed <= 0 ? HIGH : LOW);
    analogWrite(m == LEFT ? enAPin : enBPin, abs(speed));
}

// Initial motor test
void testMotors() {
    static int speed[] = {128, 255, 128, 0, 128, 255, 128, 0};
    go(RIGHT, 0);
    for (unsigned char i = 0; i < 8; i++) {
        go(LEFT, speed[i]);
        delay(200);
    }
    for (unsigned char i = 0; i < 8; i++) {
        go(RIGHT, speed[i]);
        delay(200);
    }
}

// Read distance from the ultrasonic sensor, return distance in mm
unsigned int readDistance() {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long period = pulseIn(echoPin, HIGH);
    return period * 343 / 2000; // Convert time to distance in mm
}

#define NUMANGLES 7
unsigned char sensorAngle[NUMANGLES] = {60, 70, 80, 90, 100, 110, 120};
unsigned int distance[NUMANGLES];

// Scan the area ahead by sweeping the ultrasonic sensor left and right
void readNextDistance() {
    static unsigned char angleIndex = 0;
    static signed char step = 1;
    distance[angleIndex] = readDistance();
    angleIndex += step;
    if (angleIndex == NUMANGLES - 1) step = -1;
    else if (angleIndex == 0) step = 1;
    servo.write(sensorAngle[angleIndex]);
}

// Initial configuration
void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
    pinMode(enAPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);
    pinMode(enBPin, OUTPUT);
    
    servo.attach(servoPin);
    servo.write(90);
    go(LEFT, 0);
    go(RIGHT, 0);
    
    testMotors();
    
    // Scan the surroundings before starting
    servo.write(sensorAngle[0]);
    delay(200);
    for (unsigned char i = 0; i < NUMANGLES; i++) {
        readNextDistance();
        delay(200);
    }
}

// Main loop
void loop() {
    readNextDistance();
    
    // Check if something is too close
    bool tooClose = false;
    for (unsigned char i = 0; i < NUMANGLES; i++) {
        if (distance[i] < 300) {
            tooClose = true;
            break;
        }
    }
    
    if (tooClose) {
        // Something's nearby: backup left
        go(LEFT, 180);
        go(RIGHT, 80);
    } else {
        // Nothing in our way: go forward
        go(LEFT, 255);
        go(RIGHT, 255);
    }
    
    // Check the next direction in 50ms
    delay(50);
}
