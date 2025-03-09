#include <Servo.h>

// Define servo for direction control
Servo servo;

// Define pin connections
const int pirPin = 2;     // PIR motion sensor pin
const int servoPin = 3;   // Servo pin to control direction

// Motor driver pins for L298N
const int enAPin = 6;     // PWM control for motor speed
const int in1Pin = 7;     // Motor direction pin 1
const int in2Pin = 5;     // Motor direction pin 2
const int enBPin = 9;     // PWM control for motor speed
const int in3Pin = 8;     // Motor direction pin 1
const int in4Pin = 4;     // Motor direction pin 2

volatile bool motionDetected = false;

// Motor identification for control functions
enum Motor { LEFT, RIGHT };

// Function to control motor directions and speeds
void go(Motor m, int speed) {
    if (m == LEFT) {
        digitalWrite(in1Pin, speed > 0 ? HIGH : LOW);
        digitalWrite(in2Pin, speed <= 0 ? HIGH : LOW);
        analogWrite(enAPin, abs(speed));
    } else {
        digitalWrite(in3Pin, speed > 0 ? HIGH : LOW);
        digitalWrite(in4Pin, speed <= 0 ? HIGH : LOW);
        analogWrite(enBPin, abs(speed));
    }
    Serial.print(m == LEFT ? "Left" : "Right");
    Serial.print(" motor speed set to ");
    Serial.println(speed);
}

void motionDetectedISR() {
    motionDetected = true;
}

// Function to rotate servo to serve and return to initial position
void serve() {
    Serial.println("Motion detected, serving now...");
    servo.write(180);  // Move servo to 180 degrees to serve
    delay(5000);       // Wait for 5 seconds while serving
    servo.write(0);    // Return servo to 0 degrees
    Serial.println("Returned to initial position");
}

// Setup function to initialize components
void setup() {
    Serial.begin(9600);
    pinMode(pirPin, INPUT);
    pinMode(enAPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enBPin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);

    servo.attach(servoPin);
    servo.write(0);    // Start with the servo at 0 degrees
    Serial.println("Setup complete. Robot ready.");

    attachInterrupt(digitalPinToInterrupt(pirPin), motionDetectedISR, RISING);

    go(LEFT, 0);       // Ensure motors are off
    go(RIGHT, 0);      // Ensure motors are off
}

// Main loop function
void loop() {
    if (digitalRead(pirPin) == HIGH) {  // Check if motion is detected
        Serial.println("Motion detected - stopping motors");
        go(LEFT, 0);                    // Stop left motor
        go(RIGHT, 0);                   // Stop right motor
        serve();                        // Perform serving action

        Serial.println("Waiting 10 seconds before next action...");
        delay(10000);                   // Wait for 10 seconds before moving again

        // Change direction randomly after serving
        int randomDirection = random(0, 2);
        int turnSpeed = 150;
        Serial.print("Changing direction: ");
        if (randomDirection == 0) {
            go(LEFT, -turnSpeed);
            go(RIGHT, turnSpeed);
            Serial.println("Turning left");
        } else {
            go(LEFT, turnSpeed);
            go(RIGHT, -turnSpeed);
            Serial.println("Turning right");
        }
        delay(2000);                    // Turn for 2 seconds
        go(LEFT, 100);  // Continue moving forward
        go(RIGHT, 100); // Continue moving forward
        Serial.println("Moving forward");
        delay(50);       // Small delay to manage control frequency
    }

    
}
