#include <Servo.h>

// Pin Definitions
const int pirPin = 9;       // PIR Sensor Pin
const int trigPin = 13;     // Ultrasonic Sensor Trigger Pin
const int echoPin = 12;     // Ultrasonic Sensor Echo Pin
const int servoPin = 11;    // Servo Motor Pin

Servo servo;

void setup() {
    pinMode(pirPin, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    servo.attach(servoPin);
    Serial.begin(9600);

    // Move servo to the initial position
    servo.write(90);
    delay(1000);
}

void loop() {
    int distance = getDistance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < 30) {  // If obstacle is closer than 30cm
        Serial.println("Obstacle detected! Checking for motion...");
        
        if (digitalRead(pirPin) == HIGH) {  // If motion is detected
            Serial.println("Motion detected! Waiting 5 seconds...");
            delay(5000);  // Wait for 5 seconds
            
            Serial.println("Motion check complete. Moving servo...");
        }

        // Move servo to find another obstacle
        for (int angle = 90; angle <= 120; angle += 10) {
            servo.write(angle);
            delay(500);
            if (getDistance() < 30) {  // Check again at new angle
                Serial.println("New obstacle found! Repeating process...");
                return;
            }
        }
        
        for (int angle = 90; angle >= 60; angle -= 10) {
            servo.write(angle);
            delay(500);
            if (getDistance() < 30) {  // Check again at new angle
                Serial.println("New obstacle found! Repeating process...");
                return;
            }
        }
    }

    delay(500);
}

// Function to measure distance using Ultrasonic Sensor
int getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;  // Convert to cm

    return distance;
}
