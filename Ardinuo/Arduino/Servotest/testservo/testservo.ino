#include <Servo.h>

Servo servo;
const int servoPin = 11;

void setup() {
    servo.attach(servoPin);
    Serial.begin(9600);
}

void loop() {
    for (int pos = 0; pos <= 180; pos += 90) { // Goes from 0 degrees to 180 degrees in 90-degree steps
        Serial.print("Moving servo to ");
        Serial.print(pos);
        Serial.println("°");
        servo.write(pos);              // Tell servo to go to position in variable 'pos'
        delay(1000);                   // Waits 1s for the servo to reach the position
    }
}
