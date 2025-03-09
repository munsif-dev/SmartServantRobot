const int pirPin = 9;  // PIR sensor connected to pin 9

void setup() {
    pinMode(pirPin, INPUT);
    Serial.begin(9600);
}

void loop() {
    int motion = digitalRead(pirPin);

    if (motion == HIGH) {
        Serial.println("Motion Detected!");
        delay(100);  // Wait for 5 seconds before checking again
    } else {
        Serial.println("No Motion");
    }
    
    delay(500);
}
