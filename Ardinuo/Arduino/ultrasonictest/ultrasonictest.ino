// Define the connection pins
const int trigPin = 13;  // Trigger pin for ultrasonic sensor
const int echoPin = 12;  // Echo pin for ultrasonic sensor

void setup() {
    // Begin serial communication at a baud rate of 9600
    Serial.begin(9600);

    // Set the trigger pin as an output and the echo pin as an input
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void loop() {
    // Ensure the trigger pin is low for a clean high pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    
    // Trigger the sensor by sending a high pulse of at least 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the time taken for the echo to return
    unsigned long duration = pulseIn(echoPin, HIGH);
    
    // Calculate the distance: speed of sound = 343 meters per second = 0.343 mm/microsecond
    // Distance = (Speed of Sound * Time) / 2
    float distance = (duration * 0.343) / 2;
    
    // Print the distance in millimeters
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");

    // Delay between measurements
    delay(1000);
}
