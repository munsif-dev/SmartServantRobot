#include <Servo.h>

// Pin definitions
#define TRIG_PIN 9    // Ultrasonic Trigger pin
#define ECHO_PIN 10   // Ultrasonic Echo pin
#define PIR_PIN 8     // PIR Sensor Pin
#define ENA 6         // Motor A (Right Motor) Speed Control
#define IN1 2         // Motor A Direction 1
#define IN2 3         // Motor A Direction 2
#define ENB 5         // Motor B (Left Motor) Speed Control
#define IN3 4         // Motor B Direction 1
#define IN4 7         // Motor B Direction 2
#define SERVO_PIN 11  // Servo Motor Pin

// State definitions
#define STATE_NORMAL 0       // Normal obstacle detection mode
#define STATE_CHECK_MOTION 1 // Checking for human motion after obstacle detection
#define STATE_INTERACTION 2  // Human interaction mode
#define STATE_AVOID 3        // Obstacle avoidance mode

// Constants
#define OBSTACLE_THRESHOLD 10   // cm (reduced to avoid false detections)
#define MOTION_CHECK_TIMEOUT 4000 // ms to wait for motion detection
#define INTERACTION_TIME 8000    // ms to wait during human interaction
#define TURN_TIME 300           // ms for turning actions
#define MOTOR_SPEED 100         // PWM value (0-255)
#define NUM_READINGS 5          // Number of readings to average

Servo myServo;  
long distance = 0;
int servoAngle = 90;  // Default center position
int currentState = STATE_NORMAL;
unsigned long stateTimer = 0;
boolean pathFound = false;
int clearPathAngle = 90;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // Center the servo
  
  // Initialize motors to stopped state
  stopMotors();
  
  Serial.begin(9600);
  Serial.println("Smart Servant Robot Initializing...");
  
  // Longer initialization delay to stabilize sensors
  delay(2000);
  
  // Test ultrasonic sensor to verify it's working correctly
  Serial.println("Testing ultrasonic sensor:");
  for (int i = 0; i < 3; i++) {
    long testDistance = getDistance();
    Serial.print("Test reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(testDistance);
    Serial.println(" cm");
    delay(300);
  }
  
  Serial.println("Initialization complete. Starting navigation.");
}

void loop() {
  // State machine based on the flow chart
  switch (currentState) {
    
    case STATE_NORMAL:
      // Normal operation - checking for obstacles
      myServo.write(90); // Center the servo
      
      // Wait a moment to ensure servo is centered before taking a reading
      static unsigned long lastReadTime = 0;
      if (millis() - lastReadTime >= 100) {  // Take readings every 100ms
        distance = getDistance();
        lastReadTime = millis();
        
        // Print distance for debugging
        Serial.print("Current distance: ");
        Serial.print(distance);
        Serial.println(" cm");
        
        if (distance < OBSTACLE_THRESHOLD) {
          // Verify obstacle with a second reading to avoid false triggers
          delay(50);
          long confirmDistance = getDistance();
          
          if (confirmDistance < OBSTACLE_THRESHOLD) {
            // Obstacle confirmed - transition to checking for motion
            Serial.println("Obstacle detected! Checking for human motion...");
            stopMotors();
            currentState = STATE_CHECK_MOTION;
            stateTimer = millis(); // Start timer for motion detection window
          }
        }
      }
      
      // Only move forward if still in NORMAL state
      if (currentState == STATE_NORMAL) {
        moveForward();
      }
      break;
      
    case STATE_CHECK_MOTION:
      // Check for human motion after obstacle detected
      if (digitalRead(PIR_PIN) == HIGH) {
        // Human motion detected - enter interaction mode
        Serial.println("Motion detected! Entering interaction mode.");
        currentState = STATE_INTERACTION;
        stateTimer = millis(); // Start timer for interaction time
      } 
      else if (millis() - stateTimer > MOTION_CHECK_TIMEOUT) {
        // No motion detected within timeout - scan and avoid obstacle
        Serial.println("No motion detected. Scanning for path to avoid obstacle...");
        pathFound = scanForPath();
        currentState = STATE_AVOID;
      }
      break;
      
    case STATE_INTERACTION:
      // Human interaction mode - just wait
      if (millis() - stateTimer > INTERACTION_TIME ) {
        // Interaction time complete
       
        Serial.println("Interaction complete. Resuming normal operation.");
        currentState = STATE_NORMAL;
      }
      break;
      
    case STATE_AVOID:
      // Avoid obstacle based on scan results
      if (pathFound) {
        Serial.print("Clear path found at angle: ");
        Serial.println(clearPathAngle);
        avoidObstacle();
      } else {
        Serial.println("No clear path found. Moving backward and turning.");
        moveBackward();
        delay(1000);
        stopMotors();
        turnRight();
        delay(TURN_TIME);
        stopMotors();
      }
      currentState = STATE_NORMAL; // Return to normal operation
      break;
  }
  
  // Small delay for stability
  delay(50);
}



boolean scanForPath() {
  boolean found = false;
  long distances[2]; // Store distances at only two angles (left and right)
  
  // Scan right side first (right priority to solve left-turning issue)
  myServo.write(135);
  delay(500); // Give servo time to move
  distances[1] = getDistance();
  Serial.print("Distance at angle 135 (right): ");
  Serial.print(distances[1]);
  Serial.println(" cm");
  
  // Check left side
  myServo.write(45);
  delay(500);
  distances[0] = getDistance();
  Serial.print("Distance at angle 45 (left): ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  // Find the best path - look for the longest distance first
  long maxDistance = 0;
  int bestAngleIndex = -1;
  
  for (int i = 0; i < 2; i++) {
    if (distances[i] > maxDistance && distances[i] > OBSTACLE_THRESHOLD) {
      maxDistance = distances[i];
      bestAngleIndex = i;
    }
  }
  
  // Translate the index to angle
  if (bestAngleIndex >= 0) {
    found = true;
    switch (bestAngleIndex) {
      case 0: clearPathAngle = 45; break; // Left
      case 1: clearPathAngle = 135; break; // Right
    }
    
    Serial.print("Best path found at angle: ");
    Serial.print(clearPathAngle);
    Serial.print(" with distance: ");
    Serial.print(maxDistance);
    Serial.println(" cm");
  } else {
    Serial.println("No clear path found in any direction.");
  }
  
  return found;
}

void avoidObstacle() {
  // Turn based on the clear path angle
  stopMotors();
  
  if (clearPathAngle == 45) {
    // Clear path is to the left
    Serial.println("Turning left to follow clear path");
    turnLeft();
    delay(TURN_TIME * 2); // Consistent turn time for left
  } 
  else if (clearPathAngle == 135) {
    // Clear path is to the right
    Serial.println("Turning right to follow clear path");
    turnRight();
    delay(TURN_TIME * 2); // Consistent turn time for right
  }
  
  stopMotors();
  myServo.write(90); // Center the servo again
}

long getDistance() {
  // Take multiple readings and average them to reduce noise
  long total = 0;
  long validReadings = 0;
  
  for (int i = 0; i < NUM_READINGS; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Add timeout to prevent hanging
    
    // Check if reading is valid (non-zero)
    if (duration > 0) {
      long distance = (duration * 0.0343) / 2;
      
      // Filter out obviously erroneous readings (too far or too close)
      if (distance > 0 && distance < 400) {
        total += distance;
        validReadings++;
      }
    }
    
    delay(10); // Small delay between readings
  }
  
  // If we got valid readings, return the average
  if (validReadings > 0) {
    return total / validReadings;
  } else {
    // If no valid readings, return a safe value
    return 100; // Assume no obstacles if readings are problematic
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}