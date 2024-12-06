// Motor L298N
int dirRight = 7; // if high forward
int mRight = 6; 
int dirLeft = 4; // if high forward
int mLeft = 5; 
#define lpwm 8
#define rpwm 9

int sensor[5] = {0, 0, 0, 0, 0}; // Line sensors
int weights[5] = {-2, -1, 0, 1, 2}; // Weighting for sensors
#define OBSTACLE_PIN A5 

// Base speed and PID constants
int baseSpeed = 60; // Base speed for motors
int turnSpeed = 70;
float Kp = 15;      // Proportional gain
float Ki = 0;       // Integral gain (disabled)
float Kd = 30;      // Derivative gain
float error = 0, prevError = 0, derivative = 0;
int correction = 0;

// Bluetooth module
#define RX_PIN 0
#define TX_PIN 1 
int destinationTable = 0;// Destination table received via Bluetooth

// Ultrasonic Sensor Pins
#define TRIG_PIN 13
#define ECHO_PIN 12
int isTray;

void setup() {
  Serial.begin(9600);
  // Motor L298N setup
  pinMode(mRight, OUTPUT);
  pinMode(dirRight, OUTPUT);
  pinMode(mLeft, OUTPUT);
  pinMode(dirLeft, OUTPUT);
  pinMode(lpwm, OUTPUT);
  pinMode(rpwm, OUTPUT);
  // Line sensor setup
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  // Obstacle sensor setup
  pinMode(OBSTACLE_PIN, INPUT);
}

void loop() {
  //Check for input from Bluetooth module
  if (Serial.available()) {
    char input = Serial.read(); // Read input from the Bluetooth module
    Serial.print("Received input: ");
    Serial.println(input);

    switch (input) {
      case '1':
        tablePath1();
        break;
      case '2':
        tablePath2();
        break;
      case '3':
        tablePath3();
        break;
      case '4':
        tablePath4();
        break;
      default:
        Serial.println("Invalid input. Please enter 1, 2, 3, or 4.");
        break;
    }
  }
}

// Unified function for line-following control
void lineFollowingControl() {
  handleObstacle();       // Check for obstacles
  readSensors();          // Read sensor values
  bool allWhite = true;
  for (int i = 0; i < 5; i++) {
    if (sensor[i] == 0) { // If any sensor detects black
      allWhite = false;
      break;
    }
  }

  if (allWhite) {
    stopMotors(); // Stop motors if no line is detected
    Serial.println("No line detected. Stopping motors.");
    return; // Exit the function to prevent further motor adjustments
  }
  calculatePID();         // Calculate PID correction

  // Adjust motor speeds based on PID correction
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  moveMotors(leftSpeed, rightSpeed); // Move the motors
}

// Function to handle obstacle detection
void handleObstacle() {
  if (isObstacleDetected()) {
    stopMotors(); // Stop motors if an obstacle is detected
    while (isObstacleDetected()) {
      delay(100); // Small delay to reduce CPU load
    }
  }
}

// Function to check for obstacles
bool isObstacleDetected() {
  return digitalRead(OBSTACLE_PIN) == HIGH; // LOW indicates an obstacle is detected
}

// Function to stop motors
void stopMotors() {
  analogWrite(lpwm, 0);
  analogWrite(rpwm, 0);

  digitalWrite(dirRight, LOW);
  digitalWrite(mRight, LOW);
  digitalWrite(dirLeft, LOW);
  digitalWrite(mLeft, LOW);
}

// Function to read line sensors
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensor[i] = digitalRead(A0 + i);
  }
}

// Function to calculate PID
void calculatePID() {
  // Calculate weighted error based on sensor readings
  int total = 0;
  int activeSensors = 0;

  for (int i = 0; i < 5; i++) {
    if (sensor[i] == 0) { // Assuming 0 means the sensor is on the line
      total += weights[i];
      activeSensors++;
    }
  }

  // If no sensors detect the line, keep the previous error
  if (activeSensors > 0) {
    error = (float)total / activeSensors; // Average error
  } else {
    error = prevError; // Maintain the last known error
  }

  // Apply dead zone to reduce small oscillations
  if (abs(error) < 0.5) {
    error = 0;
  }

  // PID calculations
  derivative = error - prevError;
  correction = (Kp * error) + (Kd * derivative);

  prevError = error;
  // Debugging output (optional)
  Serial.print("Error: "); Serial.println(error);
  Serial.print("Correction: "); Serial.println(correction);
}

// Function to move motors with speed adjustment
void moveMotors(int leftSpeed, int rightSpeed) {
  // Constrain speeds to valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive motors
  analogWrite(lpwm, leftSpeed);
  analogWrite(rpwm, rightSpeed);

  digitalWrite(dirRight, HIGH);
  digitalWrite(mRight, LOW);
  digitalWrite(dirLeft, HIGH);
  digitalWrite(mLeft, LOW);
}

//Tray Detection Funtion
void isTrayDetected(){
  long duration, distance;

  // Send a 10-microsecond pulse to the TRIG pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin and calculate the distance
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;  // Convert to centimeters

  // Check the distance and set the isTray variable
  if (distance > 1 && distance < 400) {  // Assuming 400 cm as the max valid range
    isTray = 0;  // Tray is not detected
  } else {
    isTray = 1;  // Tray is detected
  }
  //Serial.println(isTray);

  delay(100);  // Delay for readability
}
void turn90Left() {
  // Stop the robot momentarily
  stopMotors();
  delay(100);

  // Rotate left (left motor backward, right motor forward)
  digitalWrite(mLeft, HIGH);   // Left motor backward
  analogWrite(lpwm, baseSpeed);

  digitalWrite(dirRight, HIGH); // Right motor forward
  analogWrite(rpwm, turnSpeed);

  delay(240); // Adjust this delay for a precise 90° turn
  stopMotors();
}

void turn90Right() {
  // Stop the robot momentarily
  stopMotors();
  delay(100);

  // Rotate right (left motor forward, right motor backward)
  digitalWrite(dirLeft, HIGH);  // Left motor forward
  analogWrite(lpwm, baseSpeed);

  digitalWrite(mRight, HIGH);  // Right motor backward
  analogWrite(rpwm, turnSpeed);

  delay(240); // Adjust this delay for a precise 90° turn
  stopMotors();
}

void turn180() {
  // Stop the robot momentarily
  stopMotors();
  delay(100);

  // Rotate left (left motor backward, right motor forward)
  digitalWrite(mLeft, HIGH);   // Left motor backward
  analogWrite(lpwm, turnSpeed);

  digitalWrite(dirRight, HIGH); // Right motor forward
  analogWrite(rpwm, turnSpeed);

  delay(480); // Adjust this delay for a precise 90° turn
  stopMotors();
}

void tablePath1() {
  while (true) {
    lineFollowingControl();

    if (digitalRead(A0) == LOW) {
      turn90Left(); 

      while (true) {
        lineFollowingControl();
        if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
          stopMotors();
          isTrayDetected();

          // Stay at table until the tray is removed
          while (isTray == 1) {  // If tray is still on the robot
            isTrayDetected();   // Continuously check tray status
            delay(500);         // Small delay to reduce CPU load
          }

          // Once tray is removed, wait for 3 seconds and turn 180 degrees
          delay(3000); 
          turn180();
          
          // Return to start position
          while (true) {
            lineFollowingControl();
            if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
              turn90Right();
              while (true) {
                lineFollowingControl();
                if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
                  turn180();
                  stopMotors();
                  return;  // Exit the function, ending the robot's path
                }
              }
            }
          }
        }
      }
    }
  }
}
void tablePath2() {
  while (true) {
    lineFollowingControl();

    // Assuming A1 sensor or another condition indicates we're near Table 2
    if (digitalRead(A4) == LOW) {
      turn90Right();

      while (true) {
        lineFollowingControl();

        if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
          stopMotors();
          isTrayDetected();

          // Stay at table until the tray is removed
          while (isTray == 1) {  // If tray is still on the robot
            isTrayDetected();   // Continuously check tray status
            delay(500);         // Small delay to reduce CPU load
          }

          // Once tray is removed, wait for 3 seconds and turn 180 degrees
          delay(3000); 
          turn180();
          
          // Return to start position
          while (true) {
            lineFollowingControl();
            if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
              turn90Left(); // Turn back to return to the start
              while (true) {
                lineFollowingControl();
                if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
                  turn180(); // Final turn back to starting point
                  stopMotors();
                  return;  // Exit the function, ending the robot's path
                }
              }
            }
          }
        }
      }
    }
  }
}
void tablePath3() {
  while (true) {
    lineFollowingControl();

    // Assuming A1 sensor or another condition indicates we're near Table 2
    if (digitalRead(A0) == LOW  && digitalRead(A4) == LOW) {
      turn90Left();

      while (true) {
        lineFollowingControl();

        if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
          stopMotors();
          isTrayDetected();

          // Stay at table until the tray is removed
          while (isTray == 1) {  // If tray is still on the robot
            isTrayDetected();   // Continuously check tray status
            delay(500);         // Small delay to reduce CPU load
          }

          // Once tray is removed, wait for 3 seconds and turn 180 degrees
          delay(3000); 
          turn180();
          
          // Return to start position
          while (true) {
            lineFollowingControl();
            if (digitalRead(A4) == LOW) {
              turn90Right(); // Turn back to return to the start
              while (true) {
                lineFollowingControl();
                if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
                  turn180(); // Final turn back to starting point
                  stopMotors();
                  return;  // Exit the function, ending the robot's path
                }
              }
            }
          }
        }
      }
    }
  }
}
void tablePath4() {
  while (true) {
    lineFollowingControl();

    // Assuming A1 sensor or another condition indicates we're near Table 2
    if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
      turn90Right();

      while (true) {
        lineFollowingControl();

        if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
          stopMotors();
          isTrayDetected();

          // Stay at table until the tray is removed
          while (isTray == 1) {  // If tray is still on the robot
            isTrayDetected();   // Continuously check tray status
            delay(500);         // Small delay to reduce CPU load
          }

          // Once tray is removed, wait for 3 seconds and turn 180 degrees
          delay(3000); 
          turn180();
          
          // Return to start position
          while (true) {
            lineFollowingControl();
            if (digitalRead(A0) == LOW) {
              turn90Left(); // Turn back to return to the start
              while (true) {
                lineFollowingControl();
                if (digitalRead(A0) == LOW && digitalRead(A4) == LOW) {
                  turn180(); // Final turn back to starting point
                  stopMotors();
                  return;  // Exit the function, ending the robot's path
                }
              }
            }
          }
        }
      }
    }
  }
}
