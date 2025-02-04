#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

// Create PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo definitions (change these as needed)
#define BASE_SERVO_CHANNEL 0   // Base rotation servo (Channel 0 on PCA9685)
#define SHOULDER_SERVO_CHANNEL 1   // Shoulder movement servo (Channel 1)
#define ELBOW_SERVO_CHANNEL 2  // Elbow movement servo (Channel 2)
#define WRIST_SERVO_PIN 9      // Wrist servo connected directly to Arduino pin 9

// Ultrasonic sensor pins
#define TRIG_PIN 6  // Trigger pin of the ultrasonic sensor
#define ECHO_PIN 7  // Echo pin of the ultrasonic sensor

Servo wristServo;  // Create Servo object for wrist

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize PWM driver and set frequency to 50 Hz (for servo control)
  pwm.begin();
  pwm.setPWMFreq(50);

  // Attach the wrist servo to the specified pin
  wristServo.attach(WRIST_SERVO_PIN);

  // Set the initial position of the servos
  pwm.setPWM(BASE_SERVO_CHANNEL, 0, servoAngleToPWM(90));   // Base in the center
  pwm.setPWM(SHOULDER_SERVO_CHANNEL, 0, servoAngleToPWM(90));   // Shoulder in the center
  pwm.setPWM(ELBOW_SERVO_CHANNEL, 0, servoAngleToPWM(90));  // Elbow in the center
  wristServo.write(90);  // Wrist in the center position

  // Initialize pins for the ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("System ready. Waiting for signal...");
}
void loop() {
  // Check for incoming serial data from the laptop
  if (Serial.available() > 0) {
    char signal = Serial.read();

    if (signal == '1') {  // Signal '1' received from Python (left hand detected)
      Serial.println("Left hand detected. Starting handshake...");

      // Measure distance to target using ultrasonic sensor
      long distance = measureDistance();
      Serial.print("Distance to target: ");
      Serial.print(distance);
      Serial.println(" cm");

      // Move robotic arm according to the measured distance
      performHandshakeSequence(distance);
    }
  }
}

// Function to measure distance using the ultrasonic sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;  // Convert duration to distance in cm
  return distance;
}

// Function to perform handshake sequence based on distance
void performHandshakeSequence(long distance) {
  // Example movements (you can modify these as needed)
  if (distance < 20) {  // Close distance: perform full handshake
    pwm.setPWM(SHOULDER_SERVO_CHANNEL, 0, servoAngleToPWM(60));  // Lower shoulder
    delay(500);
    pwm.setPWM(ELBOW_SERVO_CHANNEL, 0, servoAngleToPWM(120));  // Extend elbow
    delay(500);
    wristServo.write(45);  // Rotate wrist for handshake
    delay(500);
    wristServo.write(90);  // Return wrist to normal position
    delay(500);
    pwm.setPWM(ELBOW_SERVO_CHANNEL, 0, servoAngleToPWM(90));  // Return elbow to normal
    delay(500);
    pwm.setPWM(SHOULDER_SERVO_CHANNEL, 0, servoAngleToPWM(90));  // Raise shoulder back
  } else if (distance < 50) {  // Medium distance: partial handshake
    pwm.setPWM(ELBOW_SERVO_CHANNEL, 0, servoAngleToPWM(90));  // Move elbow halfway
    delay(500);
    wristServo.write(60);  // Rotate wrist slightly
    delay(500);
    wristServo.write(90);  // Return wrist to normal position
  } else {  // Far distance: only wave
    pwm.setPWM(BASE_SERVO_CHANNEL, 0, servoAngleToPWM(60));  // Rotate base to wave
    delay(500);
    pwm.setPWM(BASE_SERVO_CHANNEL, 0, servoAngleToPWM(90));  // Return to original position
  }
}

// Helper function to convert servo angle to PCA9685 PWM value
int servoAngleToPWM(int angle) {
  int pwm_value = map(angle, 0, 180, 102, 512);  // Map angle to PWM value for 50 Hz frequency
  return pwm_value;
}