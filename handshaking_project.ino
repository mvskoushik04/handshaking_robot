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