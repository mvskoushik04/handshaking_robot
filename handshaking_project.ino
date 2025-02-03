#include <Servo.h>

Servo handshakeServo; // Define a servo for the handshake mechanism
const int trigPin = 9; // Ultrasonic sensor trigger pin
const int echoPin = 10; // Ultrasonic sensor echo pin
const int servoPin = 6; // Servo motor control pin
bool handshakeTriggered = false; // Prevent multiple activations

void setup() {
    Serial.begin(9600);
    handshakeServo.attach(servoPin);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    handshakeServo.write(0); // Set servo to initial position
}

long measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // Convert to cm
    return distance;
}

void loop() {
    if (Serial.available() > 0) {
        char receivedChar = Serial.read();

        if (receivedChar == '1' && !handshakeTriggered) {
            Serial.println("Hand detected, checking distance...");

            long distance = measureDistance();
            Serial.println(distance); // Send distance to Python

            if (distance > 5 && distance < 20) { // Distance range for handshake
                Serial.println("Initiating handshake...");
                handshakeServo.write(90); // Move servo to handshake position
                delay(1000);
                handshakeServo.write(0); // Reset servo
                handshakeTriggered = true; // Prevent repeated handshakes
            }
        }
    }
    delay(100); // Small delay for stability
}
