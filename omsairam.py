import cv2
import mediapipe as mp
from google.protobuf.json_format import MessageToDict
import serial  # Import for Arduino communication
import time

# Initialize MediaPipe Hand solution
mpHands = mp.solutions.hands
hands = mpHands.Hands(
    static_image_mode=False,
    model_complexity=1,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75,
    max_num_hands=2)

# Initialize video capture (webcam)
cap = cv2.VideoCapture(0)

# Automatically detect and connect to Arduino
arduino_port = None
for port in ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', '/dev/ttyUSB0', '/dev/ttyACM0']:
    try:
        arduino = serial.Serial(port, 9600, timeout=1)
        time.sleep(2)  # Wait for the connection to establish
        arduino_port = port
        print(f"Connected to Arduino on {port}")
        break
    except serial.SerialException:
        pass

if not arduino_port:
    print("Could not connect to Arduino. Please check the connection.")
    arduino = None

# Variables for hand detection and communication
left_hand_detected_previous = False
distance_value = "No data"

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)  # Flip the image horizontally
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    display_text = ""

    if results.multi_hand_landmarks:
        left_hand_detected = False
        right_hand_detected = False

        # Identify hands
        for i, hand in enumerate(results.multi_handedness):
            label = MessageToDict(hand)['classification'][0]['label']

            if label == 'Left':
                left_hand_detected = True
            elif label == 'Right':
                right_hand_detected = True

        # If only left hand is detected, prompt the user
        if left_hand_detected and not right_hand_detected:
            display_text = "Please place your right hand before the arm"

            # Send signal only once when the left hand is first detected
            if not left_hand_detected_previous and arduino:
                try:
                    arduino.write(b'1')  # Send signal to Arduino
                    print("Signal sent to Arduino: 1")
                except serial.SerialException:
                    print("Error: Failed to send signal to Arduino.")

            # Read distance value from Arduino
            if arduino and arduino.in_waiting > 0:
                try:
                    distance_value = arduino.readline().decode('utf-8').strip()
                    print(f"Distance: {distance_value} cm")
                except Exception as e:
                    distance_value = "Error reading distance"
                    print(f"Failed to read distance value: {e}")

            left_hand_detected_previous = True
        else:
            left_hand_detected_previous = False

    # Display text and distance on the screen
    cv2.putText(img, display_text, (100, 50),
                cv2.FONT_HERSHEY_COMPLEX,
                0.9, (0, 255, 0), 2)
    cv2.putText(img, f"Distance: {distance_value} cm", (100, 100),
                cv2.FONT_HERSHEY_COMPLEX,
                0.9, (255, 0, 0), 2)

    cv2.imshow('Image', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.05)  # Reduce CPU usage

cap.release()
cv2.destroyAllWindows()

if arduino:
    arduino.close()
