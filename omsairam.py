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

# Specify the correct serial port for the Arduino (e.g., 'COM3', 'COM4' for Windows or '/dev/ttyUSB0' for Linux)
arduino_port = 'COM6'  # Change this to the port your Arduino is connected to
try:
    # Establish a serial communication with the Arduino
    arduino = serial.Serial(arduino_port, 9600, timeout=1)
    time.sleep(2)  # Wait for the connection to establish
except serial.SerialException:
    print(f"Could not open port {arduino_port}. Please check the connection and the port.")
    arduino = None

# Flag to indicate if the left hand was detected in the previous frame
left_hand_detected_previous = False
distance_value = "No data"  # Variable to hold the distance value received from Arduino

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)  # Flip the image horizontally
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    display_text = ""

    # Process detected hands
    if results.multi_hand_landmarks:
        left_hand_detected = False

        # Check for left and right hands
        for i, hand in enumerate(results.multi_handedness):
            label = MessageToDict(hand)['classification'][0]['label']

            if label == 'Left':
                left_hand_detected = True

        # If only left hand is detected, send signal to Arduino and update display text
        if left_hand_detected:
            display_text = "Please place your right hand before the arm"
            
            # If the left hand was not detected in the previous frame, send a signal to Arduino
            if not left_hand_detected_previous and arduino:
                arduino.write(b'1')  # Send '1' as a signal to Arduino
                print("Signal sent to Arduino: 1")

            # Try to read the distance value from the Arduino
            if arduino and arduino.in_waiting > 0:  # Check if there's data to read
                try:
                    distance_value = arduino.readline().decode('utf-8').strip()  # Read and decode the distance value
                    print(f"Distance: {distance_value} cm")
                except:
                    distance_value = "Error reading distance"  # Handle any exceptions while reading
                    print("Failed to read distance value")

            left_hand_detected_previous = True
        else:
            left_hand_detected_previous = False

    # Display the message and distance on the screen
    cv2.putText(img, display_text, (100, 50),
                cv2.FONT_HERSHEY_COMPLEX,
                0.9, (0, 255, 0), 2)

    # Display the distance value
    cv2.putText(img, f"Distance: {distance_value} cm", (100, 100),
                cv2.FONT_HERSHEY_COMPLEX,
                0.9, (255, 0, 0), 2)

    # Show the image
    cv2.imshow('Image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close all windows
cap.release()
cv2.destroyAllWindows()

# Close the serial connection if it was established
if arduino:
    arduino.close()
