import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication with Arduino
try:
    arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
    time.sleep(2)  # Wait for the serial connection to initialize
except serial.SerialException as e:
    print(f"Error initializing serial connection: {e}")
    exit()

# Initialize Mediapipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

# Function to detect individual fingers (1 for up, 0 for down)
def detect_fingers(hand_landmarks):
    finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky
    thumb_tip = 4
    finger_states = [0, 0, 0, 0, 0]  # Thumb, Index, Middle, Ring, Pinky

    # Check thumb
    if hand_landmarks.landmark[thumb_tip].x < hand_landmarks.landmark[thumb_tip - 1].x:
        finger_states[0] = 1  # Thumb is up

    # Check the other fingers
    for idx, tip in enumerate(finger_tips):
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            finger_states[idx + 1] = 1  # Other fingers are up

    return finger_states

# Start capturing video
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Error: Unable to read from the camera.")
        break

    # Flip and process the image
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            fingers_state = detect_fingers(hand_landmarks)
            try:
                arduino.write(bytearray(fingers_state))  # Send list of fingers as bytes
                print(f"Fingers State: {fingers_state}")

                # Check for "index and pinky up" gesture
                if fingers_state == [0, 0, 0, 0, 1]:
                    print(" pinky gesture detected. Closing the program.")
                    cap.release()
                    cv2.destroyAllWindows()
                    arduino.close()
                    exit()
            except serial.SerialException as e:
                print(f"Error writing to Arduino: {e}")

    cv2.imshow('Hand Tracking', image)
    if cv2.waitKey(5) & 0xFF == 27:  # Exit on pressing 'Escape'
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()