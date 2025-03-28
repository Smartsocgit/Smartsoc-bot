import cv2
import mediapipe as mp
import serial
import time
import pygame  # Import pygame for audio playback
import threading  # Import threading for parallel processing
import speech_recognition as sr  # Import speech recognition

# Initialize pygame mixer for audio
pygame.mixer.init()

# Load the audio files (MP3)
audio_face_detected = "face_detected.mp3"  # Replace with your actual file
audio_no_face_detected = "no_face_detected.mp3"  # Replace with your actual file

# Setup MediaPipe face detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Video capture and serial communication setup
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

try:
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # Make sure this is the correct port
    print("Serial connection established.")
except Exception as e:
    print(f"Error: Could not open serial port. {e}")
    exit()

# Global variables for face tracking and flags
prev_x = prev_y = None
last_face_center_x = last_face_center_y = None
last_face_detected_time = time.time()  # Track the last time a face was detected
no_face_duration = 5  # Time in seconds to wait before sending center coordinates
face_detected = False  # Flag to track if a face has been detected in the current loop
face_audio_played = False  # Flag to prevent audio from repeating continuously when face is detected
no_face_audio_played = False  # Flag to prevent audio from repeating continuously when no face detected
face_was_detected_previous = False  # Flag to track if face was detected previously
sleep_mode = False  # Flag to track if the system should be in sleep mode


def play_audio(audio_file):
    """Play an audio file."""
    pygame.mixer.music.load(audio_file)
    pygame.mixer.music.play()

    # Wait until the audio is finished playing before continuing
    while pygame.mixer.music.get_busy():
        time.sleep(0.1)  # Wait for 0.1 seconds to avoid CPU overload


def audio_thread(audio_file, play_condition_flag):
    """This thread is responsible for playing the audio only when required."""
    global face_audio_played, no_face_audio_played

    while True:
        if play_condition_flag():  # Check the condition whether to play audio
            play_audio(audio_file)
        time.sleep(0.1)  # Allow other threads to work


def face_detection_thread():
    """This thread is responsible for detecting faces and sending data."""
    global prev_x, prev_y, face_detected, last_face_center_x, last_face_center_y, face_audio_played, no_face_audio_played, last_face_detected_time, face_was_detected_previous, sleep_mode

    while True:
        if sleep_mode:
            time.sleep(1)
            continue  # Skip the face detection logic when in sleep mode
        
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detection.process(rgb_frame)

        if results.detections:
            # Face detected
            face_detected = True
            last_face_detected_time = time.time()  # Reset timer when a face is detected

            # Sort detections based on bounding box area, largest first
            detections = sorted(results.detections, key=lambda x: x.location_data.relative_bounding_box.width * x.location_data.relative_bounding_box.height, reverse=True)

            # Take the first (largest) face
            detection = detections[0]
            mp_draw.draw_detection(frame, detection)
            bboxC = detection.location_data.relative_bounding_box
            h, w, _ = frame.shape
            x1, y1, w1, h1 = int(bboxC.xmin * w), int(bboxC.ymin * h), int(bboxC.width * w), int(bboxC.height * h)

            face_center_x = x1 + w1 // 2
            face_center_y = y1 + h1 // 2

            # Track only the largest face and send the data if it has moved
            if prev_y is None or abs(prev_y - face_center_y) > 5:
                ser.write(f'Y{face_center_y}\n'.encode())
                prev_y = face_center_y

            if prev_x is None or abs(prev_x - face_center_x) > 5:
                ser.write(f'X{face_center_x}\n'.encode())
                prev_x = face_center_x

            ser.write(f'M{face_center_y}\n'.encode())

            # Update the coordinates of the last tracked face
            last_face_center_x = face_center_x
            last_face_center_y = face_center_y

            # If face was detected previously, don't repeat the audio
            if not face_was_detected_previous:
                # Reset the flag so audio is only played once after face enters
                face_audio_played = False

            # Update the face_was_detected_previous flag
            face_was_detected_previous = True

            # Reset no face audio flag when face is detected
            no_face_audio_played = False  # Allow no face audio to play again if face is detected

            cv2.putText(frame, f"Face Center: ({face_center_x}, {face_center_y})", (face_center_x, face_center_y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        else:
            # No face detected
            face_detected = False
            face_was_detected_previous = False  # Reset flag when face is lost
            if time.time() - last_face_detected_time >= no_face_duration:
                # If no face detected for 5 seconds, send the center coordinates
                if last_face_center_x is None or last_face_center_y is None:
                    last_face_center_x, last_face_center_y = 320, 240  # Default center values for a 640x480 frame
                ser.write(f'X{last_face_center_x}\n'.encode())
                ser.write(f'Y{last_face_center_y}\n'.encode())
                ser.write(f'M{last_face_center_y}\n'.encode())

                # Set no_face_audio_played to False to allow the audio to play again after 5 seconds
                if not no_face_audio_played:
                    no_face_audio_played = True  # Audio can be played only once when no face is detected for 5 seconds

                cv2.putText(frame, "No Face Detected for 5 seconds", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Display the resulting frame
        cv2.imshow('Face Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Reset flags when no face is detected
        if not face_detected:
            face_audio_played = False  # Allow face audio to play again if face is detected
            # Do not reset no_face_audio_played here

    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("Video capture stopped.")


def face_detected_audio_condition():
    """Condition for playing face detected audio."""
    global face_audio_played, face_detected
    if not face_audio_played and face_detected:
        face_audio_played = True
        return True
    return False


def no_face_detected_audio_condition():
    """Condition for playing no face detected audio."""
    global no_face_audio_played
    if not no_face_audio_played and not face_detected:
        no_face_audio_played = True
        return True
    return False


def listen_for_sleep_or_wake():
    """Function to listen for the words 'sleep' or 'wake' to toggle the system state."""
    global sleep_mode

    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    while True:
        with microphone as source:
            print("Listening for 'sleep' or 'wake' command...")
            recognizer.adjust_for_ambient_noise(source)  # Adjust for ambient noise
            audio = recognizer.listen(source)

            try:
                # Recognize speech using Google Web Speech API
                command = recognizer.recognize_google(audio).lower()
                print(f"Recognized command: {command}")

                if 'sleep' in command:
                    sleep_mode = True  # Enable sleep mode
                    print("Sleep mode enabled.")
                elif 'wake' in command:
                    sleep_mode = False  # Disable sleep mode
                    print("Sleep mode disabled (wakeup).")

            except sr.UnknownValueError:
                print("Could not understand audio.")
            except sr.RequestError as e:
                print(f"Error with the speech recognition service: {e}")
            time.sleep(1)  # Wait before listening again


if __name__ == '__main__':
    try:
        # Start the face detection thread
        face_detection_thread = threading.Thread(target=face_detection_thread, daemon=True)
        face_detection_thread.start()

        # Start the audio threads
        audio_face_detected_thread = threading.Thread(target=audio_thread, args=(audio_face_detected, face_detected_audio_condition), daemon=True)
        audio_face_detected_thread.start()

        audio_no_face_detected_thread = threading.Thread(target=audio_thread, args=(audio_no_face_detected, no_face_detected_audio_condition), daemon=True)
        audio_no_face_detected_thread.start()

        # Start the speech recognition thread
        listen_for_sleep_or_wake_thread = threading.Thread(target=listen_for_sleep_or_wake, daemon=True)
        listen_for_sleep_or_wake_thread.start()

        # Keep the main thread alive to allow the other threads to work
        while True:
            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
