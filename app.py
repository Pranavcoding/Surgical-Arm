from flask import Flask, render_template, Response
import cv2
import mediapipe as mp
import numpy as np
from collections import deque

isAnalyzing = False
app = Flask(__name__)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

camera = cv2.VideoCapture(0)
times = 0
the_coords = deque(maxlen=2)
the_coords.append([0, 0, 0])
the_coords.append([0, 0, 0])

@app.route('/')
def home():
    return render_template('index.html')

def generate_frames():
    global times, the_coords

    while True:
        _, frame = camera.read()
        try:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pose_results = pose.process(frame_rgb)
            mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            waist = pose_results.pose_landmarks.landmark[24]
            shoulder = pose_results.pose_landmarks.landmark[12]
            elbow = pose_results.pose_landmarks.landmark[14]
            wrist = pose_results.pose_landmarks.landmark[16]
            finger = pose_results.pose_landmarks.landmark[20]

            shoulder_angle = 180 - two_d_angle(elbow, shoulder, waist)
            elbow_angle = two_d_angle(shoulder, elbow, wrist)
            wrist_angle = two_d_angle(elbow, wrist, finger)

            if times % 100 == 0:
                arduino_shoulder_angle = shoulder_angle
                arduino_elbow_angle = elbow_angle
                arduino_wrist_angle = wrist_angle

                the_coords.append([arduino_shoulder_angle, arduino_elbow_angle, arduino_wrist_angle])
                the_change_in_angle = [the_coords[1][0] - the_coords[0][0], the_coords[1][1] - the_coords[0][1], the_coords[1][2] - the_coords[0][2]]

                print("\n\nTHE ANGLE IS AT SHOULDER:", shoulder_angle)
                print("THE ANGLE IS AT ELBOW:", elbow_angle)
                print("THE ANGLE IS AT WRIST:", wrist_angle)
                print("THE COORDS:", the_coords[0], the_coords[1])
                print("THE CHANGE IN ANGLE:", the_change_in_angle)

            times += 1

            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
        except:
            pass
        
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle_analysis')
def toggle_analysis():
    global isAnalyzing
    isAnalyzing = not isAnalyzing
    if isAnalyzing:
        start_analysis()
    else:
        stop_analysis()
    return 'Analysis toggled.', 200

@app.route('/start_analysis')
def start_analysis():
    global isAnalyzing
    isAnalyzing = True
    # Start your analysis here
    return 'Analysis started.', 200

@app.route('/stop_analysis')
def stop_analysis():
    global isAnalyzing
    isAnalyzing = False
    # Stop your analysis here
    return 'Analysis stopped.', 200


def angle_between_points(classe1, classe2, classe3):
    a = [classe1.x, classe1.y, classe1.z]
    b = [classe2.x, classe2.y, classe2.z]
    c = [classe3.x, classe3.y, classe3.z]
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    ab = b - a
    bc = c - b
    dot_product = np.dot(ab, bc)
    ab_magnitude = np.linalg.norm(ab)
    bc_magnitude = np.linalg.norm(bc)
    angle_rad = np.arccos(dot_product / (ab_magnitude * bc_magnitude))
    angle_deg = np.degrees(angle_rad)
    return angle_deg

def two_d_angle(classe1, classe2, classe3):
    a = [classe1.x, classe1.y]
    b = [classe2.x, classe2.y]
    c = [classe3.x, classe3.y]
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    ab = b - a
    bc = c - b
    dot_product = np.dot(ab, bc)
    ab_magnitude = np.linalg.norm(ab)
    bc_magnitude = np.linalg.norm(bc)
    angle_rad = np.arccos(dot_product / (ab_magnitude * bc_magnitude))
    angle_deg = np.degrees(angle_rad)
    return angle_deg

if __name__ == '__main__':
    app.run(debug=True)

