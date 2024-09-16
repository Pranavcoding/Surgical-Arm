
import cv2
import mediapipe as mp
import numpy as np
from collections import deque


mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

#Pose estimation for video

print("start")

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


#Different Points
#24 = waist
#12 = Shoulder
#14 = elbow
#16 = wrist
#20 = finger


cap = cv2.VideoCapture(0)

times = 0

the_coords = deque(maxlen=2)
the_coords.append([0, 0, 0])
the_coords.append([0, 0, 0])

while cap.isOpened():
    _, frame = cap.read()
    try:

        #resize for portrait video
        #frame = cv2.resize(frame, (350, 600))
        #convert to rgb
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # process frame for pose detection
        
        pose_results = pose.process(frame_rgb)
        
        mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        cv2.imshow('Output', frame)

        waist = pose_results.pose_landmarks.landmark[24]
        shoulder = pose_results.pose_landmarks.landmark[12]
        elbow = pose_results.pose_landmarks.landmark[14]
        wrist = pose_results.pose_landmarks.landmark[16]
        finger = pose_results.pose_landmarks.landmark[20]

        shoulder_angle = 180 - two_d_angle(elbow, shoulder, waist)
        elbow_angle = two_d_angle(shoulder, elbow, wrist)
        wrist_angle = two_d_angle(elbow, wrist, finger)


        ####print("waist is", waist, "   shoulder is", shoulder, "    elbow is", elbow)
        #higher you go, y-value is less
        #righter you go when facing camera, x-value is less
        if times % 100 == 0:

          arduino_shoulder_angle = shoulder_angle
          arduino_elbow_angle = elbow_angle
          arduino_wrist_angle = wrist_angle

          the_coords.append([arduino_shoulder_angle, arduino_elbow_angle, arduino_wrist_angle])
          #SHOULDER, ELBOW, WRIST

          the_change_in_angle = [the_coords[1][0] - the_coords[0][0], the_coords[1][1] - the_coords[0][1], the_coords[1][2] - the_coords[0][2]]

          ##### IN THE_COORDS, THE NEW VALUE IS ON RIGHT AND THE OLD VALUE IS ON LEFT

          print("\n\nTHE ANGLE IS AT SHOULDER:", shoulder_angle) #when hands down by side, it gives low degrees like 20 degrees. When hands up in air gives large value like 170 degrees
          print("THE ANGLE IS AT ELBOW:", elbow_angle)
          print("THE ANGLE IS AT WRIST:", wrist_angle)
          print("THE COORDS:", the_coords[0], the_coords[1])
          print("THE CHANGE IN ANGLE:", the_change_in_angle)
          
          ### the_coords[0] is old one AND the_coords[1] is new one
        






          
        times += 1


        #waist.x or y or z will
          
        
        

        
        
    except:
        pass
        
    if cv2.waitKey(1) == ord('q'):
        break
    
        

cap.release()
cv2.destroyAllWindows

print("done")

