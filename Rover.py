import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't access!!!")
    exit()


port = "/dev/ttyACM0"  
board = pyfirmata.Arduino(port)

servo_pinX = board.get_pin('d:9:s') # Pin 9 for the X-axis servo
servo_pinY = board.get_pin('d:10:s') # Pin 10 for the Y-axis servo

detector = FaceDetector()
servoPos = [90, 90]

motor_front_left_pin1 = board.get_pin('d:2:o')  # Front left wheel motor pin 1
motor_front_left_pin2 = board.get_pin('d:3:o')  # Front left wheel motor pin 2
motor_front_right_pin1 = board.get_pin('d:4:o')  # Front right wheel motor pin 1
motor_front_right_pin2 = board.get_pin('d:5:o')  # Front right wheel motor pin 2
motor_back_left_pin1 = board.get_pin('d:6:o')  # Back left wheel motor pin 1
motor_back_left_pin2 = board.get_pin('d:7:o')  # Back left wheel motor pin 2
motor_back_right_pin1 = board.get_pin('d:8:o')  # Back right wheel motor pin 1
motor_back_right_pin2 = board.get_pin('d:11:o')  # Back right wheel motor pin 2

target_x = ws / 2  
target_y = hs / 2
speed = 100  
threshold_distance = 50  

def move_motors(face_x, face_y):
    if face_x < target_x - threshold_distance:
        # Move right
        motor_front_left_pin1.write(0)  # Stop front left wheel
        motor_front_left_pin2.write(speed)  # Anticlockwise rotation for front left wheel
        motor_front_right_pin1.write(speed)  # Clockwise rotation for front right wheel
        motor_front_right_pin2.write(0)  # Stop front right wheel
        motor_back_left_pin1.write(0)  # Stop back left wheel
        motor_back_left_pin2.write(speed)  # Anticlockwise rotation for back left wheel
        motor_back_right_pin1.write(speed)  # Clockwise rotation for back right wheel
        motor_back_right_pin2.write(0)  # Stop back right wheel
    elif face_x > target_x + threshold_distance:
        # Move left
        motor_front_left_pin1.write(speed)  # Clockwise rotation for front left wheel
        motor_front_left_pin2.write(0)  # Stop front left wheel
        motor_front_right_pin1.write(0)  # Stop front right wheel
        motor_front_right_pin2.write(speed)  # Anticlockwise rotation for front right wheel
        motor_back_left_pin1.write(speed)  # Clockwise rotation for back left wheel
        motor_back_left_pin2.write(0)  # Stop back left wheel
        motor_back_right_pin1.write(0)  # Stop back right wheel
        motor_back_right_pin2.write(speed)  # Anticlockwise rotation for back right wheel
    elif face_y < target_y - threshold_distance:
        # Move forward
        motor_front_left_pin1.write(speed)  # Clockwise rotation for front left wheel
        motor_front_left_pin2.write(0)  # Stop front left wheel
        motor_front_right_pin1.write(0)  # Stop front right wheel
        motor_front_right_pin2.write(speed)  # Clockwise rotation for front right wheel
        motor_back_left_pin1.write(speed)  # Clockwise rotation for back left wheel
        motor_back_left_pin2.write(0)  # Stop back left wheel
        motor_back_right_pin1.write(0)  # Stop back right wheel
        motor_back_right_pin2.write(speed)  # Clockwise rotation for back right wheel
    elif face_y > target_y + threshold_distance:
        # Move backward
        motor_front_left_pin1.write(0)  # Stop front left wheel
        motor_front_left_pin2.write(speed)  # Anticlockwise rotation for front left wheel
        motor_front_right_pin1.write(speed)  # Anticlockwise rotation for front right wheel
        motor_front_right_pin2.write(0)  # Stop front right wheel
        motor_back_left_pin1.write(0)  # Stop back left wheel
        motor_back_left_pin2.write(speed)  # Anticlockwise rotation for back left wheel
        motor_back_right_pin1.write(speed)  # Anticlockwise rotation for back right wheel
        motor_back_right_pin2.write(0)  # Stop back right wheel
    else:
        # Stop if face is centered
        motor_front_left_pin1.write(0)
        motor_front_left_pin2.write(0)
        motor_front_right_pin1.write(0)
        motor_front_right_pin2.write(0)
        motor_back_left_pin1.write(0)
        motor_back_left_pin2.write(0)
        motor_back_right_pin1.write(0)
        motor_back_right_pin2.write(0)

try:
    while True:
        success, img = cap.read()
        img, bboxs = detector.findFaces(img, draw=False)
        if bboxs:
            fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
            servoX = np.interp(fx, [0, ws], [0, 180])
            servoY = np.interp(fy, [0, hs], [0, 180])

            servoX = max(0, min(180, servoX))
            servoY = max(0, min(180, servoY))

            servoPos[0] = servoX
            servoPos[1] = servoY

            cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
            cv2.putText(img, str([fx, fy]), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            move_motors(fx,fy)
        else:
            cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
            motor_front_left_pin1.write(0)
            motor_front_left_pin2.write(0)
            motor_front_right_pin1.write(0)
            motor_front_right_pin2.write(0)
            motor_back_left_pin1.write(0)
            motor_back_left_pin2.write(0)
            motor_back_right_pin1.write(0)
            motor_back_right_pin2.write(0)

        cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

        servo_pinX.write(servoPos[0])
        servo_pinY.write(servoPos[1])

        cv2.imshow("Target Screen", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
finally:
    cap.release()
    cv2.destroyAllWindows()
    board.exit()
