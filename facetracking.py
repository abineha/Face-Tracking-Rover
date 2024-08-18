import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't access!!!")
    exit()

# Update the port for Raspberry Pi
port = "/dev/ttyACM0"
board = pyfirmata.Arduino(port)

servo_pinX = board.get_pin('d:9:s') # Pin 9 for the X-axis servo
servo_pinY = board.get_pin('d:10:s') # Pin 10 for the Y-axis servo

detector = FaceDetector()
servoPos = [90, 90] # Initial servo position

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
    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)

    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])

    cv2.imshow("Target Screen", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
board.exit()
