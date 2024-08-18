import cv2
import numpy as np

# Ensure the haarcascade path is correct and file is available on Raspberry Pi
haarcascade_path = '/home/pi/haarcascade_frontalface_default.xml'  # Adjust path as needed
pengklasifikasiWajah = cv2.CascadeClassifier(haarcascade_path)

videoCam = cv2.VideoCapture(0)

if not videoCam.isOpened():
    print("Could not open video camera")
    exit()

tombolQditekan = False
while not tombolQditekan:
    ret, kerangka = videoCam.read()

    if ret:
        abuAbu = cv2.cvtColor(kerangka, cv2.COLOR_BGR2GRAY)
        dafWajah = pengklasifikasiWajah.detectMultiScale(abuAbu, scaleFactor=1.3, minNeighbors=2)

        for (x, y, w, h) in dafWajah:
            cv2.rectangle(kerangka, (x, y), (x + w, y + h), (0, 255, 0), 2)

        teks = "Jumlah Wajah Terdeteksi = " + str(len(dafWajah))
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(kerangka, teks, (0, 30), font, 1, (255, 0, 0), 1)
        cv2.imshow("Hasil", kerangka)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            tombolQditekan = True

videoCam.release()
cv2.destroyAllWindows()
