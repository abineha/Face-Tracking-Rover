import pyfirmata
import time

port = 'COM5'
board = pyfirmata.Arduino(port)
it = pyfirmata.util.Iterator(board)
it.start()

servo_pin = board.get_pin('d:9:s')

# Explicitly set pin mode for servo
board.digital[9].mode = pyfirmata.SERVO

def move_servo(angle):
    servo_pin.write(angle)
    print(f"Moving servo to {angle} degrees")
    time.sleep(1)

try:
    move_servo(90)  # Start at middle position
    while True:
        move_servo(0)
        move_servo(180)
        move_servo(90)
except KeyboardInterrupt:
    print("Program exited by user")
finally:
    board.exit()
