from pyfirmata import Arduino, util
import time

# Initialize Arduino board
board = Arduino('/dev/ttyACM0')

# Define pin numbers
IN1 = 2  # Direction control pin 1 for Motor 1
IN2 = 3  # Direction control pin 2 for Motor 1

# Start Firmata
iterator = util.Iterator(board)
iterator.start()

# Set pin modes
board.digital[IN1].mode = board.OUTPUT
board.digital[IN2].mode = board.OUTPUT

# Function to control the motor
def control_motor(speed, direction):
    if speed < 0:
        speed = -speed  # Ensure speed is positive
        board.digital[IN1].write(0)  # Set direction 1 to LOW
        board.digital[IN2].write(1)  # Set direction 2 to HIGH
    elif speed > 0:
        board.digital[IN1].write(1)  # Set direction 1 to HIGH
        board.digital[IN2].write(0)  # Set direction 2 to LOW
    else:
        board.digital[IN1].write(0)  # Stop motor if speed is 0
        board.digital[IN2].write(0)

    # Set motor speed using analog output
    analog_speed = int((speed / 255) * 1023)  # Convert speed to analog value (0-1023)
    board.analog[0].write(analog_speed)  # Set A0 (PWM) pin with analog value

try:
    while True:
        # Example: Rotate motor forward at full speed for 2 seconds
        control_motor(255, 1)
        time.sleep(2)

        # Stop motor for 1 second
        control_motor(0, 0)
        time.sleep(1)

        # Rotate motor backward at half speed for 3 seconds
        control_motor(-128, -1)
        time.sleep(3)

        # Stop motor for 1 second
        control_motor(0, 0)
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up on exit
    board.exit()