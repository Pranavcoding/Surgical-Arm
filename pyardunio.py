import pyfirmata
from pyfirmata2 import Arduino, util

# Define the pins connected to the L293D motor driver
enable_pin = 3  # PWM pin for motor speed control
input1_pin = 4   # Input 1 of the motor driver
input2_pin = 5   # Input 2 of the motor driver

# Connect to the Arduino
board = Arduino('COMX')  # Replace 'COMX' with your Arduino's serial port

# Start an iterator thread so that serial buffer doesn't overflow
it = util.Iterator(board)
it.start()

# Set the pins as output
board.digital[enable_pin].mode = pyfirmata.PWM
board.digital[input1_pin].mode = pyfirmata.OUTPUT
board.digital[input2_pin].mode = pyfirmata.OUTPUT

def motor_control(speed):
    # Control the direction of the motor based on the speed
    if speed > 0:
        board.digital[input1_pin].write(1)
        board.digital[input2_pin].write(0)
    elif speed < 0:
        board.digital[input1_pin].write(0)
        board.digital[input2_pin].write(1)
    else:
        board.digital[input1_pin].write(0)
        board.digital[input2_pin].write(0)

    # Set the motor speed
    speed = abs(speed)  # Ensure speed is positive
    board.digital[enable_pin].write(speed)

# Example: Rotate the motor forward at half speed for 2 seconds
motor_control(127)  # 50% duty cycle for PWM
time.sleep(2)

# Stop the motor
motor_control(0)

# Disconnect from the Arduino
board.exit()
