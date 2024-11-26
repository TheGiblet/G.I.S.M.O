import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the line following sensors
left_sensor_pin = 9  # Replace with your left sensor pin
right_sensor_pin = 11 # Replace with your right sensor pin

# Set up GPIO pins as input
GPIO.setup(left_sensor_pin, GPIO.IN)
GPIO.setup(right_sensor_pin, GPIO.IN)

try:
    while True:
        # Read sensor states
        left_sensor_state = GPIO.input(left_sensor_pin)
        right_sensor_state = GPIO.input(right_sensor_pin)

        # Print sensor readings
        print("Left sensor:", left_sensor_state)  # 0 = on the line, 1 = off the line
        print("Right sensor:", right_sensor_state) # 0 = on the line, 1 = off the line

        # Edge detection
        if left_sensor_state == 1 and right_sensor_state == 1:
            print("Edge detected!")

        time.sleep(0.2)  # Adjust delay as needed

except KeyboardInterrupt:
    GPIO.cleanup()
