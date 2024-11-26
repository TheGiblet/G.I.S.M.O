import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pin for the KY-032 obstacle avoidance sensor
obstacle_sensor_pin = 4  # Replace with your sensor pin

# Set up GPIO pin as input
GPIO.setup(obstacle_sensor_pin, GPIO.IN)

try:
    while True:
        # Read sensor state
        obstacle_sensor_state = GPIO.input(obstacle_sensor_pin)

        # Print sensor reading
        print("Obstacle sensor:", obstacle_sensor_state)  # 0 = obstacle detected, 1 = no obstacle

        time.sleep(0.2)  # Adjust delay as needed

except KeyboardInterrupt:
    GPIO.cleanup()
