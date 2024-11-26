import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pin for the TTP223B touch sensor
touch_sensor_pin = 22  # Replace with your sensor pin

# Set up GPIO pin as input
GPIO.setup(touch_sensor_pin, GPIO.IN)

try:
    while True:
        # Read sensor state
        touch_sensor_state = GPIO.input(touch_sensor_pin)

        # Print sensor reading
        if touch_sensor_state == GPIO.LOW:  # 0 = touched, 1 = not touched
            print("Touch sensor: Touched")
        else:
            print("Touch sensor: Not touched")

        time.sleep(0.2)  # Adjust delay as needed

except KeyboardInterrupt:
    GPIO.cleanup()
