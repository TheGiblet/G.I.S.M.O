import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the HC-SR04
trig_pin = 17  # Replace with your trigger pin
echo_pin = 4   # Replace with your echo pin

# Set up GPIO pins
GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

try:
    while True:
        # Send trigger pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)  # 10 microseconds pulse
        GPIO.output(trig_pin, False)

        # Measure echo pulse
        StartTime = time.time()
        StopTime = time.time()

        while GPIO.input(echo_pin) == 0:
            StartTime = time.time()

        while GPIO.input(echo_pin) == 1:
            StopTime = time.time()

        # Calculate distance
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2  # Speed of sound in cm/s

        print("Distance: %.1f cm" % distance)
        time.sleep(0.5)  # Adjust delay as needed

except KeyboardInterrupt:
    GPIO.cleanup()
