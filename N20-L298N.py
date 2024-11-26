import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# L298N motor driver pins
in1 = 23
in2 = 24
in3 = 17
in4 = 27
ena = 18
enb = 13

# Set up GPIO pins as output
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT) 1 

# Set initial speed (0-100)
speed = 50

# Function to drive motors
def drive_motors(in1_state, in2_state, in3_state, in4_state):
    GPIO.output(in1, in1_state)
    GPIO.output(in2, in2_state)
    GPIO.output(in3, in3_state)
    GPIO.output(in4, in4_state)

    # Optional: Use PWM for speed control
    pwm_a = GPIO.PWM(ena, 1000)  # 1kHz frequency
    pwm_b = GPIO.PWM(enb, 1000)
    pwm_a.start(speed)
    pwm_b.start(speed)

try:
    while True:
        # Move forward
        drive_motors(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
        time.sleep(2)

        # Move backward
        drive_motors(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
        time.sleep(2)

        # Turn left
        drive_motors(GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW)
        time.sleep(2)

        # Turn right
        drive_motors(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)
        time.sleep(2)

except KeyboardInterrupt:
    # Stop motors and clean up GPIO pins on exit
    stop_motors()
    GPIO.cleanup()
