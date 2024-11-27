# This code controls a robot with the following features:
# - Motor control using an L298N motor driver
# - Obstacle avoidance using a Grove Ultrasonic Ranger
# - Edge detection and line following using two IR tracking sensors
# - Rear obstacle detection using an additional IR sensor (active only in reverse)
# - I2S audio output using a Daokai MAX98357 I2S Class D amplifier and speaker
# - Touch sensor on GPIO 22 to trigger a "shake" behavior (with debouncing)

import RPi.GPIO as GPIO
import time
import subprocess  # For interacting with the command line

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# --- Motor driver pins ---
in1 = 23
in2 = 24
in3 = 17
in4 = 27
ena = 18  # Example PWM pin for motor 1
enb = 13  # Example PWM pin for motor 2

# --- Grove Ultrasonic Ranger pin ---
grove_ultrasonic = 25

# --- IR tracking sensor pins ---
left_ir = 9
right_ir = 11

# --- Rear IR obstacle detection sensor pin ---
rear_ir_out = 4  # Changed to GPIO 4
# rear_ir_en = 14  # If your sensor has an enable pin, uncomment this

# --- Touch sensor variables for debouncing ---
touch_pin = 22
last_touch_time = 0
debounce_delay = 0.2  # Debounce time in seconds

# --- Set up GPIO pins ---
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT) 
GPIO.setup(grove_ultrasonic, GPIO.OUT)  # Initially set as output for trigger
GPIO.setup(left_ir, GPIO.IN)  # Left IR sensor pin as input
GPIO.setup(right_ir, GPIO.IN) # Right IR sensor pin as input
GPIO.setup(rear_ir_out, GPIO.IN)  # Rear IR sensor OUT pin as input
GPIO.setup(touch_pin, GPIO.IN)  # Touch sensor pin as input
# GPIO.setup(rear_ir_en, GPIO.OUT)  # Uncomment if your rear sensor has an enable pin

# --- Create PWM objects ---
pwm_a = GPIO.PWM(ena, 1000) 
pwm_b = GPIO.PWM(enb, 1000)
pwm_a.start(0)
pwm_b.start(0)

# --- Motor control functions ---
def motor1_forward(speed):
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)

def motor1_backward(speed):
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)

def motor2_forward(speed):
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    pwm_b.ChangeDutyCycle(speed)

def motor2_backward(speed):
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(speed)

def stop_motors():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

# --- Function to get distance from Grove Ultrasonic Ranger ---
def get_distance():
    # Trigger the sensor
    GPIO.output(grove_ultrasonic, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(grove_ultrasonic, GPIO.LOW)

    # Switch pin to input mode for echo
    GPIO.setup(grove_ultrasonic, GPIO.IN)

    StartTime = time.time()
    StopTime = time.time()

    while GPIO.input(grove_ultrasonic) == 0:
        StartTime = time.time()

    while GPIO.input(grove_ultrasonic) == 1:
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2  # Speed of sound in cm/s

    # Switch pin back to output mode for next trigger
    GPIO.setup(grove_ultrasonic, GPIO.OUT)
    return distance

# --- Function to play a sound file ---
def play_sound(file_path):
    subprocess.Popen(["aplay", file_path])  # Use aplay to play the sound

# --- Function to make the robot shake side to side ---
def shake_side_to_side(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        motor1_forward(50)
        motor2_backward(50)
        time.sleep(0.1)  # Adjust shaking speed
        motor1_backward(50)
        motor2_forward(50)
        time.sleep(0.1)

try:
    while True:
        # --- Get distance from Grove Ultrasonic Ranger ---
        dist = get_distance()
        print("Measured Distance = %.1f cm" % dist)

        # --- Read IR tracking sensors ---
        left_ir_state = GPIO.input(left_ir)
        right_ir_state = GPIO.input(right_ir)

        # --- Edge detection using IR sensors ---
        if left_ir_state == 1 and right_ir_state == 1:  # Both sensors off the line (edge detected)
            print("Edge detected!")
            play_sound("edge.wav")  # Play a sound for edge detection
            stop_motors()  # Stop immediately
            time.sleep(0.5)  # Briefly pause
            motor1_backward(50)  # Move backward
            motor2_backward(50)
            time.sleep(1)  # Move backward for a short duration
            motor1_forward(50)  # Turn 90 degrees (adjust timing as needed)
            motor2_backward(50) 
            time.sleep(0.5)  # Adjust turning time
            
        # --- Obstacle avoidance (only if not at an edge) ---
        elif dist < 20:
            print("Obstacle detected!")
            play_sound("obstacle.wav")  # Play a sound for obstacle detection
            stop_motors()  # Stop initially
            time.sleep(0.5)  # Briefly pause
            motor1_backward(50)  # Move back
            motor2_backward(50)
            time.sleep(0.5)  # Move back for a short duration
            motor1_forward(50)  # Turn right to avoid obstacle
            motor2_backward(50)
            time.sleep(1)  # Turn for a short duration

        # --- Rear obstacle detection while reversing ---
        if GPIO.input(in1) == GPIO.LOW and GPIO.input(in2) == GPIO.HIGH and \
           GPIO.input(in3) == GPIO.LOW and GPIO.input(in4) == GPIO.HIGH:  # Check if motors are in reverse
            rear_ir_state = GPIO.input(rear_ir_out)  # Read from GPIO 4
            if rear_ir_state == 0:  # Obstacle detected at the rear
                print("Rear obstacle detected!")
                play_sound("rear_obstacle.wav")  # Play a sound for rear obstacle detection
                stop_motors()
                time.sleep(0.5)
                motor1_forward(50)  # Turn and go forward (adjust as needed)
                motor2_backward(50)
                time.sleep(0.5)  # Adjust turning time

        # --- Touch sensor check with debouncing ---
        if GPIO.input(touch_pin) == GPIO.HIGH and time.time() - last_touch_time > debounce_delay:
            print("Touch detected!")
            shake_side_to_side(2)  # Shake for 2 seconds (adjust as needed)
            last_touch_time = time.time()  # Update the last touch time

        # --- Line following logic (if line detected, else keep moving) ---
        if left_ir_state == 0 or right_ir_state == 0:  # At least one sensor on the line
            if left_ir_state == 0 and right_ir_state == 1:  # Left sensor on line
                print("Left sensor on line")
                motor1_forward(40)  # Turn left
                motor2_forward(70)
            elif left_ir_state == 1 and right_ir_state == 0:  # Right sensor on line
                print("Right sensor on line")
                motor1_forward(70)  # Turn right
                motor2_forward(40)
            elif left_ir_state == 0 and right_ir_state == 0:  # Both sensors on line
                print("Both sensors on line")
                motor1_forward(70)  # Move forward
                motor2_forward(70)
        else:  # No sensor on the line, keep moving forward
            print("No line detected, moving forward")
            motor1_forward(70)  # Move forward
            motor2_forward(70)

        time.sleep(0.1)  # Adjust delay as needed

except KeyboardInterrupt:
    GPIO.cleanup()
