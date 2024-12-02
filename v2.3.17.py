# This code controls a robot with the following features:
# - Motor control using an L298N motor driver
# - Obstacle avoidance using a Grove Ultrasonic Ranger
# - Edge detection using two IR sensors
# - Rear obstacle detection using an additional IR sensor (active only in reverse)
# - Touch sensor on GPIO 22 to trigger a "shake" behavior (with debouncing)
# - Camera for capturing images of obstacles (using libcamera)
# - Head servo (micro servo 9g MS18) controlled by PCA9685 PWM driver on channel 16
# - Lifting arm with LHS servo on channel 14 and RHS servo on channel 15 of PCA9685
# - 7-color flashing LED connected to GPIO 12 to indicate various states

import RPi.GPIO as GPIO
import time
import subprocess
import datetime
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# --- Motor driver pins ---
in1 = 23
in2 = 24
in3 = 17
in4 = 27
ena = 18  # PWM pin for motor 1
enb = 13  # PWM pin for motor 2

# --- Grove Ultrasonic Ranger pin ---
grove_ultrasonic = 25

# --- IR tracking sensor pins ---
left_ir = 9
right_ir = 11

# --- Rear IR obstacle detection sensor pin ---
rear_ir_out = 4

# --- Touch sensor pin ---
touch_sensor = 22

# --- 7-color flashing LED pin ---
led_pin = 15

# --- PCA9685 setup ---
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 50  # Set PWM frequency for servos

# --- Set up GPIO pins ---
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT) 
GPIO.setup(grove_ultrasonic, GPIO.OUT)  # Ultrasonic sensor trigger pin
GPIO.setup(left_ir, GPIO.IN)  # Left IR sensor pin
GPIO.setup(right_ir, GPIO.IN)  # Right IR sensor pin
GPIO.setup(rear_ir_out, GPIO.IN)  # Rear IR sensor pin
GPIO.setup(touch_sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Touch sensor pin with pull-up
GPIO.setup(led_pin, GPIO.OUT)  # LED pin

# --- Create PWM objects for motor control ---
pwm_a = GPIO.PWM(ena, 1000)
pwm_b = GPIO.PWM(enb, 1000)
pwm_a.start(0)
pwm_b.start(0)

# --- Motor control functions ---
def motor1(speed):
    """Controls motor 1 connected to IN1 and IN2."""
    if speed > 0:  # Forward
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(speed)
    elif speed < 0:  # Backward
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm_a.ChangeDutyCycle(-speed)
    else:  # Stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(0)

def motor2(speed):
    """Controls motor 2 connected to IN3 and IN4."""
    if speed > 0:  # Forward
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
        pwm_b.ChangeDutyCycle(speed)
    elif speed < 0:  # Backward
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)
        pwm_b.ChangeDutyCycle(-speed)
    else:  # Stop
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        pwm_b.ChangeDutyCycle(0)

def stop_motors():
    """Stops both motors."""
    motor1(0)
    motor2(0)

# --- Function to get distance from Grove Ultrasonic Ranger ---
def get_distance():
    """Measures distance using the Grove Ultrasonic Ranger."""
    # Trigger the sensor
    GPIO.output(grove_ultrasonic, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(grove_ultrasonic, GPIO.LOW)

    # Switch pin to input mode for echo
    GPIO.setup(grove_ultrasonic, GPIO.IN)

    StartTime = time.time()
    StopTime = time.time()
    timeout = 0.02  # Timeout for echo reading

    while GPIO.input(grove_ultrasonic) == 0:
        StartTime = time.time()
        if time.time() - StartTime > timeout:
            return -1

    while GPIO.input(grove_ultrasonic) == 1:
        StopTime = time.time()
        if time.time() - StopTime > timeout:
            return -1

    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2  # Speed of sound in cm/s

    # Switch pin back to output mode for next trigger
    GPIO.setup(grove_ultrasonic, GPIO.OUT)
    return distance

# --- Function to take a picture with timestamp ---
def take_picture(filename_prefix):
    """Captures a picture with a timestamped filename."""
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"{filename_prefix}_{timestamp}.jpg"
    try:
        # Use libcamera-still to capture a still image
        subprocess.run(["libcamera-still", "-o", filename], check=True)
        print(f"Picture saved to {filename}")
    except subprocess.CalledProcessError as e:
        print(f"Error taking picture: {e}")

# --- Function for "shake" behavior ---
def shake():
    """Performs a "shake" motion."""
    # play_sound("shake.wav")  # Commented out - No sound
    for _ in range(2):  # Adjust the range for more or fewer shakes
        motor1(50)
        motor2(-50)
        time.sleep(0.2)  # Adjust shaking speed
        motor1(-50)
        motor2(50)
        time.sleep(0.2)
    stop_motors()

# --- Function to set servo angle (adjusted for micro servo) ---
def set_servo_angle(channel, angle):
    """Sets the servo angle on the specified channel."""
    pulse = int(angle / 180 * (2400 - 500) + 500)  # Adjusted pulse width for micro servo
    pca.channels[channel].duty_cycle = pulse

# --- Function to control the lifting arm ---
def lift_arm(angle):
    """
    Controls the lifting arm servos.

    Args:
      angle: The desired angle of the arm (0-180 degrees).
    """
    # Assuming both servos mirror each other for lifting
    set_servo_angle(13, angle)  # LHS servo on channel 14
    set_servo_angle(14, 180 - angle)  # RHS servo on channel 15 (inverted)

# --- Function to flash the LED ---
def flash_led(color, duration):
    """Flashes the LED with the specified color and duration."""
    # Implement your logic here to control the 7-color LED
    # This will depend on how your LED is wired and controlled
    if color == "red":
        GPIO.output(led_pin, GPIO.HIGH)  # Example: Turn on the LED for red
    elif color == "green":
        GPIO.output(led_pin, GPIO.HIGH)  # Example: Turn on the LED for green
    # ... add more color logic if needed ...

    time.sleep(duration)
    GPIO.output(led_pin, GPIO.LOW)  # Turn off the LED

# --- Main loop ---
try:
    robot_direction = "forward"  # Initially moving forward

    # Debouncing for touch sensor
    touch_sensor_state = 0
    last_touch_time = time.time()
    debounce_delay = 0.2  # Debounce time in seconds

    while True:
        # --- Get distance from Grove Ultrasonic Ranger ---
        dist = get_distance()
        print("Measured Distance = %.1f cm" % dist)

        # --- Read IR tracking sensors ---
        left_ir_state = GPIO.input(left_ir)
        right_ir_state = GPIO.input(right_ir)

        # --- Obstacle avoidance ---
        if dist < 20 and dist != -1:
            print("Obstacle detected!")
            # play_sound("obstacle.wav")  # Commented out
            time.sleep(1)  # Add a delay for the sound to play
            stop_motors()
            time.sleep(0.5)
            take_picture("obstacle_detected")  # Take picture before moving

            # --- Scan for alternative route using the head servo ---
            clear_route_found = False
            head_angle = 90  # Start with the head centered

            # Scan from left to right
            for angle in range(45, 135, 10):  # Adjust the range and step as needed
                set_servo_angle(15, angle)  # Set head servo angle (channel 16)
                time.sleep(0.1)  # Allow time for servo to move
                dist = get_distance()
                if dist > 30:  # Check for clear distance
                    clear_route_found = True
                    head_angle = angle
                    break

            if clear_route_found:
                print("Alternative route found!")
                motor1(70)  # Move towards the clear route
                motor2(70)
                set_servo_angle(15, head_angle)  # Keep head pointing towards the clear route
            else:  # No clear route found, reverse and turn
                motor1(-50)  # Move back
                motor2(-50)
                time.sleep(0.5)
                motor1(50)  # Turn right 
                motor2(-50)
                time.sleep(1)

            robot_direction = "forward" 
            
            # --- Lift the arm to collect the obstacle ---
            lift_arm(120)  # Lift the arm (adjust angle as needed)
            time.sleep(2)  # Wait for the arm to lift
            lift_arm(0)  # Lower the arm
            time.sleep(1)

        # --- Edge detection using IR sensors ---
        elif left_ir_state == 1 and right_ir_state == 1:
            print("Edge detected!")
            # play_sound("edge.wav")  # Commented out
            time.sleep(1)  # Add a delay for the sound to play
            stop_motors()
            time.sleep(0.5)
            take_picture("edge_detected")  # Take picture before moving
            motor1(-50)  # Move backward
            motor2(-50)
            time.sleep(1)
            motor1(50)  # Turn 90 degrees
            motor2(-50)
            time.sleep(0.5)
            robot_direction = "forward"

        # --- Rear obstacle detection ---
        elif robot_direction == "backward":
            rear_ir_state = GPIO.input(rear_ir_out)
            if rear_ir_state == 0:
                print("Rear obstacle detected!")
                # play_sound("rear_obstacle.wav")  # Commented out
                time.sleep(1)  # Add a delay for the sound to play
                stop_motors()
                time.sleep(0.5)
                motor1(50)
                motor2(-50)
                time.sleep(0.5)
                robot_direction = "forward"

        # --- No obstacle, edge, or reversing ---
        else:
            print("Moving forward")
            motor1(70)  # Move forward
            motor2(70)
            robot_direction = "forward"

        # --- Touch sensor with debouncing ---
        if GPIO.input(touch_sensor) == 0 and touch_sensor_state == 0:
            if time.time() - last_touch_time > debounce_delay:
                shake()  # Make sure shake() doesn't use play_sound()
                touch_sensor_state = 1
                last_touch_time = time.time()
        elif GPIO.input(touch_sensor) == 1:
            touch_sensor_state = 0

        # --- Flash the LED based on different conditions ---
        if dist < 20:
            flash_led("red", 0.5)  # Flash red for obstacle detected
        elif left_ir_state == 1 and right_ir_state == 1:
            flash_led("blue", 0.2)  # Flash blue for edge detected
        elif robot_direction == "backward":
            flash_led("yellow", 0.2)  # Flash yellow for reversing
        else:
            flash_led("green", 0.1)  # Flash green for normal operation

        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
