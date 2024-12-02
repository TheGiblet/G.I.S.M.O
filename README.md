# G.I.S.M.O. - General-purpose Investigative System for Mobile Omnidirectional Operations

This project is an exploration into robotics and automation using a Raspberry Pi Zero 2 W as the main controller. G.I.S.M.O. is a versatile robot that combines various sensors, actuators, and control mechanisms to perform a range of tasks and experiments.

## Features

* Motor control: Two N20 motors driven by an L298N motor driver enable omnidirectional movement.
* Obstacle avoidance: A Grove Ultrasonic Ranger detects obstacles, allowing the robot to navigate around them.
* Line following and edge detection: Two IR tracking sensors enable the robot to follow lines and detect edges.
* Rear obstacle detection: An additional IR sensor prevents the robot from backing into obstacles.
* Touch interaction: A touch sensor triggers a "shake" behavior when activated.
* Camera: A Raspberry Pi Camera Module captures images and video of the robot's surroundings.

## Hardware Components

* Raspberry Pi Zero 2 W
* L298N Motor Driver
* N20 Motors (x2)
* 3.7V Rechargeable Battery
* USB Charging Port
* Grove Ultrasonic Ranger
* IR Tracking Sensors (x2)
* Rear IR Obstacle Detection Sensor
* TTP223B Capacitive Touch Sensor
* Camera Module

## Software

* Raspberry Pi OS Lite (64-bit)
* Python 3
* RPi.GPIO library: For controlling the GPIO pins on the Raspberry Pi.
* picamera library: For interacting with the Raspberry Pi Camera Module.

## Wiring

**Raspberry Pi GPIO Pin Connections**

| Component                    | Pin  | Description                                    |
| ---------------------------- | ---- | ---------------------------------------------- |
| L298N IN1                    | 23   | Motor 1 direction control                      |
| L298N IN2                    | 24   | Motor 1 direction control                      |
| L298N IN3                    | 17   | Motor 2 direction control                      |
| L298N IN4                    | 27   | Motor 2 direction control                      |
| L298N ENA                    | 18   | Motor 1 speed control (PWM)                   |
| L298N ENB                    | 13   | Motor 2 speed control (PWM)                   |
| Grove Ultrasonic Ranger      | 25   | Trigger/Echo pin for distance measurement     |
| Left IR Tracking Sensor      | 9    | Detects line/edge                             |
| Right IR Tracking Sensor     | 11   | Detects line/edge                             |
| Rear IR Obstacle Sensor      | 4    | Detects obstacles at the back                 |
| Touch Sensor                | 22   | Triggers "shake" behavior                     |
| Camera Module               | CSI  | Dedicated camera port                          |

**Additional connections:**

*   Connect the L298N motor driver to the N20 motors and an external power source for the motors.
*   Connect the sensors and the camera module to the appropriate power and ground pins on the Raspberry Pi.
*   
## Code

The main code for the robot is in `v2.2.7.py` (this is a temporary name). It includes the following functionalities:

* Motor control functions (forward, backward, turn, stop)
* Sensor readings (ultrasonic, IR, touch)
* Obstacle avoidance logic
* Line following and edge detection
* Rear obstacle detection
* Touch interaction (shake behavior)
* Camera control (image capture)

##Update: Camera Functionality and Optimized Navigation

After much effort and numerous iterations using libcamera and libcamera-still, I finally got the camera working on my Raspberry Pi. Unfortunately, I encountered performance issues, including significant lag. As a result, I decided to revert to capturing timestamped images of obstacles and storing them. The robot now takes a picture, processes it, and then looks for a clear route to continue its journey. This solution is more efficient given the limited resources of the Raspberry Pi and ensures smooth operation.

## Future Plans

* Integrate audio feedback.
* Integrate the micro:bit for additional sensing, input, and communication.
* Implement more advanced robot behaviors using the camera (e.g., object recognition, navigation).
* Explore the use of the PCA9685 to control servos or other PWM devices.
* Add battery monitoring and charging status indicators.

## Contributing

Contributions are welcome! Feel free to submit bug reports, feature requests, or pull requests.

## License

MIT License
