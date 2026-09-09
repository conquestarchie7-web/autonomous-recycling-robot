# Autonomous Recycling Robot

An autonomous robotic recycling system developed as a university engineering project. The robot combines **embedded C++, sensing, line following, object detection, colour classification, autonomous navigation and servo-actuated manipulation** to collect objects and sort them into the appropriate recycling zones.

## Project overview

The system uses two Arduino-based control boards.

**Primary controller**
- Controls the drive motors
- Follows a floor line using three infrared sensors
- Uses ultrasonic sensors for front obstacle detection and side-object detection
- Controls a servo-actuated gripper
- Reads the ground colour to identify deposit zones
- Communicates robot states and colour information to the secondary board

**Secondary colour/display controller**
- Reads a TCS34725 RGB colour sensor
- Classifies collected objects as red, green or blue
- Displays robot state and colour readings on a 16x2 LCD
- Provides audible feedback for obstacles and task completion
- Communicates the detected object colour back to the primary controller

## System behaviour

The intended operating sequence is:

1. Follow the marked track using the three IR sensors.
2. Detect an object beside the robot using the side ultrasonic sensor.
3. Pivot toward and approach the object.
4. Lower and close the servo gripper.
5. Return to the track.
6. Read the collected object's colour using the secondary board.
7. Continue following the route while checking the ground/deposit-zone colour.
8. When the object's colour matches the destination zone, approach the deposit point.
9. Release the object and return to the track.
10. Repeat until three objects have been deposited.

Front obstacles trigger a safety routine in which the robot stops, waits to see whether the obstacle clears, and otherwise performs an avoidance manoeuvre before reacquiring the line.

## Key engineering features

### Embedded control
The robot is implemented in Arduino C++ with separate control logic for sensing, actuation, communication and navigation.

### Line following
Three analogue IR sensors are converted into digital states and encoded as a three-bit pattern. The pattern is mapped to a steering error that adjusts the relative motor speeds.

### Autonomous object collection
Ultrasonic sensing identifies objects beside the robot. The robot pivots toward a target, approaches it at controlled speed and operates the two-stage gripper.

### Obstacle avoidance
A forward ultrasonic sensor provides short-range safety detection. The robot can stop, wait for an obstruction to clear, or execute a predefined avoidance manoeuvre and search for the line again.

### Colour-based sorting
A TCS34725 RGB sensor is used to classify objects and identify matching coloured deposit zones.

### Inter-board communication
The two controllers exchange status and colour information over serial communication at 57600 baud. The primary controller transmits state messages such as `GRABBING`, `DEPOSITING`, `OBJECT IN FRONT` and `FINISHED`.

## Hardware

The exact hardware configuration used by the project included:

- Arduino-compatible microcontroller boards
- 2 DC drive motors / motor driver interface
- 3 infrared line sensors
- 2 ultrasonic distance sensors
- TCS34725 RGB colour sensor(s)
- 2 servo motors for the gripper mechanism
- 16x2 character LCD
- Buzzer
- Custom mechanical gripper
- Custom wiring and mechanical chassis

## Software dependencies

The sketches use the following Arduino libraries:

- `NewPing`
- `Servo`
- `Wire`
- `Adafruit_TCS34725`
- `LiquidCrystal`
- `SoftwareSerial`

## Repository structure

```text
autonomous-recycling-robot/
├── robot_controller/
│   └── robot_controller.ino
├── colour_sensor_board/
│   └── colour_sensor_board.ino
├── docs/
│   └── system_overview.md
└── README.md
```

## Running the project

The project uses two independent Arduino sketches because the original system used two communicating controller boards.

1. Install the required Arduino libraries.
2. Open `robot_controller/robot_controller.ino` in the Arduino IDE and upload it to the primary controller.
3. Open `colour_sensor_board/colour_sensor_board.ino` and upload it to the secondary controller.
4. Recreate the sensor, motor, servo, LCD and serial wiring described by the pin definitions in the sketches.
5. Tune the IR threshold and colour-classification thresholds for the physical environment before operation.

## Engineering notes

The control thresholds and servo positions are hardware-specific calibration values. They were tuned for the prototype used during the project and may need adjustment on different hardware.

The code intentionally uses simple deterministic state and timing logic rather than a larger robotics framework because the prototype was implemented directly on Arduino-compatible microcontrollers.

## Project context

This was developed as a university robotics engineering project involving integration of mechanical, electronic and software subsystems. The project required the robot to perceive its environment, make navigation decisions and manipulate physical objects reliably.

## Technologies

**C++ · Arduino · Embedded Systems · Sensor Integration · Ultrasonic Sensing · Infrared Sensing · RGB Colour Sensing · Servo Control · Motor Control · Serial Communication · Autonomous Robotics**
