# Autonomous-Car
Code to run the autonomous NATCAR components, including the linescan camera, servo, and motor. All components run using power from a battery connected to a self-designed PCB using Eagle.

# LineScan-Camera
The linescan camera scans for the white tape, assisting in the movement of the car. Algorithm is implemented using ping-pong buffers and can be run with the voltage threshold / slope threshold algorithms.

# Servo
The servo takes input from the linescan camera and turns accordingly to control the direction of where the car moves. Implemented with PWM duty cycles determined by the system clock to determine angle of the servo.

# Motor
The motor is also implemented with PWM duty cycles determined by the system clock to determine the speed the car moves. Different inputs taken from any bluetooth connectable device can alter the PWM to control speed of the car.
