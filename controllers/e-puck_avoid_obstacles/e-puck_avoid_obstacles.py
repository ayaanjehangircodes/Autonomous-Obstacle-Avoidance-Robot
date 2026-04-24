from controller import Robot
import random

# Create robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Enable all 8 proximity sensors
ps = []
for i in range(8):
    sensor = robot.getDevice('ps' + str(i))
    sensor.enable(timestep)
    ps.append(sensor)

# Speed settings
MAX_SPEED = 6.28
FORWARD = 5.8
TURN = 5.0
SIDE_SPEED = 3.5
THRESHOLD = 80

# Stuck detection
last_turn = 1
stuck_counter = 0

while robot.step(timestep) != -1:

    # Read sensors
    values = [sensor.getValue() for sensor in ps]

    # Sensor groups
    front = values[7] + values[0]
    left_side = values[5] + values[6] + values[7]
    right_side = values[0] + values[1] + values[2]

    # Default move forward
    leftSpeed = FORWARD
    rightSpeed = FORWARD

    # Front obstacle detected
    if front > THRESHOLD:

        # Choose clearer side
        if left_side < right_side:
            leftSpeed = -TURN
            rightSpeed = TURN
            last_turn = -1
        elif right_side < left_side:
            leftSpeed = TURN
            rightSpeed = -TURN
            last_turn = 1
        else:
            # Equal distance -> random turn
            if random.random() < 0.5:
                leftSpeed = -TURN
                rightSpeed = TURN
                last_turn = -1
            else:
                leftSpeed = TURN
                rightSpeed = -TURN
                last_turn = 1

        stuck_counter += 1

    # Left obstacle only
    elif left_side > THRESHOLD:
        leftSpeed = FORWARD
        rightSpeed = SIDE_SPEED
        stuck_counter = 0

    # Right obstacle only
    elif right_side > THRESHOLD:
        leftSpeed = SIDE_SPEED
        rightSpeed = FORWARD
        stuck_counter = 0

    # Clear path
    else:
        leftSpeed = FORWARD
        rightSpeed = FORWARD
        stuck_counter = 0

    # Anti-stuck mode
    if stuck_counter > 20:
        leftSpeed = -TURN * last_turn
        rightSpeed = TURN * last_turn
        stuck_counter = 0

    # Safety speed limit
    leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, leftSpeed))
    rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, rightSpeed))

    # Apply speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)