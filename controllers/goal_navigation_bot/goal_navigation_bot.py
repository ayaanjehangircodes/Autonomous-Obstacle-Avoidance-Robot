from controller import Robot
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

ps = []
for i in range(8):
    s = robot.getDevice('ps' + str(i))
    s.enable(timestep)
    ps.append(s)

MAX_SPEED = 6.28

FAST = 5.2
MED = 4.0
SLOW = 2.0
TURN = 4.5
REVERSE = -3.0

THRESHOLD_FRONT = 85
THRESHOLD_SIDE = 70
THRESHOLD_GOAL = 260

mode = "SEARCH"
turn_timer = 0
reverse_timer = 0
wander_bias = 0
stuck_counter = 0
last_front = 0

while robot.step(timestep) != -1:

    vals = [x.getValue() for x in ps]

    front = max(vals[7], vals[0])
    front_left = max(vals[6], vals[7])
    front_right = max(vals[0], vals[1])

    left_side = max(vals[5], vals[6])
    right_side = max(vals[1], vals[2])

    rear = max(vals[3], vals[4])

    leftSpeed = MED
    rightSpeed = FAST

    # Goal touch / stop
    if front > THRESHOLD_GOAL:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        break

    # Recovery reverse mode
    if reverse_timer > 0:
        leftSpeed = REVERSE
        rightSpeed = REVERSE
        reverse_timer -= 1

    # Timed turn mode
    elif turn_timer > 0:
        if mode == "TURN_LEFT":
            leftSpeed = -TURN
            rightSpeed = TURN
        else:
            leftSpeed = TURN
            rightSpeed = -TURN
        turn_timer -= 1

    else:

        # Hard obstacle directly ahead
        if front > THRESHOLD_FRONT:

            if left_side < right_side:
                mode = "TURN_LEFT"
            elif right_side < left_side:
                mode = "TURN_RIGHT"
            else:
                mode = random.choice(["TURN_LEFT", "TURN_RIGHT"])

            turn_timer = 12
            stuck_counter += 1

        # Narrow passage keep centered
        elif left_side > THRESHOLD_SIDE and right_side > THRESHOLD_SIDE:

            if left_side > right_side:
                leftSpeed = FAST
                rightSpeed = SLOW
            else:
                leftSpeed = SLOW
                rightSpeed = FAST

            stuck_counter = 0

        # Wall / maze on left
        elif left_side > THRESHOLD_SIDE:
            leftSpeed = FAST
            rightSpeed = MED

            stuck_counter = 0

        # Wall / maze on right
        elif right_side > THRESHOLD_SIDE:
            leftSpeed = MED
            rightSpeed = FAST

            stuck_counter = 0

        # Open path: move toward goal zone bias
        else:
            wander_bias += 1

            if wander_bias < 80:
                leftSpeed = MED
                rightSpeed = FAST
            elif wander_bias < 160:
                leftSpeed = FAST
                rightSpeed = MED
            else:
                wander_bias = 0
                leftSpeed = MED
                rightSpeed = FAST

            stuck_counter = 0

    # Detect no progress while obstacle ahead
    if abs(front - last_front) < 2 and front > 90:
        stuck_counter += 1
    else:
        if stuck_counter > 0:
            stuck_counter -= 1

    # Escape trap
    if stuck_counter > 18:
        reverse_timer = 10
        turn_timer = 14
        mode = random.choice(["TURN_LEFT", "TURN_RIGHT"])
        stuck_counter = 0

    last_front = front

    leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, leftSpeed))
    rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, rightSpeed))

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)