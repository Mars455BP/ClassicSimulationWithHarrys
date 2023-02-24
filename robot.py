from sr.robot3 import *

R = Robot()

def Move(direction): #moves the robot forwards and backwards
    R.motor_board.motors[0].power = direction
    R.motor_board.motors[1].power = direction


def RotateRobotLeft(): #rotates robot left in an anti-clockwise direction
    R.motor_board.motors[0].power = 0
    R.motor_board.motors[1].power = 0.5
    
def RotateRobotRight(): #rotates robot right in a clockwise direction
    R.motor_board.motors[0].power = 0.5
    R.motor_board.motors[1].power = 0

def RobotUTurn(): #roates robot 180 degrees
    R.motor_board.motors[0].power = 0
    R.motor_board.motors[1].power = 1
    R.sleep(0.53)
    Move(0)

def AdjustBothClaws(adjustment):
    AdjustClaw(0, adjustment)
    AdjustClaw(1, adjustment)

def AdjustClaw(claw, adjustment):
    R.servo_board.servos[claw].position = adjustment

def AdjustBothFingers(adjustment):
    AdjustFinger(2, adjustment)
    AdjustFinger(3, adjustment)

def AdjustFinger(finger, adjustment):
    R.servo_board.servos[finger].position = adjustment


RotateRobotLeft()
R.sleep(0.49)
Move(0)
Move(1)
R.sleep(0.7)
Move(0)
AdjustBothClaws(1)

#AdjustBothFingers(1)


