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

