def RotateRobot():
    R.motor_board.motors[0].power = 0 
    R.motor_board.motors[1].power = 1 
    R.sleep(0.53)
    Move(0)

    #Code above rotates robot 180 degrees
    #
