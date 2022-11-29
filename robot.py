from sr.robot3 import *
R = Robot()

def Move(direction):
    R.motor_board.motors[0].power = direction
    R.motor_board.motors[1].power = direction

#Reminder - make this intituive to the programs needs, eg. not hardcoded
def RotateRobot():
    R.motor_board.motors[1].power = -0.25
    R.motor_board.motors[0].power = -0.75

def AdjustBothClaws(adjustment):
    AdjustClaw(0, adjustment)
    AdjustClaw(1, adjustment)

def AdjustClaw(claw, adjustment):
    R.servo_board.servos[claw].position = adjustment

def MoveAwayFromObject(pin):
    if R.ruggeduino.pins[A5].analogue_read() < 2:
        Move(1)

def MoveTowardObject(pin):
    if R.ruggeduino.pins[pin].analogue_read() < 2:
        Move(1)

def SpotMarker():
    return R.camera.see()
    print(camera.see.Marker)

def NoticeUserAboutMarkers(currentMarkers):
    print("I can see", len(currentMarkers), "markers:")

    for m in currentMarkers:
        print(" - Marker #{0} is {1} metres away".format(m.id, m.distance / 1000))

def CheckMarkerCloseness(m):
    if m.distance / 1000 < 2:
        Move(1)

markers = SpotMarker()
NoticeUserAboutMarkers(markers)
CheckMarkerCloseness(markers[0])


R.sleep(0.50)
MoveSpeed=0.5

#R.sleep(0.6/abs(MoveSpeed))
#Reminder - move speed is how fast the robot moves. negative is backwards.
#Reminder- set time divided by move speed allows for same distance