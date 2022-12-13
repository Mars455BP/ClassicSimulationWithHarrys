from sr.robot3 import *
R = Robot()

def Move(direction):
    R.motor_board.motors[0].power = direction
    R.motor_board.motors[1].power = direction

#Reminder - make this intituive to the programs needs, eg. not hardcoded
def RotateRobot():
    R.motor_board.motors[0].power = 0.75
    R.motor_board.motors[1].power = 0.25

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

def SpotMarkers():
    return R.camera.see()

def NoticeUserAboutMarkers(currentMarkers, homeMarkers):
    print("I can view", len(currentMarkers), "markers:")

    for m in currentMarkers:
        print(" - Marker #{0} is {1} metres away".format(m.id, m.distance / 1000))
	CheckMarkerWithinOurArea(m, homeMarkers)

def CheckMarkerWithinOurArea(marker, homeMarkers):
    if(marker.id <= homeMarkers[0]) or (marker.id >= homeMarkers[1]):
        print("This is within our box")
    else:
        print("This is not within our area")

def CollectWallMarkers(currentMarkers):
    wallMarkers = []
    for m in currentMarkers:
        if(m != 99):
            wallMarkers.append(m)
    return wallMarkers

def CheckMarkerCloseness(m):
    if m.distance / 1000 < 2:
        Move(1)

RotateRobot()
R.sleep(0.6)
Move(0)
Move(-1)
R.sleep(1)
Move(0)

homeMarkers = [3, 24]
NoticeUserAboutMarkers(SpotMarkers(), homeMarkers)
wallMarkers = CollectWallMarkers(SpotMarkers())
NoticeUserAboutMarkers(wallMarkers)



R.sleep(0.50)
MoveSpeed=0.5

#R.sleep(0.6/abs(MoveSpeed))
#Reminder - move speed is how fast the robot moves. negative is backwards.
#Reminder- set time divided by move speed allows for same distance
