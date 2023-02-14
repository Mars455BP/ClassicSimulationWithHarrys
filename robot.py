from sr.robot3 import *

global pins
R = Robot()

##Moving Robots##
#MoveRobot(direction)
def MoveRobot(direction):
    PowerOneMotor(0, direction)
    PowerOneMotor(1, direction)
#PivotRobot(motorThatPivots, pivotDirection, otherDirection) ~ For pivoting, motor_board[1] with power pivots anti-clockwise and vice versa
def PivotRobot(motorThatPivots, pivotDirection, otherDirection, duration):
    PowerOneMotor(motorThatPivots, pivotDirection)
    if motorThatPivots == 1:
        PowerOneMotor(0, otherDirection)
    else:
        PowerOneMotor(1, otherDirection)
    R.sleep(duration)
    MoveRobot(0)
#PowerOneMotor(motor, direction)
def PowerOneMotor(motor, direction):
    R.motor_board.motors[motor].power = direction
#MoveRobotInGoodTime(movementInSeconds, direction)
def MoveRobotInGoodTime(movementDuration, direction):
    endMove = False
    while endMove == False:
        MoveRobot(direction)
        WatchYourStepNow()
        R.sleep(movementDuration)
        endMove = True
#WatchYourStepNow()
def WatchYourStepNow():
    closnessOfSensors = CheckClosenessOfAllSensors()
    #if closnessOfSensors[pins[0]] <= 2 or closnessOfSensors[pins[1]] <= 2:
    #    MoveRobot(0)
    NoticeUserAboutClosenessToEachPin(closnessOfSensors)

##RUGGEDUINO##
#CheckClosenessOfAllSensors()
def CheckClosenessOfAllSensors():
    pinsAndDistance = {}
    for pin in pins:
        pinsAndDistance[pin] = CheckClosenessViaSensor(pin)
    return SortPinsbyCloseness(pinsAndDistance)
#CheckClosenessViaSensor(pin)
def CheckClosenessViaSensor(pin):
    return R.ruggeduino.pins[pin].analogue_read()
#SortPinsByCloseness(pinsAndDistance) ~ This is a dictionary parameter
def SortPinsbyCloseness(pinsAndDistance):
    return sorted(pinsAndDistance.values())
#NoticeUserAboutClosenessToEachPin(pinsAndDistance) ~ This is a dictionary parameter
def NoticeUserAboutClosenessToEachPin(pinsAndDistance):
    for i in range(len(pins)):
        print(f"Pin {str(pins[i])} closeness: {pinsAndDistance[i]}")

##Moving servos##
#AdjustBothClaws(adjustment)
def AdjustBothClaws(adjustment):
    AdjustClaw(0, adjustment)
    AdjustClaw(1, adjustment)
#AdjustBothClaws(adjustment1, adjustment2)
def AdjustBothClaws(adjustment1, adjustment2):
    AdjustClaw(0, adjustment1)
    AdjustClaw(1, adjustment2)
#AdjustClaw(claw, adjustment)
def AdjustClaw(claw, adjustment):
    R.servo_board.servos[claw].position = adjustment

##Camera Related##
#GetTheCameraItems() ~ Returns the whole marker object
def GetTheCameraItems():
    return R.camera.see()
#GetTheMarkerIDS() ~ Only returns marker ids
def GetTheMarkerIDS():
    return R.camera.see_ids()
#RemarkAboutSeenMarkers(items) ~ This provides interface between the user and the debug console.
def RemarkAboutSeenMarkers(items):
    print(f"I can see {len(items)} markers.")
    for i in items:
        print(f"{i.id} is {i.distance/1000} away. It is {i.size/100}m large")
        print(i.cartesian.x, i.cartesian.y, i.cartesian.z)
        #print(i.pixel_centre)
        #print(i.pixel_corners)
#FilterSeenItemsByID(idFilter, items)
def FilterSeenItemsByID(idFilter, items):
    newItems = []
    for i in items:
        if i.id == idFilter:
            newItems.append(i)
    return newItems
#FilterSeenItemsBySize(idFilter, items)
def FilterSeenItemsBySize(sizeFilter, items):# ~ Use size in metres
    newItems = []
    for i in items:
        if i.size/1000 == sizeFilter:
            newItems.append(i)
    return newItems
#FilterSeenItemsByDistance(distanceFilter, items) ~ IMPORTANT distance is measured in milimeters. To get it in metres, divide by 1000
def FilterSeenItemsByDistance(maxDistance, items):
    newItems = []
    for i in items:
        if i.distance / 1000 <= maxDistance:
            newItems.append(i)
    return newItems

##Competition Commands##
#MoveAboutTheArena()
def MoveAboutTheArena():
    items = []
    foundBox = False
    while foundBox == False:
        if len(items) == 0:
            MoveRobotInGoodTime(0.5, 1)
            UpdateSeenMarkers()
        else:
            print("FOUND!")
        R.sleep(0.000001)
#UpdateSeenMarkers()
def UpdateSeenMarkers():
    items = CompetitionFilter()
    RemarkAboutSeenMarkers(items)
#CompetitionFilter()
def CompetitionFilter():
    items = GetTheCameraItems()
    items = FilterSeenItemsByID(99, items)
    #items = FilterSeenItemsBySize(0.061, items)
    #items = FilterSeenItemsByDistance(5, items)
    return items

##Raw code ~ No subs, put them elsewhere
pins = [A0, A1, A2, A3, A4, A5]
PivotRobot(0, 0.5, 0, 3)
PivotRobot(1, 0.5, 0, 3)
##MoveAboutTheArena()