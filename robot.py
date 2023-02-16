from sr.robot3 import *

global pins, homeMarkers, allWallMarkers
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
        MoveRobot(0)
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
        print(f"{i.id} is {i.distance/1000} away. It is {i.size/100}m large.")
#SortMarkersByCloseness(markersAndDistance)
def SortMarkersByCloseness(markersAndDistance):
    return sorted(markersAndDistance.values())
#GetMarkerAndDistance()
def GetMarkerAndDistance(inMarkers):
    markersAndDistance = {}
    for i in inMarkers:
        markersAndDistance[i] = i.distance / 1000
    return markersAndDistance
#FilterSeenItemsByID(idFilter, items)
def FilterSeenItemsByID(idFilter, items):
    newItems = []
    for i in items:
        if i.id == idFilter:
            newItems.append(i)
    return newItems
#FilterSeenItemsByIDRange(range, items)
def FilterSeenItemsByIDRange(rangeOfMarkers, items):
    newItems = []
    for i in items:
        if i.id >= rangeOfMarkers[0] and i.id <= rangeOfMarkers[1]:
            newItems.append(i)
    return newItems
#FilterSeenItemsBySize(idFilter, items)
def FilterSeenItemsBySize(sizeFilter, items):# ~ Use size in metres
    newItems = []
    for i in items:
        if i.size / 1000 == sizeFilter:
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
#GetHomeMarkers()
def GetHomeMarkers():
    PivotRobot(0, 0.3, 0, 2)
    MoveRobotInGoodTime(1, -0.5)
    items = []
    while len(items) == 0:
        R.sleep(0.00001)
        items = FilterSeenItemsByIDRange([0, len(allWallMarkers)], GetTheCameraItems())
        RemarkAboutSeenMarkers(items)
    return WorkOutHomeMarkers(items[0])

#WorkOutHomeMarkers(marker)
def WorkOutHomeMarkers(marker):
    maxMarker = marker.id + 1
    minMarker = maxMarker - 7
    if minMarker < 0:
        minMarker = 24
    homeMarkers = []
    if minMarker == 24:
        homeMarkers = [24,25,26,27,0,1,2,3]
    else:
        for i in range(minMarker, maxMarker + 1):
            homeMarkers.append(i)
    print(homeMarkers)
    return homeMarkers

#MarkerWithinHomeMarkers(marker)
def MarkerWithinHomeMarkers(markerId):
    if markerId in homeMarkers: return True
    else: return False

#PinpointSingleMarker():
def PinpointSingleClosestMarker(inMarkers):
    return SortMarkersByCloseness(GetMarkerAndDistance(inMarkers))[0]

#LoadInAllWallMarkers()
def LoadInAllWallMarkers():
    allWallMarkers = []
    for i in range(0, 28):
        allWallMarkers.append(i)
    return allWallMarkers

##Raw code ~ No subs, put them elsewhere
pins = [A0, A1, A2, A3, A4, A5]
#allWallMarkers = LoadInAllWallMarkers()
#homeMarkers = GetHomeMarkers()
#print("TIME TO START")
#MoveAboutTheArena()
print(PinpointSingleClosestMarker(GetTheCameraItems()))

###YOU WERE TRYING TO FIND THE HOME MARKERS###