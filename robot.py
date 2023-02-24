from sr.robot3 import *
from math import cos, sqrt, acos, asin, sin, pi

global pins, homeMarkers, allWallMarkers

R = Robot()

class Vec3:
    # x: float
    # y: float
    # z: float

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def add(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def sub(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def mul(self, other):
        return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)

    def smul(self, other: float):
        return Vec3(self.x * other, self.y * other, self.z * other)

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def length(self):
        return sqrt(self.dot(self))

    def normalise(self):
        return self.smul(1.0 / self.length())

    def cross(self, other):
        # cx = aybz − azby
        # cy = azbx − axbz
        # cz = axby − aybx
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
def Clamp(min, max, t):
    if t < min: return min
    if t > max: return max
    return t
class Vision:
    UP: Vec3 = Vec3(0.0, 0.0, 1.0)
    ORIGIN: Vec3 = Vec3(0.0, 0.0, 0.0)

    # Assumes use of the bpv.maths library
    # marker0/1 must be a Marker object e.g one in the list returned by Robot.camera.see
    # marker_world_positions must be indexable by an int and when indexed return a Vec3
    @staticmethod
    def robot_position(marker0, marker1, marker_world_positions):
        # distances to markers taking into account camera angle
        # marker0_distance = marker0.distance * cos(Vision.camera_angle)
        # marker1_distance = marker1.distance * cos(Vision.camera_angle)
        marker0_distance = marker0.distance
        marker1_distance = marker1.distance

        # marker world positions
        marker0_position = marker_world_positions[marker0.id]
        marker1_position = marker_world_positions[marker1.id]

        # vector between the markers
        inter_marker = marker1_position.sub(marker0_position)
        inter_marker_distance = inter_marker.length()

        # angle between the vectors to t
        theta = acos(
            Clamp(-1, 1,
            ((inter_marker_distance ** 2) - (marker0_distance ** 2) - (marker1_distance ** 2)) 
            / (-2 * marker0_distance * marker1_distance)
            )
        )

        # angle between (vector from marker0 to marker1) and (vector from marker0 to camera)
        alpha = asin((marker1_distance * sin(theta)) / inter_marker_distance)

        epsilon = marker0_distance * cos(alpha)
        psi = marker0_distance * sin(alpha)

        omega = marker0_position.add(inter_marker.smul(1 / inter_marker_distance).smul(epsilon))
        wall_normal = inter_marker.cross(Vision.UP).normalise()

        if wall_normal.dot(Vision.ORIGIN.sub(omega).normalise()) < 0:
            wall_normal = wall_normal.smul(-1.0)

        print("hhh", wall_normal.x, wall_normal.y, wall_normal.z)

        robot_position = omega.add(wall_normal.smul(psi))

        return robot_position

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

#MarkerWithinHomeMarkers(marker)
def MarkerWithinHomeMarkers(markerId):
    if markerId in homeMarkers: return True
    else: return False

#PinpointSingleMarker():
def PinpointSingleClosestMarker(inMarkers):
    return SortMarkersByCloseness(GetMarkerAndDistance(inMarkers))[0]

#LoadInAllWallMarkers()
def LoadInAllWallMarkers():
    allWallMarkers = {
        0: Vec3(2.154, -2.8749,0.175),
        1: Vec3(1.436, -2.8749,0.175),
        2: Vec3(0.718, -2.8749, 0.175),
        3: Vec3(0.0,-2.8749,0.175),
        4: Vec3(-0.718,-2.8749,0.175),
        5: Vec3(-1.436,-2.8749,0.175),
        6: Vec3(-2.154,-2.8749,0.175),
        7: Vec3(-2.8749,-2.154,0.175),
        8: Vec3(-2.8749,-1.436,0.175),
        9: Vec3(-2.8749,-0.718,0.175),
        10: Vec3(-2.8749,0,0.175),
        11: Vec3(-2.8749,0.718,0.175),
        12: Vec3(-2.8749,1.436,0.175),
        13: Vec3(-2.8749,2.154,0.175),
        14: Vec3(-2.154, 2.8749, 0.175),
        15: Vec3(-1.436, 2.8749, 0.175),
        16: Vec3(-0.718, 2.8749, 0.175),
        17: Vec3(0, 2.8749, 0.175),
        18: Vec3(0.718, 2.8749, 0.175),
        19: Vec3(1.436, 2.8749, 0.175),
        20: Vec3(2.154, 2.8749, 0.175),
        21: Vec3(2.8749, 2.154, 0.175),
        22: Vec3(2.8749, 1.436, 0.175),
        23: Vec3(2.8749, 0.718, 0.175),
        24: Vec3(2.8749, 0, 0.175),
        25: Vec3(2.8749, -0.718, 0.175),
        26: Vec3(2.8749, -1.436, 0.175),
        27: Vec3(2.8749, -2.154, 0.175)}
    return allWallMarkers

#GetHomeMarkers()
def GetHomeMarkers(zone : int):
    if zone == 0:
        return [0, 1, 2, 3, 24, 25, 25, 27]
    elif zone == 1:
        return [3, 4, 5, 6, 7, 8, 9, 10]
    elif zone == 2:
        return [10, 11, 12, 13, 14, 15, 16, 17]
    else:
        return [17, 18, 19, 20, 21, 22, 23, 24]

#GetLocationOfRobot()
def GetLocationOfRobot(marker1, marker2):
    return Vision.robot_position(marker1, marker2, allWallMarkers)

def RotateRobotLeft(): #rotates robot left in an anti-clockwise direction
    R.motor_board.motors[0].power = 0
    R.motor_board.motors[1].power = 0.5

##Raw code ~ No subs, put them elsewhere
pins = [A0, A1, A2, A3, A4, A5]
allWallMarkers = LoadInAllWallMarkers()
homeMarkers = GetHomeMarkers(R.zone)
print(allWallMarkers)
print(homeMarkers)
#print("TIME TO START")
#MoveAboutTheArena()
RotateRobotLeft()
R.sleep(0.5)
MoveRobot(0)
thing = GetLocationOfRobot(FilterSeenItemsByIDRange([0, 27], R.camera.see())[0], FilterSeenItemsByIDRange([0, 27], R.camera.see())[1])
print(thing.x, thing.y)

###YOU WERE TRYING TO FIND THE HOME MARKERS###