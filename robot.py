import time
from sr.robot3 import *
from math import cos, sqrt, acos, asin, sin, pi

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
    def CalculateRobotPosition(marker0, marker_world_positions):
        marker0_distance = marker0.distance / 1000
        
        print("using marker", marker0.id, "distance:", marker0_distance)
        theta = marker0.spherical.rot_y
        
        if marker0.id >= 0 and marker0.id < 7:
            x = marker_world_positions[marker0.id].x - marker0_distance * sin(theta)
            y = marker_world_positions[marker0.id].y + marker0_distance * cos(theta)
        if marker0.id >= 7 and marker0.id < 14:
            x = marker_world_positions[marker0.id].x + marker0_distance * cos(theta)
            y = marker_world_positions[marker0.id].y - marker0_distance * sin(theta)
        if marker0.id >= 14 and marker0.id < 21:
            x = marker_world_positions[marker0.id].x + marker0_distance * sin(theta)
            y = marker_world_positions[marker0.id].y - marker0_distance * cos(theta)
        if marker0.id >= 21:
            x = marker_world_positions[marker0.id].x + marker0_distance * cos(theta)
            y = marker_world_positions[marker0.id].y - marker0_distance * sin(theta)
        
        
        robot_position = Vec3(x, y, 0)
        return robot_position
    

sleepTime = 0.01
currentX = 0
currentY = 0
targetCoordinatesMatrix = [[-0.8, -1, -1.1, -1.55, -1.815, -2.325, -2.525, -2.525], [-2.225, -1, -2.525, -1.55, -2.325, -1.815, -2.525, -1.1]]
rotationMatrix = [[0, 1], [-1, 0]]
allWallMarkers = None

#LoadInAllWallMarkers()
def LoadInAllWallMarkers():
    global allWallMarkers
    allWallMarkers = [
        Vec3(2.154, -2.8749,0.0),
        Vec3(1.436, -2.8749,0.0),
        Vec3(0.718, -2.8749, 0.0),
        Vec3(0.0,-2.8749,0.0),
        Vec3(-0.718,-2.8749,0.0),
        Vec3(-1.436,-2.8749,0.0),
        Vec3(-2.154,-2.8749,0.0),
        Vec3(-2.8749,-2.154,0.0),
        Vec3(-2.8749,-1.436,0.0),
        Vec3(-2.8749,-0.718,0.0),
        Vec3(-2.8749,0,0.0),
        Vec3(-2.8749,0.718,0.0),
        Vec3(-2.8749,1.436,0.0),
        Vec3(-2.8749,2.154,0.0),
        Vec3(-2.154, 2.8749, 0.0),
        Vec3(-1.436, 2.8749, 0.0),
        Vec3(-0.718, 2.8749, 0.0),
        Vec3(0, 2.8749, 0.0),
        Vec3(0.718, 2.8749, 0.0),
        Vec3(1.436, 2.8749, 0.0),
        Vec3(2.154, 2.8749, 0.0),
        Vec3(2.8749, 2.154, 0.0),
        Vec3(2.8749, 1.436, 0.0),
        Vec3(2.8749, 0.718, 0.0),
        Vec3(2.8749, 0, 0.0),
        Vec3(2.8749, -0.718, 0.0),
        Vec3(2.8749, -1.436, 0.0),
        Vec3(2.8749, -2.154, 0.0)]

def PrintTargetCoordinates():
    global targetCoordinatesMatrix
    i = 0
    while i < 8:
        print("x = ", targetCoordinatesMatrix[0][i], "   y = ", targetCoordinatesMatrix[1][i])
        i += 1

def MatrixMultiply(A, B):
    i = 0
    result = [ [0]*8 for i in range(2)]
    while i < 8:
        result[0][i] = A[0][0] * B[0][i] + A[0][1] * B[1][i]
        result[1][i] = A[1][0] * B[0][i] + A[1][1] * B[1][i]
        i += 1
    return result

def GetStartPosition(zone):  
    match zone:
        case 0:
            currentX = 0.5
            currentY = -2.375
        case 1:
            currentX = -2.375
            currentY = -0.5
        case 2:
            currentX = -0.5
            currentY = 2.375
        case 3:
            currentX = 2.375
            currentY = 0.5
            
def CalculateTargetCoordinates(zone):
    global targetCoordinatesMatrix
    rotateCount = zone
    result = targetCoordinatesMatrix
    while rotateCount > 0:
        result = MatrixMultiply(rotationMatrix, result)
        rotateCount -= 1
    targetCoordinatesMatrix = result

def GoToCoordinate(x, y):
    x = None
    #Go there

#FilterSeenItemsByIDRange(range, items)
def FilterSeenItemsByIDRange(rangeOfMarkers, items):
    newItems = []
    for i in items:
        if i.id >= rangeOfMarkers[0] and i.id <= rangeOfMarkers[1]:
            newItems.append(i)
    return newItems

def RotateUntilWallSeen():
    while True:
        wallCount = 0
        time.sleep(sleepTime)
        R.motor_board.motors[1].power = 0.3
        time.sleep(sleepTime)
        markers = R.camera.see()
        R.motor_board.motors[1].power = 0
        for m in markers:
            if m.id != 73:
                wallCount += 1
        print("Wall marker count = ", wallCount)
        if wallCount >= 1:
            return markers


LoadInAllWallMarkers()
homeZone = R.zone
#homeZone = 1 #Change this (or delete) to test different zones
GetStartPosition(homeZone)
CalculateTargetCoordinates(homeZone)
PrintTargetCoordinates()
markers = RotateUntilWallSeen()

position = Vision.CalculateRobotPosition(FilterSeenItemsByIDRange([0, 27], markers)[0], allWallMarkers)
print("x = ", position.x, "   y = ", position.y)
R.motor_board.motors[0].power = 0.3
R.motor_board.motors[1].power = 0.3
time.sleep(2)
R.motor_board.motors[0].power = 0
R.motor_board.motors[1].power = 0
time.sleep(0.2)
markers = RotateUntilWallSeen()
R.motor_board.motors[0].power = 0
R.motor_board.motors[1].power = 0
position = Vision.CalculateRobotPosition(FilterSeenItemsByIDRange([0, 27], markers)[0], allWallMarkers)
print("x = ", position.x, "   y = ", position.y)
