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
    def CalculateRobotPosition(marker0, marker1, marker_world_positions):
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


sleepTime = 0.01
currentX = 0
currentY = 0
targetCoordinatesMatrix = [[-0.8, -1, -1.1, -1.55, -1.815, -2.325, -2.525, -2.525], [-2.225, -1, -2.525, -1.55, -2.325, -1.815, -2.525, -1.1]]
rotationMatrix = [[0, 1], [-1, 0]]
allWallMarkers = None

#LoadInAllWallMarkers()
def LoadInAllWallMarkers():
    global allWallMarkers
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
        R.sleep(sleepTime)
        R.motor_board.motors[1].power = 0.3
        R.sleep(sleepTime)
        markers = R.camera.see()
        R.motor_board.motors[1].power = 0
        for m in markers:
            if m.id != 99:
                wallCount += 1
        print("Wall marker count = ", wallCount)
        if wallCount >= 2:
            return markers


LoadInAllWallMarkers()
homeZone = R.zone
#homeZone = 1 #Change this (or delete) to test different zones
GetStartPosition(homeZone)
CalculateTargetCoordinates(homeZone)
PrintTargetCoordinates()
markers = RotateUntilWallSeen()
position = Vision.CalculateRobotPosition(FilterSeenItemsByIDRange([0, 27], markers)[0], FilterSeenItemsByIDRange([0, 27], markers)[1], allWallMarkers)
print("x = ", position.x, "   y = ", position.y, "   z = ", position.z)
