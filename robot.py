from sr.robot3 import *

R = Robot()
sleepTime = 0.01
currentX = 0
currentY = 0
targetCoordinatesMatrix = [[-0.8, -1, -1.1, -1.55, -1.815, -2.525, -2.525], [12,15,8,6]]

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

def FindCurrentPosition():
    x = None
    #Probably spin and figure it out

def GoToCoordinate(x, y):
    x = None
    #Go there
    
homeZone = R.zone
homeZone = 1 #Change this (or delete) to test different zones
GetStartPosition(homeZone)
CalculateTargetCoordinates(homeZone)
PrintTargetCoordinates()
