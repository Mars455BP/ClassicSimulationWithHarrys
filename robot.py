from sr.robot3 import *

R = Robot()
sleepTime = 0.01
currentX = 0
currentY = 0
targetCoordinatesMatrix = [[-0.8, -1, -1.1, -1.55, -1.815, -2.525, -2.525], [12,15,8,6]]

#Starts from wall marker 14
#wallMarkerCoordinatesMatrix
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
27: Vec3(2.8749, -2.154, 0.175)

def GetStartPosition(zone)
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
            
def CalculateTargetCoordinates(zone)
    

def FindCurrentPosition()
    #Probably spin and figure it out

def GoToCoordinate(x, y)
    #Go there
    
homeZone = R.zone
GetStartPosition(homeZone)
CalculateTargetCoordinates(homeZone)