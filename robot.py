from sr.robot3 import *

R = Robot()

distance = R.ruggeduino.pins[A5].analogue_read()
print(f"Rear ultrasound distance: {distance} meters")
sleepTime = 0.01

def CheckValidTarget(leftWallRotY, rightWallRotY, marker):
    print("x =", marker.spherical.rot_x, " y =", marker.spherical.rot_y)
    if marker.spherical.rot_y > leftWallRotY and marker.spherical.rot_y < rightWallRotY:
        print("Valid")
        return True

def FindTargetMarker(leftTargetID, rightTargetID):
    targetMarker = None
    while targetMarker is None:
        leftWallMarkerID = 27
        leftWallMarkerRotY = 99
        rightWallMarkerID = 0
        rightWallMarkerRotY = -99
        R.sleep(sleepTime)
        R.motor_board.motors[1].power = 0.2
        R.sleep(sleepTime)
        markers = R.camera.see()
        R.motor_board.motors[1].power = 0
        print("I can see ", len(markers), " markers:")
        markers.sort(key=lambda x: x.id, reverse=False)
        for m in markers:
            print(m.id)
            if m.id != 99 and m.id > leftTargetID and m.id < rightTargetID:
                if m.id < leftWallMarkerID:
                    leftWallMarkerID = m.id
                    leftWallMarkerRotY = m.spherical.rot_y
                if m.id > rightWallMarkerID:
                    rightWallMarkerID = m.id
                    rightWallMarkerRotY = m.spherical.rot_y
        print(leftWallMarkerRotY)
        print(rightWallMarkerRotY)
        for m in markers:
            if m.id == 99:
                if CheckValidTarget(leftWallMarkerRotY, rightWallMarkerRotY, m):
                    return m.spherical.rot_y

def MoveToTarget(targetY):
    rotMult = 0.05
    R.motor_board.motors[0].power = 0.5
    R.motor_board.motors[1].power = 0.5
    R.sleep(sleepTime)
    distance = 99999
    while distance / 1000 > 0.3:
        minRot = 0.1
        
        markers = R.camera.see()
        print("I can see", len(markers), "markers:")
        R.sleep(sleepTime)
        markers.sort(key=lambda x: abs(x.spherical.rot_y), reverse=False)
        for m in markers:
            yRot = m.spherical.rot_y
            if m.id == 99 and (abs(yRot - targetY) % (3.1415926 - minRot)) < abs(minRot):
                print("Valid marker at y =", yRot)
                R.motor_board.motors[0].power += (m.distance / 1000) * rotMult * (abs(yRot - targetY) % (3.1415926 - minRot)) * (yRot / abs(yRot))
                R.motor_board.motors[1].power -= (m.distance / 1000) * rotMult * (abs(yRot - targetY) % (3.1415926 - minRot)) * (yRot / abs(yRot))
                print("yRot: {0}  targetY: {1}".format(yRot, targetY))
                distance = m.distance
                break
        print("Target Marker is {0} metres away".format(distance / 1000))
        
def ReturnHome(): #Currently just grabs whatever is infront and reverses
    R.motor_board.motors[0].power = 0
    R.motor_board.motors[1].power = 0
    R.servo_board.servos[0].position = 1
    R.servo_board.servos[1].position = 1
    R.sleep(0.5)
    R.motor_board.motors[0].power = -1
    R.motor_board.motors[1].power = -1
    R.sleep(5)
    R.servo_board.servos[0].position = -1
    R.servo_board.servos[1].position = -1
        
def StealToken():
    targetY = FindTargetMarker(3, 10)
    MoveToTarget(0)
    ReturnHome()
    
markers = R.camera.see()
for m in markers:
    print(m.id)
    print(m.spherical.rot_y)



#Moves the robot forwards a little bit for a better starting position
R.motor_board.motors[0].power = 1
R.motor_board.motors[1].power = 1
R.sleep(0.3)
R.motor_board.motors[0].power = 0
R.motor_board.motors[1].power = 0

StealToken()

R.motor_board.motors[0].power = 0
R.motor_board.motors[1].power = 0

