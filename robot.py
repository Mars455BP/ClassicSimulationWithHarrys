def MoveToTarget(targetMarker):
    Move(0.5)
    Move(0.5)
    distance = targetMarker.distance
    while distance / 1000 > 0.2:
        minRot = 0.02
        markers = R.camera.see()
        for m in markers:
            zRot = m.orientation.rot_z
            if m.id == 99 and abs(zRot) < abs(minRot) + 0.02:
                print("valid marker at z = {}".format(zRot))
                if zRot < 0 and R.motor_board.motors[0].power > -0.95:
                    R.motor_board.motors[0].power += zRot
                    R.motor_board.motors[1].power -= zRot
                elif zRot > 0 and R.motor_board.motors[0].power < 0.95:
                    R.motor_board.motors[0].power += zRot
                    R.motor_board.motors[1].power -= zRot
                distance = m.distance
                minRot = zRot
                R.sleep(0.01)
                print(" - Target Marker is {0} metres away with rotation x = {1}, z = {2}".format(distance / 1000, minRot, zRot))
