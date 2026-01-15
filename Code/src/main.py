from controller import Robot
import numpy as np


# **************** #
# *** CONSTANT *** #
# **************** #

robot = Robot()
VELOCITY = 6.28
TIMESTEP = int(robot.getBasicTimeStep())


# ******************************* #
# *** INITIALIZER - COMPONENT *** #
# ******************************* #

rWheel = robot.getDevice("wheel1 motor")
lWheel = robot.getDevice("wheel2 motor")

rWheel.setPosition(float("inf"))
lWheel.setPosition(float("inf"))

rWheel.setVelocity(0)
lWheel.setVelocity(0)

inertialUnit = robot.getDevice("inertial_unit")
inertialUnit.enable(TIMESTEP)

rCamera = robot.getDevice("camera1")
lCamera = robot.getDevice("camera2")

rCamera.enable(TIMESTEP)
lCamera.enable(TIMESTEP)

sColor = robot.getDevice("colour_sensor")
sColor.enable(TIMESTEP)

emitter = robot.getDevice("emitter")

receiver = robot.getDevice("receiver")
receiver.enable(TIMESTEP)
 
gps = robot.getDevice("gps")
gps.enable(TIMESTEP)

lidar = robot.getDevice("lidar")
lidar.enable(TIMESTEP)

pSensor1 = robot.getDevice("distance sensor1")
pSensor2 = robot.getDevice("distance sensor2")
pSensor3 = robot.getDevice("distance sensor3")
pSensor4 = robot.getDevice("distance sensor4")
pSensor5 = robot.getDevice("distance sensor5")
pSensor6 = robot.getDevice("distance sensor6")
pSensor7 = robot.getDevice("distance sensor7")
pSensor8 = robot.getDevice("distance sensor8")

pSensor1.enable(TIMESTEP)
pSensor2.enable(TIMESTEP)
pSensor3.enable(TIMESTEP)
pSensor4.enable(TIMESTEP)
pSensor5.enable(TIMESTEP)
pSensor6.enable(TIMESTEP)
pSensor7.enable(TIMESTEP)
pSensor8.enable(TIMESTEP)


# ***************** #
# *** Functions *** #
# ***************** #

def getColor():
    image = sColor.getImage()
    r = sColor.imageGetRed(image, 1, 0, 0)
    g = sColor.imageGetGreen(image, 1, 0, 0)
    b = sColor.imageGetBlue(image, 1, 0, 0)
    return r, g, b

def getPosition():
    position = gps.getValues()
    x = position[0] * 100
    y = position[2] * 100
    return x, y

def stop(ms = 2000):
    rWheel.setVelocity(0)
    lWheel.setVelocity(0)
    delay(ms)

def delay(ms):
    init_time = robot.getTime()
    while robot.step(TIMESTEP) != -1:
        if (robot.getTime() - init_time) * 1000 >= ms:
            break 


# ************ #
# *** MAIN *** #
# ************ #

def main():
    while robot.step(TIMESTEP) != -1:
        # **************** #
        # *** MOVEMENT *** #
        # **************** #

        print("-------------------------------------")
        print("MEASUREMENT")
        print(f" - LEFT WALL: {pSensor7.getValue()*100}")
        print(f" - FRONT WALL: {pSensor1.getValue()*100}")
        print(f" - LEFT CORNER: {pSensor8.getValue()*100}")

        lWall = pSensor7.getValue()*100 < 10
        fWall = pSensor1.getValue()*100 < 10
        lCorner = pSensor8.getValue()*100 < 10

        lSpeed = VELOCITY
        rSpeed = VELOCITY

        print("DIRECTION")
        if fWall:
            print(" - Turn right")
            lSpeed = VELOCITY
            rSpeed = -VELOCITY
        else:
            if lWall:
                print(" - Drive forward")
                lWall = VELOCITY
                rSpeed = VELOCITY
            else:
                print(" - Turn left")
                lSpeed = VELOCITY/16
                rSpeed = VELOCITY
            if lCorner:
                print(" - Too close, turn right")
                lSpeed = VELOCITY
                rSpeed = VELOCITY/16

        print("-------------------------------------", "\n", "\n")

        rWheel.setVelocity(rSpeed)
        lWheel.setVelocity(lSpeed)

if __name__ == "__main__":
    main()
