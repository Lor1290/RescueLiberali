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

sColour = robot.getDevice("colour_sensor")
sColour.enable(TIMESTEP)

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

def goForward():
    rWheel.setVelocity(VELOCITY)
    lWheel.setVelocity(VELOCITY)

def goBack():
    rWheel.setVelocity(-VELOCITY)
    lWheel.setVelocity(-VELOCITY)

def spinOnRight():
    rWheel.setVelocity(-VELOCITY)
    lWheel.setVelocity(VELOCITY)

def getColour():
    image = sColour.getImage()
    r = sColour.imageGetRed(image, 1, 0, 0)
    g = sColour.imageGetGreen(image, 1, 0, 0)
    b = sColour.imageGetBlue(image, 1, 0, 0)
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
    initTime = robot.getTime()
    while robot.step(TIMESTEP) != -1:
        if (robot.getTime() - initTime) * 1000 >= ms:
            break 

def avoidingHole():
    spinOnRight()
    delay(500)
    goForward
    

# ************ #
# *** MAIN *** #
# ************ #

def main():
    stop(100)
    initPosition = getPosition()
    while robot.step(TIMESTEP) != -1:
        """currentOrientation = getPosition()
        if initPosition[0]-2 <= currentOrientation[0] <= initPosition[0]+13 and initPosition[1]-2 <= currentOrientation[1] <= initPosition[1]+13:
            print("REACHED STARTING TILE")
            stop(1000)
            TODO: DA MIGLIORARE"""

        print("-------------------------------------")
        print("MEASUREMENT")
        print(f" - LEFT WALL: {pSensor7.getValue()*100}")
        print(f" - FRONT WALL: {pSensor1.getValue()*100}")
        print(f" - LEFT CORNER: {pSensor8.getValue()*100}")

        r, g, b = getColour()
        print(f" - R: {r}, - G: {g}, - B: {b}")
        if 50 <= r <= 60 and 50 <= g <= 60 and 50 <= b <= 60:
            print("BLACK HOLE DETECTED")
            avoidingHole()

        # **************** #
        # *** MOVEMENT *** #
        # **************** #

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
