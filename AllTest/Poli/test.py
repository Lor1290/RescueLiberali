from controller import Robot
import numpy


# **************** #
# *** CONSTANT *** #
# **************** #

robot = Robot()
VELOCITY = 6.28
TIMESTEP = 32

MAXLIDARDISTANCE = 0.14


# **************************** #
# *** INITIALIZER - DRIVER *** #
# **************************** #

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

rWheel = robot.getDevice("wheel1 motor")
lWheel = robot.getDevice("wheel2 motor")
rWheel.setPosition(float("inf"))
lWheel.setPosition(float("inf"))
rWheel.setVelocity(0)
lWheel.setVelocity(0)

rCamera = robot.getDevice("camera1")
lCamera = robot.getDevice("camera2")
rCamera.enable(TIMESTEP)
lCamera.enable(TIMESTEP)

inertialUnit = robot.getDevice("inertial_unit")
inertialUnit.enable(TIMESTEP)

sColour = robot.getDevice("colour_sensor")
sColour.enable(TIMESTEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIMESTEP)

gps = robot.getDevice("gps")
gps.enable(TIMESTEP)

emitter = robot.getDevice("emitter" )


# ***************** #
# *** FUNCTIONS *** #
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
    x = np.round(position[0] * 100)
    y = np.round(position[2] * 100)
    return x, y

def stop(ms = 2000):
    rWheel.setVelocity(0)
    lWhe el.setVelocity(0)
    delay(ms)

def delay(ms):
    initTime = robot.getTime()
    while robot.step(TIMESTEP) != -1:
        if (robot.getTime() - initTime) * 1000 >= ms:
            break 

def getRight():
    bRight = pSensor1.getValues() * 100
    tRight = pSenso2.getValue() * 100

    return bRight, tRight 

def getLeft():
    b

def main():
    while robot.step(TIMESTEP) != -1:

        
