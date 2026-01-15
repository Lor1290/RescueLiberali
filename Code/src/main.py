from controller import DistanceSensor, Emitter, Robot, Camera, Lidar, GPS

import numpy as np
import math

# **************** #
# *** CONSTANT *** #
# **************** #
VELOCITY = 6.28
TIMESTEP = 1

# ******************************* #
# *** INITIALIZER - COMPONENT *** #
# ******************************* #
robot = Robot()

lWheel = robot.getDevice("lWheel motor")
rWheel = robot.getDevice("rWheel motor")

lEncoder = lWheel.getPositionSensor()
rEncoder = rWheel.getPositionSensor()

lEncoder.enable(TIMESTEP)
rEncoder.enable(TIMESTEP)

rWheel.setPosition(float("inf"))
lWheel.setPosition(float("inf"))

rWheel.setVelocity(0)
lWheel.setVelocity(0)

rCamera = robot.getDevice("lCamera")
lCamera = robot.getDevice("rCamera")

rCamera.enable(TIMESTEP)
lCamera.enable(TIMESTEP)

sColor = robot.getDevice("cSensor")
sColor.enable(TIMESTEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIMESTEP)
 
gps = robot.getDevice("gps")
gps.enable(TIMESTEP)

lidar = robot.getDevice("lidar")
lidar.enable(TIMESTEP)

emitter = robot.getDevice("emitter")

# **************** #
# *** MOVEMENT *** #
# **************** #

def goFwd(dx = VELOCITY, sx = VELOCITY):
    rWheel.setVelocity(dx)
    lWheel.setVelocity(sx)

def goLeft(dx = VELOCITY, sx = VELOCITY):
    rWheel.setVelocity(dx)
    lWheel.setVelocity(-sx)

def goRight(dx = VELOCITY, sx = VELOCITY):
    rWheel.setVelocity(-dx)
    lWheel.setVelocity(sx)

def goBack(dx = VELOCITY, sx = VELOCITY):
    rWheel.setVelocity(-dx)
    lWheel.setVelocity(-sx)

def stop(ms = 2000):
    rWheel.setVelocity(0)
    lWheel.setVelocity(0)
    delay(ms)

def delay(ms):
    intial_time = robot.getTime()
    while robot.step(TIMESTEP) != -1:
        if (robot.getTime() - intial_time) * 1000 >= ms:
            break   


# ************** #
# *** GETTER *** #
# ************** #

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

# ************ #
# *** MAIN *** #

# ************ #
def main():
    goLeft()
    while robot.step(TIMESTEP) != -1:
        if lEncoder.getValue() <= -2.20:
            stop()
            break
    
    print(lEncoder.getValue())


if __name__ == "__main__":
    main()
