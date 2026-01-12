from controller import DistanceSensor, Emitter, Robot, Camera, Liadar, GPS 


# **************** #
# *** CONSTANT *** #
# **************** #
VELOCITY = 6.28
TIMESTEP = 32


# ******************************* #
# *** INITIALIZER - COMPONENT *** #
# ******************************* #
robot = Robot()

rWheel = robot.getDevice("wheel1 motor")
lWheel = robot.getDevice("wheel2 motor")

rWheel.setPosition(float("inf"))
lWheel.setPosition(float("inf"))

rWhell.setVelocity(0)
lWheel.setVelocity(0)

rCamera = robot.getDevice("camera1")
lCamera = robot.getDevice("camera2")

rCamera.enable(TIMESTEP)
lCamera.enable(TIMESTEP)

sColor = robot.getDevice("colour_sensor")
sColor.enable(TIMESTEP)

receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(TIMESTEP)

gps = robot.getDevice("gps")
gps.enable(TIMESTEP)

lidar = robot.getDevice("lidar")
lidar.enable(TIMESTEP)


# **************** #
# *** MOVEMENT *** #
# **************** #

def driveTo(dx: int, sx: int):
        rWheel.setVelocity(dx)
        lWheel.setVelocity(sx)

def stop(ms):
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
    pass

if __name__ == "__main__":
    main()
