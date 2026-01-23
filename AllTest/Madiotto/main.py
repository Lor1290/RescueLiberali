from controller import Robot
import numpy as np


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
 
lidar = robot.getDevice("lidar")
lidar.enable(TIMESTEP)

gps = robot.getDevice("gps")
gps.enable(TIMESTEP)

emitter = robot.getDevice("emitter")


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
    lWheel.setVelocity(0)
    delay(ms)

def delay(ms):
    initTime = robot.getTime()
    while robot.step(TIMESTEP) != -1:
        if (robot.getTime() - initTime) * 1000 >= ms:
            break 

def avoidingHole():
    spinOnRight()
    delay(300)
    goForward

def getLidarDistanceFront():
    lidarArray = lidar.getRangeImage()
    avgDistance = 0.0
    rightValue = 0
    # print("-----------------------------------")
    for i in list(range(1023, 1055)) + list(range(1503, 1535)):
        if lidarArray[i] < MAXLIDARDISTANCE:
            avgDistance += lidarArray[i]
            rightValue += 1
            # print(f"i {i-1023}: {round(lidarArray[i],3)} ")
    if rightValue <= 10:
        return 1.0
    avgDistance = round(avgDistance / rightValue, 3)
    # print(f"The average front distance is :  {avgDistance}")
    return avgDistance

def getLidarDistanceRight():
    lidarArray = lidar.getRangeImage()
    avgDistance = 0.0
    rightValue = 0
    # print("-----------------------------------")
    for i in range(1119, 1183):
        if lidarArray[i] < MAXLIDARDISTANCE:
            avgDistance += lidarArray[i]
            rightValue += 1
            # print(f"i {i-1023}: {round(lidarArray[i],3)} ")
    if rightValue <= 10:
        return 1.0
    avgDistance = round(avgDistance / rightValue, 3)
    # print(f"The average right distance is :  {avgDistance}")
    return avgDistance

def getLidarDistanceBack():
    lidarArray = lidar.getRangeImage()
    avgDistance = 0.0
    rightValue = 0
    # print("-----------------------------------")
    for i in range(1247, 1311):
        if lidarArray[i] < MAXLIDARDISTANCE:
            avgDistance += lidarArray[i]
            rightValue += 1
            # print(f"i {i-1023}: {round(lidarArray[i],3)} ")
    if rightValue <= 10:
        return 1.0
    avgDistance = round(avgDistance / rightValue, 3)
    # print(f"The average back distance is :  {avgDistance}")
    return avgDistance

def getLidarDistanceLeft():
    lidarArray = lidar.getRangeImage()
    avgDistance = 0.0
    rightValue = 0
    # print("-----------------------------------")
    for i in range(1375, 1439):
        if lidarArray[i] < MAXLIDARDISTANCE:
            avgDistance += lidarArray[i]
            rightValue += 1
            # print(f"i {i-1023}: {round(lidarArray[i],3)} ")
    if rightValue <= 10:
        return 1.0
    avgDistance = round(avgDistance / rightValue, 3)
    # print(f"The average left distance is :  {avgDistance}")
    return avgDistance

def getLidarDistanceCorner():
    lidarArray = lidar.getRangeImage()
    avgDistance = 0.0
    rightValue = 0
    # print("-----------------------------------")
    for i in range(1439, 1503):
        if lidarArray[i] < MAXLIDARDISTANCE:
            avgDistance += lidarArray[i]
            rightValue += 1
            # print(f"i {i-1023}: {round(lidarArray[i],3)} ")
    if rightValue <= 10:
        return 1.0
    avgDistance = round(avgDistance / rightValue, 3)
    # print(f"The average left distance is :  {avgDistance}")
    return avgDistance
    

# ************ #
# *** MAIN *** #
# ************ #

def main():
    stop(100)
    initPosition = getPosition()
    while robot.step(TIMESTEP) != -1:
        """ 
        TODO: DA MIGLIORARE

        currentOrientation = getPosition()
        if initPosition[0]-2 <= currentOrientation[0] <= initPosition[0]+13 and initPosition[1]-2 <= currentOrientation[1] <= initPosition[1]+13:
            print("REACHED STARTING TILE")
            stop(1000)
        """

        print("-------------------------------------")
        print("MEASUREMENT")
        print(f" - LEFT WALL: {getLidarDistanceLeft()}")
        print(f" - FRONT WALL: {getLidarDistanceFront()}")
        print(f" - LEFT CORNER: {getLidarDistanceCorner()}")
        print(f" - X: {getPosition()[0]}    - Y: {getPosition()[1]}")

        r, g, b = getColour()
        print(f" - R: {r}, - G: {g}, - B: {b}")
        if 50 <= r <= 60 and 50 <= g <= 60 and 50 <= b <= 60:
            print("BLACK HOLE DETECTED")
            avoidingHole()

        # **************** #
        # *** MOVEMENT *** #
        # **************** #

        lWall = getLidarDistanceLeft() < 0.07
        fWall = getLidarDistanceFront() < 0.07
        lCorner = getLidarDistanceCorner() < 0.07

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
