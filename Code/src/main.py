from controller import Robot
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt


robot = Robot()
TIMESTEP = 32


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


# **************** #
# *** CONSTANT *** #
# **************** #

VELOCITY = 6.28
MAXLIDARDISTANCE = 0.14
RESOLUTION = lidar.getHorizontalResolution()
MAX_RANGE = lidar.getMaxRange()
FOV = lidar.getFov()
ROTATION_OFFSET = -math.pi / 2.0
ANGLES = -np.linspace(-FOV / 2.0, FOV / 2.0, RESOLUTION, endpoint=False, dtype=np.float32) + ROTATION_OFFSET
GRID_SIZE = 3.0
global_map_points = set()
plot_counter = 0


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
    x = position[0] * 100
    y = position[2] * 100
    return x, y


def stop(ms=2000):
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
    goForward()


def getLidarDistanceFront(layer_data):
    avgDistance = 0.0
    rightValue = 0
    for i in list(range(480, 512)) + list(range(0, 32)):
        if layer_data[i] < MAXLIDARDISTANCE:
            avgDistance += layer_data[i]
            rightValue += 1
    if rightValue <= 10:
        return 1.0
    return round(avgDistance / rightValue, 3)


def getLidarDistanceRight(layer_data):
    avgDistance = 0.0
    rightValue = 0
    for i in range(96, 160):
        if layer_data[i] < MAXLIDARDISTANCE:
            avgDistance += layer_data[i]
            rightValue += 1
    if rightValue <= 10:
        return 1.0
    return round(avgDistance / rightValue, 3)


def getLidarDistanceBack(layer_data):
    avgDistance = 0.0
    rightValue = 0
    for i in range(224, 288):
        if layer_data[i] < MAXLIDARDISTANCE:
            avgDistance += layer_data[i]
            rightValue += 1
    if rightValue <= 10:
        return 1.0
    return round(avgDistance / rightValue, 3)


def getLidarDistanceLeft(layer_data):
    avgDistance = 0.0
    rightValue = 0
    for i in range(352, 416):
        if layer_data[i] < MAXLIDARDISTANCE:
            avgDistance += layer_data[i]
            rightValue += 1
    if rightValue <= 10:
        return 1.0
    return round(avgDistance / rightValue, 3)


def getLidarDistanceCorner(layer_data):
    avgDistance = 0.0
    rightValue = 0
    for i in range(416, 480):
        if layer_data[i] < MAXLIDARDISTANCE:
            avgDistance += layer_data[i]
            rightValue += 1
    if rightValue <= 10:
        return 1.0
    return round(avgDistance / rightValue, 3)


def buildMap2D(layer_data):
    global global_map_points, plot_counter

    distances_m = np.array(layer_data, dtype=np.float32)
    valid_mask = (distances_m < (MAX_RANGE - 0.05)) & (~np.isinf(distances_m))
    distances_cm = distances_m * 100

    robot_yaw = inertialUnit.getRollPitchYaw()[2]
    absolute_angles = ANGLES + robot_yaw

    x, y = cv2.polarToCart(distances_cm, absolute_angles, angleInDegrees=False)
    x_rel = x.flatten()[valid_mask]
    y_rel = y.flatten()[valid_mask]

    robot_x, robot_y = getPosition()
    global_x = x_rel + robot_x
    global_y = y_rel - robot_y

    for i in range(len(global_x)):
        gx = int(round(global_x[i] / GRID_SIZE) * GRID_SIZE)
        gy = int(round(global_y[i] / GRID_SIZE) * GRID_SIZE)
        global_map_points.add((gx, gy))

    plot_counter += 1
    if plot_counter % 15 == 0:
        if len(global_map_points) > 0:
            map_x, map_y = zip(*global_map_points)
        else:
            map_x, map_y = [], []

        fig, ax = plt.subplots(figsize=(8, 8), facecolor='black')
        ax.set_facecolor('black')

        ax.scatter(map_x, map_y, c='#0055FF', s=150, alpha=0.5)

        ax.scatter(robot_x, -robot_y, c='red', marker='X', s=50)

        ax.set_aspect('equal', 'box')
        ax.axis('off')

        plt.savefig('/Users/simone/Documents/develop/python/rcj_simulation/map_test_2.png',
                    facecolor='black', bbox_inches='tight', pad_inches=0)
        plt.close(fig)


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

        '''print("-------------------------------------")
        print("MEASUREMENT")
        print(f" - LEFT WALL: {getLidarDistanceLeft()}")
        print(f" - FRONT WALL: {getLidarDistanceFront()}")
        print(f" - LEFT CORNER: {getLidarDistanceCorner()}")
        print(f" - X: {getPosition()[0]}    - Y: {getPosition()[1]}")'''

        range_image = lidar.getRangeImage()

        start_layer_2 = RESOLUTION
        end_layer_2 = 2 * RESOLUTION
        layer_2_data = range_image[start_layer_2:end_layer_2]

        buildMap2D(layer_2_data)

        r, g, b = getColour()
        print(f" - R: {r}, - G: {g}, - B: {b}")
        if 50 <= r <= 60 and 50 <= g <= 60 and 50 <= b <= 60:
            print("BLACK HOLE DETECTED")
            avoidingHole()

        # **************** #
        # *** MOVEMENT *** #
        # **************** #

        lWall = getLidarDistanceLeft(layer_2_data) < 0.07
        fWall = getLidarDistanceFront(layer_2_data) < 0.07
        lCorner = getLidarDistanceCorner(layer_2_data) < 0.07

        print("DIRECTION")

        if fWall:
            print(" - Turn right")
            lSpeed = VELOCITY
            rSpeed = -VELOCITY
        else:
            if lWall:
                print(" - Drive forward")
                lSpeed = VELOCITY
                rSpeed = VELOCITY
            else:
                print(" - Turn left")
                lSpeed = VELOCITY / 16
                rSpeed = VELOCITY
            if lCorner:
                print(" - Too close, turn right")
                lSpeed = VELOCITY
                rSpeed = VELOCITY / 16

        print("-------------------------------------", "\n", "\n")

        rWheel.setVelocity(rSpeed)
        lWheel.setVelocity(lSpeed)


if __name__ == "__main__":
    main()
