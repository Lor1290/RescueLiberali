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
    # Davanti: da 480 a 511 (sinistra-avanti) e da 0 a 31 (destra-avanti)
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
    # Destra: indici da 96 a 160 (centro a 128)
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
    # Dietro: indici da 224 a 288 (centro a 256)
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
    # Sinistra: indici da 352 a 416 (centro a 384)
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
    # Angolo sinistro: indici da 416 a 480 (centro a 448)
    for i in range(416, 480):
        if layer_data[i] < MAXLIDARDISTANCE:
            avgDistance += layer_data[i]
            rightValue += 1
    if rightValue <= 10:
        return 1.0
    return round(avgDistance / rightValue, 3)


def polarToCartesian(layer_data):
    distances_m = np.array(layer_data, dtype=np.float32)
    valid_mask = (distances_m < (MAX_RANGE - 0.01)) & (~np.isinf(distances_m))

    distances_cm = distances_m * 100

    x, y = cv2.polarToCart(distances_cm, ANGLES, angleInDegrees=False)
    x = x.flatten()
    y = y.flatten()

    x[~valid_mask] = np.nan
    y[~valid_mask] = np.nan
    return x, y


def plotLidar3d(x1, y1, x2, y2):
    z1 = np.full(x1.shape, 6.0)
    z2 = np.full(x2.shape, 5.0)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x1, y1, z1, c='blue', label='Layer 1', s=10)
    ax.scatter(x2, y2, z2, c='cyan', label='Layer 2', s=10)

    ax.scatter(0, 0, 0, c='red', marker='X', s=100, label='Robot')

    ax.set_xlabel('Asse X (cm)')
    ax.set_ylabel('Asse Y (cm)')
    ax.set_zlabel('Altezza Z (cm)')
    ax.set_title('Scansione 3D dei muri di Erebus')

    ax.set_box_aspect([1, 1, 0.5])
    ax.legend()
    plt.savefig('/Users/simone/Documents/develop/python/rcj_simulation/map.png')
    plt.close(fig)


def buildMap2D(layer_data):
    distances_m = np.array(layer_data, dtype=np.float32)
    valid_mask = (distances_m < (MAX_RANGE - 0.01)) & (~np.isinf(distances_m))
    distances_cm = distances_m * 100

    robot_yaw = inertialUnit.getRollPitchYaw()[2]

    absolute_angles = ANGLES + robot_yaw

    x, y = cv2.polarToCart(distances_cm, absolute_angles, angleInDegrees=False)

    x_valid = x.flatten()[valid_mask]
    y_valid = y.flatten()[valid_mask]

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.scatter(x_valid, y_valid, c='blue', s=15, label='Muri (Nord Assoluto)')
    ax.scatter(0, 0, c='red', marker='X', s=100, label='Robot')

    ax.set_aspect('equal', 'box')

    ax.set_xlabel('Est / Ovest (Asse X in cm)')
    ax.set_ylabel('Nord / Sud (Asse Y in cm)')
    ax.set_title('Mappa 2D Globale (Nord in alto)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

    plt.savefig('/Users/simone/Documents/develop/python/rcj_simulation/map_test_2.png')
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
