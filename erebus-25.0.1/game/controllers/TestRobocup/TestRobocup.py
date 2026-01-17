from time import sleep
from PIL import Image
from ultralytics import YOLO
from controller import Robot, DistanceSensor, PositionSensor, Camera, GPS, Emitter, Lidar
import cv2
import numpy as np
import math
import random
import struct

import threading


# variabili generali o globali
timeStep = 32
max_velocity = 5.1
rotation_speed = 5.0
maxLidarDistance = 0.14


# creazione robot
robot = Robot()


# dichiarazione ruota sinistra e ruota destra
wheel_left = robot.getDevice("wheel2 motor")
wheel_right = robot.getDevice("wheel1 motor")


# settare la posizione della ruota sinistra e della ruota destra
wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))
wheel_right.setVelocity(0)
wheel_left.setVelocity(0)


# camera
cameraRight = robot.getDevice("camera1")
cameraLeft = robot.getDevice("camera2")
cameraRight.enable(timeStep)
cameraLeft.enable(timeStep)


# inertial unit
inertialUnit = robot.getDevice("inertial_unit")
inertialUnit.enable(timeStep)


# sensore di colore
colourSensor = robot.getDevice("colour_sensor")
colourSensor.enable(timeStep)


# emitter e receiver
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timeStep)


# GPS
gps = robot.getDevice("gps")
gps.enable(timeStep)


# lidar
lidar = robot.getDevice("lidar")
lidar.enable(timeStep)


# model
#  TF

#  KERAS
# np.set_printoptions(suppress=True)
# model = load_model("/Users/simone/Documents/RoboCup/Erebus-v24_1_0/player_controllers/AI/converted_keras/keras_model.h5",compile=False)
# classNames = open("/Users/simone/Documents/RoboCup/Erebus-v24_1_0/player_controllers/AI/converted_keras/labels.txt","r").readlines()

start = robot.getTime()


def forward():
    wheel_left.setVelocity(max_velocity)
    wheel_right.setVelocity(max_velocity)


def forwardD():
    wheel_left.setVelocity(max_velocity)
    wheel_right.setVelocity(max_velocity)
    position = getGpsValues()
    f_x = abs(position[0])+10
    f_y = abs(position[1])+10

    print(f"{f_x} {f_x}")
    print(f"{abs(getGpsValues()[0])} {abs(getGpsValues()[1])}")
    sleep(10)

    while abs(getGpsValues()[0]) != f_x or abs(getGpsValues()[1]) != f_y:
        wheel_left.setVelocity(max_velocity)
        wheel_right.setVelocity(max_velocity)

        if getColour() == "hole":
            hole()
            break

    stopMotors()


def goBack():
    wheel_left.setVelocity(-max_velocity)
    wheel_right.setVelocity(-max_velocity)


def stopMotors():
    wheel_left.setVelocity(0)
    wheel_right.setVelocity(0)


def spinOnRight():
    wheel_left.setVelocity(rotation_speed)
    wheel_right.setVelocity(-rotation_speed)


def spinOnLeft():
    wheel_left.setVelocity(-rotation_speed)
    wheel_right.setVelocity(rotation_speed)


def getColour():
    image = colourSensor.getImage()
    r = colourSensor.imageGetRed(image, 1, 0, 0)
    g = colourSensor.imageGetGreen(image, 1, 0, 0)
    b = colourSensor.imageGetBlue(image, 1, 0, 0)
    # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
    if 203 <= r <= 233 and 170 <= g <= 200 and 93 <= b <= 123:
        print("Brown")
        return "brown"
    if 0 <= r <= 30 and 0 <= g <= 30 and 0 <= b <= 30:
        print("Black")
        return "black"
    if 225 <= r <= 255 and 50 <= g <= 80 and 50 <= b <= 80:
        print("Red")
        return "red"
    if 20 <= r <= 50 and 225 <= g <= 255 and 20 <= b <= 50:
        print("Green")
        return "green"
    if 50 <= r <= 80 and 50 <= g <= 80 and 225 <= b <= 255:
        print("Blue")
        return "blue"
    if 113 <= r <= 173 and 56 <= g <= 79 and 225 <= b <= 255:
        print("Purple")
        return "purple"
    if 120 <= r <= 180 and 120 <= g <= 180 and 120 <= b <= 180:
        print("Gray")
        return "gray"
    if 29 <= r <= 59 and 33 <= g <= 63 and 46 <= b <= 76:
        print("Checkpoint")
        return "checkpoint"
    if 93 <= r <= 123 and 93 <= g <= 123 and 93 <= b <= 123:
        print("Hole border")
        return "hole"
    if 225 <= r <= 255 and 225 <= g <= 255 and 225 <= b <= 255:
        print("White")
        return "white"
    if 86 <= r <= 116 and 177 <= g <= 127 and 191 <= b <= 221:
        print("Wall")
        return "wall"
    if 42 <= r <= 46 and 42 <= g <= 46 and 42 <= b <= 46:
        print("Wall Black")
        return "wallB"
    if 126 <= r <= 234 and 222 <= g <= 255 and 231 <= b <= 255:
        print("Strange wall")
        return "wallS"


def getLidarDistanceFront():
    lidarArray = lidar.getRangeImage()
    avgDistance = 0.0
    rightValue = 0
    # print("-----------------------------------")
    for i in list(range(1023, 1055)) + list(range(1503, 1535)):
        if lidarArray[i] < maxLidarDistance:
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
        if lidarArray[i] < maxLidarDistance:
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
        if lidarArray[i] < maxLidarDistance:
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
        if lidarArray[i] < maxLidarDistance:
            avgDistance += lidarArray[i]
            rightValue += 1
            # print(f"i {i-1023}: {round(lidarArray[i],3)} ")
    if rightValue <= 10:
        return 1.0
    avgDistance = round(avgDistance / rightValue, 3)
    # print(f"The average left distance is :  {avgDistance}")
    return avgDistance


def turnLeft():
    currentOrientation = inertialUnit.getRollPitchYaw()[2]
    targetOrientation = 0.0
    if -0.1 <= currentOrientation <= 0.1:  # nord
        # print("nord")
        targetOrientation = math.pi / 2
        spinOnLeft()
    elif (math.pi / 2 - 0.1) <= currentOrientation <= (math.pi / 2 + 0.1):  # ovest
        # print("ovest")
        targetOrientation = math.pi
        spinOnLeft()
    elif -math.pi <= currentOrientation <= (-math.pi + 0.1) or (math.pi - 0.1) <= currentOrientation <= math.pi:  # sud
        # print("sud")
        targetOrientation = -(math.pi / 2)
        spinOnLeft()
    elif (-(math.pi / 2) - 0.1) <= currentOrientation <= (-(math.pi / 2) + 0.1):  # est
        # print("est")
        targetOrientation = 0.0
        spinOnLeft()
    while robot.step(timeStep) != -1:
        newCurrentOrientation = inertialUnit.getRollPitchYaw()[2]
        if abs(angleNormalization(newCurrentOrientation - targetOrientation)) < 0.05:
            stopMotors()
            # print("Left turn completed")
            return


def turnRight():
    currentOrientation = inertialUnit.getRollPitchYaw()[2]
    targetOrientation = 0.0
    if -0.1 <= currentOrientation <= 0.1:  # nord
        # print("nord")
        targetOrientation = -(math.pi / 2)
        spinOnRight()
    elif (-(math.pi / 2) - 0.1) <= currentOrientation <= (-(math.pi / 2) + 0.1):  # est
        # print("est")
        targetOrientation = math.pi
        spinOnRight()
    elif -math.pi <= currentOrientation <= (-math.pi + 0.1) or (math.pi - 0.1) <= currentOrientation <= math.pi:  # sud
        # print("sud")
        targetOrientation = math.pi / 2
        spinOnRight()
    elif (math.pi / 2 - 0.1) <= currentOrientation <= (math.pi / 2 + 0.1):  # ovest
        # print("ovest")
        targetOrientation = 0.0
        spinOnRight()
    while robot.step(timeStep) != -1:
        newCurrentOrientation = inertialUnit.getRollPitchYaw()[2]
        if abs(angleNormalization(newCurrentOrientation - targetOrientation)) < 0.05:
            stopMotors()
            # print("Turn turn completed")
            return


def directionCorrection():
    currentOrientation = inertialUnit.getRollPitchYaw()[2]
    if 0.2 <= currentOrientation <= 1.4:
        if currentOrientation <= 0.8:  # nord
            stopMotors()
            targetOrientation = 0.0
            spinOnLeft()
            while robot.step(timeStep) != -1:
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
        else:  # ovest
            stopMotors()
            targetOrientation = 1.6
            spinOnLeft()
            while robot.step(timeStep) != -1:
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
    if 1.8 <= currentOrientation <= 2.9:
        if currentOrientation <= 2.2:  # ovest
            stopMotors()
            targetOrientation = 1.6
            spinOnLeft()
            while robot.step(timeStep) != -1:
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
        else:  # sud
            stopMotors()
            targetOrientation = 3.2
            spinOnLeft()
            counter = 0
            while robot.step(timeStep) != -1:
                counter = counter + 1
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
    if -2.9 <= currentOrientation <= -1.8:
        if currentOrientation <= -2.2:  # sud
            stopMotors()
            targetOrientation = 3.2
            spinOnLeft()
            counter = 0
            while robot.step(timeStep) != -1:
                counter = counter + 1
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
        else:  # est
            stopMotors()
            targetOrientation = -1.6
            spinOnLeft()
            while robot.step(timeStep) != -1:
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
    if -1.4 <= currentOrientation <= -0.2:
        if currentOrientation <= -0.8:  # est
            stopMotors()
            targetOrientation = -1.6
            spinOnLeft()
            while robot.step(timeStep) != -1:
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
        else:  # nord
            stopMotors()
            targetOrientation = 0.0
            spinOnLeft()
            while robot.step(timeStep) != -1:
                currentOrientation2 = inertialUnit.getRollPitchYaw()[2]
                if abs(angleNormalization(currentOrientation2 - targetOrientation)) < 0.05:
                    return
    return


# def directionCorrection():
#     currentOrientation = inertialUnit.getRollPitchYaw()[2]
#     print("Current Orientation " + str(currentOrientation))
#     if 0.15 <= currentOrientation <= math.pi / 4:  # nord-ovest
#         print("nord-ovest")
#         spinOnRight()
#         targetOrientation = 0.0
#     elif math.pi / 4 <= currentOrientation <= math.pi / 2 - 0.15:  # ovest-nord
#         print("ovest-nord")
#         spinOnLeft()
#         targetOrientation = math.pi / 2
#     elif math.pi / 2 + 0.15 <= currentOrientation <= 3 * math.pi / 4:  # ovest-sud
#         print("ovest-nord")
#         spinOnRight()
#         targetOrientation = math.pi / 2
#     elif 3 * math.pi / 4 <= currentOrientation <= math.pi - 0.15:  # sud-ovest
#         print("sud-ovest")
#         spinOnLeft()
#         targetOrientation = math.pi
#     elif -math.pi + 0.15 <= currentOrientation <= -3 * math.pi / 4:  # sud-est
#         print("sud-est")
#         spinOnRight()
#         targetOrientation = -math.pi
#     elif -3 * math.pi / 4 <= currentOrientation <= -math.pi / 2 + 0.15:  # est-usd
#         print("est-sud")
#         spinOnLeft()
#         targetOrientation = -math.pi / 2
#     elif -math.pi / 2 + 0.15 <= currentOrientation <= -math.pi / 4:  # est-nord
#         print("est-nord")
#         spinOnRight()
#         targetOrientation = -math.pi / 2
#     elif -math.pi / 4 <= currentOrientation <= -0.15:  # nord-est
#         print("nord-est")
#         spinOnLeft()
#         targetOrientation = 0.0
#     else:
#         return
#     while robot.step(timeStep) != -1:
#         newCurrentOrientation = inertialUnit.getRollPitchYaw()[2]
#         if abs(angleNormalization(newCurrentOrientation - targetOrientation)) < 0.05:
#             stopMotors()
#             print("direction correction completed")
#             return
#     return


def angleNormalization(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def wallAhead():
    if getLidarDistanceLeft() > getLidarDistanceRight():
        turnLeft()
    elif getLidarDistanceLeft() < getLidarDistanceRight():
        turnRight()
    else:
        if random.randint(1, 100) % 2 == 0:
            turnLeft()
        else:
            turnRight()


def hole():
    goBack()
    while robot.step(timeStep) != -1:
        colour = getColour()
        if colour == "white" or colour == "brown" or colour == "blue" or colour == "green" or colour == "purple" or colour == "gray" or colour == "checkpoint" or colour == "red":
            break
    wallAhead()


def getGpsValues():
    position = gps.getValues()
    x = position[0] * 100
    y = position[2] * 100
    return x, y


def printGpsValues():
    x, y = getGpsValues()
    print("X: " + str(x) + " - Y: " + str(y))


# def exploreNewAreas():
#     imageRight = cameraRight.getImage()
#     imageLeft = cameraLeft.getImage()
#     colourFoundLeft = getEntranceTile(imageLeft)
#     colourFoundRight = getEntranceTile(imageRight)
#
#     if colourFoundRight != "null":
#         print("gay simo")
#
#     if colourFoundLeft != "null":
#         print("gay simo")
#         # adding logic


def turnBack():
    turnLeft()
    turnLeft()


def getImageCamera():
    image1 = cameraRight.getImage()
    image2 = cameraLeft.getImage()
    width = cameraRight.getWidth()
    height = cameraLeft.getHeight()
    image_array1 = np.frombuffer(image1, dtype=np.uint8).reshape((height, width, 4))
    image_array2 = np.frombuffer(image2, dtype=np.uint8).reshape((height, width, 4))
    image_rgb1 = cv2.cvtColor(image_array1, cv2.COLOR_RGBA2RGB)
    image_rgb2 = cv2.cvtColor(image_array2, cv2.COLOR_RGBA2RGB)
    image_resized1 = cv2.resize(image_rgb1, (64, 40))
    image_resized2 = cv2.resize(image_rgb2, (64, 40))
    cv2.imwrite("captured_image_cameraRight.jpg", image_resized1)
    cv2.imwrite("captured_image_cameraLeft.jpg", image_resized2)


def rgbToGray(image):
    return np.dot(image[..., :3], [0.2989, 0.5870, 0.1140])


def get_left():
    imageLeft = cv2.imread("captured_image_cameraLeft.jpg")
    yolo = YOLO(r"C:\Users\mmich\PycharmProjects\PythonProject1\my_model.pt")
    results_left = yolo.track(imageLeft, stream=True)

    for result in results_left:
        if result.boxes is not None:
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            for cls_id in class_ids:
                class_name_left = result.names[cls_id]
                print(f"[+] Detected class: {class_name_left}")
                return class_name_left

def get_right():
    imageRight = cv2.imread("captured_image_cameraRight.jpg")
    yolo = YOLO(r"C:\Users\mmich\PycharmProjects\PythonProject1\my_model.pt")
    results_right = yolo.track(imageRight, stream=True)

    for result in results_right:
        if result.boxes is not None:
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            for cls_id in class_ids:
                class_name_right = result.names[cls_id]
                print(f"[+] Detected class: {class_name_right}")
                return class_name_right


stop = ""


sleep_event = threading.Event()

def predictChar():
    # YOLO

    global stop

    while True:
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        getImageCamera()

        class_name_left = get_left()
        class_name_right = get_right()



        print(class_name_left)
        print(class_name_right)


        if "corrosive_Hazmat" == class_name_left:
            print("C")
            sleep_event.set()
            score('C')
        elif "flammble-gas_Hazmat" == class_name_left:
            print("F")
            sleep_event.set()
            score('F')
        elif "harmed_victims" == class_name_left:
            print("H")
            sleep_event.set()
            score('H')
        elif "organic-peroxide_Hazmat" == class_name_left:
            print("O")
            sleep_event.set()
            score('O')
        elif "poison_Hazmat" == class_name_left:
            print("P")
            sleep_event.set()
            score('P')
        elif "stable_victims" == class_name_left:
            print("S")
            sleep_event.set()
            score('S')
        elif "unharmed_Victims" == class_name_left:
            print("U")
            sleep_event.set()
            score('U')
        else:
            print("fake")
            stop = ""

        if "corrosive_Hazmat" == class_name_right:
            print("C")
            sleep_event.set()
            score('C')
        elif "flammble-gas_Hazmat" == class_name_right:
            print("F")
            sleep_event.set()
            score('F')
        elif "harmed_victims" == class_name_right:
            print("H")
            sleep_event.set()
            score('H')
        elif "organic-peroxide_Hazmat" == class_name_right:
            print("O")
            sleep_event.set()
            score('O')
        elif "poison_Hazmat" == class_name_right:
            print("P")
            sleep_event.set()
            score('P')
        elif "stable_victims" == class_name_right:
            print("S")
            sleep_event.set()
            score('S')
        elif "unharmed_Victims" == class_name_right:
            print("U")
            sleep_event.set()
            score('U')
        else:
            print("fake")
            stop = ""



    # ARRAY
    # imageRight = cameraRight.getImage()
    # imageLeft = cameraLeft.getImage()
    # width = cameraRight.getWidth()
    # height = cameraLeft.getHeight()
    #
    # grayRightImage = rgbToGray(imageRight)
    # grayLeftImage = rgbToGray(imageLeft)
    #
    # imageRight = Image.open(grayRightImage)
    # imageLeft = Image.open(grayLeftImage)
    #
    # numpyArrayRight = np.array(imageRight)
    # numpyArrayLeft = asarray(imageLeft)
    #
    # rowsMediaRight = np.array([0])
    # rowsMediaLeft = np.array([0])
    #
    # for i in height:
    #     mediaRight = 0
    #     for k in width:
    #         mediaRight = mediaRight + numpyArrayRight[i][k]
    #     rowsMediaRight = np.append(rowsMediaRight, mediaRight)
    #     mediaLeft = 0
    #     for k in width:
    #         mediaLeft = mediaLeft + numpyArrayLeft[i][k]
    #     rowsMediaLeft = np.append(rowsMediaLeft, mediaLeft)

    # TF
    # KERAS
    # imageRight = cameraRight.getImage()
    # imageLeft = cameraLeft.getImage()
    # imageRight = np.asarray(imageRight, dtype=np.float32).reshape(1, 64, 40, 3)
    # imageLeft = np.asarray(imageLeft, dtype=np.float32).reshape(1, 64, 64, 3)
    # imageRight = (imageRight / 127.5) - 1
    # imageLeft = (imageLeft / 127.5) -1
    #
    # predictionRight = model.predict(imageRight)
    # indexRight = np.argmax(predictionRight)
    # class_nameRight = classNames[indexRight]
    # confidence_scoreRight = predictionRight[0][indexRight]
    #
    # print("--------Right---------")
    # print("     Class:", class_nameRight[2:], end="")
    # print("     Confidence Score:", str(np.round(confidence_scoreRight * 100))[:-2], "%")
    #
    # predictionLeft = model.predict(imageLeft)
    # indexLeft = np.argmax(predictionLeft)
    # class_nameLeft = classNames[indexLeft]
    # confidence_scoreLeft = predictionLeft[0][indexLeft]
    #
    # print("--------Left---------")
    # print("     Class:", class_nameLeft[2:], end="")
    # print("     Confidence Score:", str(np.round(confidence_scoreLeft * 100))[:-2], "%")
    # if class_nameRight[2:] == "only_wall":
    #     match (class_nameLeft[2:]):
    #         case "corrosive_Hazmat":
    #             sleep(1)
    #             score('C')
    #         case "flammable-gas_Hazmat":
    #             sleep(1)
    #             score('F')
    #         case "harmed_victims":
    #             sleep(1)
    #             score('H')
    #         case "only_wall":
    #             return
    #         case "organic-peroxide_Hazmat":
    #             sleep(1)
    #             score('O')
    #         case "poison_Hazmat":
    #             sleep(1)
    #             score('P')
    #         case "stable_victims":
    #             sleep(1)
    #             score('S')
    #         case "unharmed_Victims":
    #             sleep(1)
    #             score('U')
    # elif class_nameLeft[2:] == "only_wall":
    #     match (class_nameRight[2:]):
    #         case "corrosive_Hazmat":
    #             sleep(1)
    #             score('C')
    #         case "flammable-gas_Hazmat":
    #             sleep(1)
    #             score('F')
    #         case "harmed_victims":
    #             sleep(1)
    #             score('H')
    #         case "only_wall":
    #             return
    #         case "organic-peroxide_Hazmat":
    #             sleep(1)
    #             score('O')
    #         case "poison_Hazmat":
    #             sleep(1)
    #             score('P')
    #         case "stable_victims":
    #             sleep(1)
    #             score('S')
    #         case "unharmed_Victims":
    #             sleep(1)
    #             score('U')


#   --------- indice ---------
#       '0': None/Unknown
#       '1': Walls
#       '2': Holes
#       '3': Swamps
#       '4': Checkpoints
#       '5': Starting tile
#       '6': Connection tile from 1 to 2
#       '7': Connection tile from 1 to 3
#       '8': Connection tile from 2 to 3
#       'H': Harmed victim
#       'S': Stable victim
#       'U': Unharmed victim
#       'F': Flammable Gas
#       'P': Poison
#       'C': Corrosive
#       'O': Organic Peroxide
#       '8': Grazie Poli (Massimo)


def score(ch):
    stopMotors()
    victimType = bytes(ch, "utf-8")
    print("motor stopped")
    position = gps.getValues()  # Get the current gps position of the robot
    x = int(position[0] * 100)  # Get the xy coordinates, multiplying by 100 to convert from meters to cm
    y = int(position[2] * 100)
    message = struct.pack("i i c", x, y, victimType)
    print(f"[+] Position X: {x} Position Y: {y}")
    print(f"[DEBUG] Sending message: {message} (length: {len(message)})")
    #emitter.send(message)
    print("[INFO] Message sent. motor ripartito")


def main():
    global stop

    while robot.step(timeStep) != -1:
        directionCorrection()
        # exploreNewAreas()
        # predictChar()

        if sleep_event.is_set():
            sleep(1)
            sleep_event.clear()

        if stop != "":
            sleep(1)

        if getColour() == "hole":
            hole()
        if getLidarDistanceFront() <= 0.065:
            stopMotors()
            if getLidarDistanceRight() <= 0.08:
                print("spin on left")
                turnLeft()
            elif getLidarDistanceLeft() <= 0.08:
                print("spin on right")
                turnRight()
            else:
                print("Wall ahead")
                wallAhead()
            continue
        forward()

"""
def main2():
    not_repeat = ""
    while robot.step(timeStep) != -1:
        directionCorrection()
        # getImageCamera()
        # predictChar()

        if getLidarDistanceRight() >= 0.08 and not_repeat != "r":
            stopMotors()
            turnRight()
            not_repeat = "r"
        elif getLidarDistanceFront() >= 0.08:
            forwardD()
        elif getLidarDistanceLeft() >= 0.08 and not_repeat != "l":
            stopMotors()
            turnLeft()
            not_repeat = "l"
        elif getLidarDistanceBack() >= 0.08 and not_repeat !    = "b":
            stopMotors()
            turnBack()
            not_repeat = "b"
"""

if __name__ == "__main__":
    Thread_2 = threading.Thread(target=main)
    Thread_1 = threading.Thread(target=predictChar)

    Thread_2.start()
    Thread_1.start()