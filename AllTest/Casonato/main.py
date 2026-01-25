from controller import Robot
import numpy as np, time


# **************** #
# *** CONSTANT *** #
# **************** #

robot = Robot()
VELOCITY = 6.28
TIMESTEP = 32
MAXLIDARDISTANCE = 0.14
MAP_SIZE = 200
ORIG = int(MAP_SIZE/2)

last_save = 0
SAVE_INTERVAL = 0.1  # secondi, 0.1 = 10 volte al secondo


grid = [[0 for _ in range(MAP_SIZE)] 
            for _ in range(MAP_SIZE)]

# for i in range(MAP_SIZE):     NON NECESSARIO -> ASSEGNAMENTO INUTILE
#     grid[0][i] = 1            # muro superiore
#     grid[MAP_SIZE-1][i] = 1   # muro inferiore
#     grid[i][0] = 1            # muro sinistro
#     grid[i][MAP_SIZE-1] = 1   # muro destro


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
    goForward()

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
    
def updateMap():
    lWall = getLidarDistanceLeft() < 0.07
    rWall = getLidarDistanceRight() < 0.07
    bWall = getLidarDistanceBack() < 0.07
    fWall = getLidarDistanceFront() < 0.07

    posX = int(getPosition()[1]) 
    posY = int(getPosition()[0])

    global grid, last_save

    grid[ORIG][ORIG] = "ORIGIN"
    print(f" - BOX X: {ORIG + posX}    - BOX Y: {ORIG + posY}")


    # ****************** #
    # *** WALL CHECK *** #
    # ****************** #

    if lWall:
        grid[ORIG+posX-2][ORIG+posY] = 1    # left center
        grid[ORIG+posX-2][ORIG+posY-1] = 1  # left top
        grid[ORIG+posX-2][ORIG+posY+1] = 1  # left bottom
    if rWall:
        grid[ORIG+posX+2][ORIG+posY] = 1    # right center
        grid[ORIG+posX+2][ORIG+posY-1] = 1  # right top
        grid[ORIG+posX+2][ORIG+posY+1] = 1  # right bottom
    if bWall:
        grid[ORIG+posX][ORIG+posY-2] = 1    # back center
        grid[ORIG+posX-1][ORIG+posY-2] = 1  # back left
        grid[ORIG+posX+1][ORIG+posY-2] = 1  # back right
    if fWall:
        grid[ORIG+posX][ORIG+posY+2] = 1    # front center
        grid[ORIG+posX-1][ORIG+posY+2] = 1  # front left
        grid[ORIG+posX+1][ORIG+posY+2] = 1  # front right

    
    # ******************* #
    # *** TILES CHECK *** #
    # ******************* #

    r, g, b = getColour()

    # BLUE
    if 35 <= r <= 84 and 35 <= g <= 84 and 202 <= b <= 255:
        grid[ORIG+posX-1][ORIG+posY-1] = 'b'  # top left
        grid[ORIG+posX+1][ORIG+posY-1] = 'b'  # top right
        grid[ORIG+posX-1][ORIG+posY+1] = 'b' # bottom left
        grid[ORIG+posX+1][ORIG+posY+1] = 'b'  # bottom right

    # PURPLE
    if 179 <= r <= 182 and 82 <= g <= 84 and 246 <= b <= 248:
        grid[ORIG+posX-1][ORIG+posY-1] = 'p'  # top left
        grid[ORIG+posX+1][ORIG+posY-1] = 'p'  # top right
        grid[ORIG+posX-1][ORIG+posY+1] = 'p'  # bottom left
        grid[ORIG+posX+1][ORIG+posY+1] = 'p'  # bottom right
    
    # RED
    if r == 255 and 79 <= g <= 84 and 79 <= b <= 84:
        grid[ORIG+posX-1][ORIG+posY-1] = 'r'  # top left
        grid[ORIG+posX+1][ORIG+posY-1] = 'r'  # top right
        grid[ORIG+posX-1][ORIG+posY+1] = 'r'  # bottom left
        grid[ORIG+posX+1][ORIG+posY+1] = 'r'  # bottom right
    
    # GREEN
    if 18 <= r <= 44 and 184 <= g <= 255 and 18 <= b <= 44:
        grid[ORIG+posX-1][ORIG+posY-1] = 'g'  # top left
        grid[ORIG+posX+1][ORIG+posY-1] = 'g'  # top right
        grid[ORIG+posX-1][ORIG+posY+1] = 'g'  # bottom left
        grid[ORIG+posX+1][ORIG+posY+1] = 'g'  # bottom right
    
    # ORANGE
    if 189 <= r <= 255 and 139 <= g <= 250 and 1 <= b <= 8:
        grid[ORIG+posX-1][ORIG+posY-1] = 'o'  # top left
        grid[ORIG+posX+1][ORIG+posY-1] = 'o'  # top right
        grid[ORIG+posX-1][ORIG+posY+1] = 'o'  # bottom left
        grid[ORIG+posX+1][ORIG+posY+1] = 'o'  # bottom right
    
    # YELLOW
    if 240 <= r <= 255 and 240 <= g <= 255 and 50 <= b <= 84:
        grid[ORIG+posX-1][ORIG+posY-1] = 'y'  # top left
        grid[ORIG+posX+1][ORIG+posY-1] = 'y'  # top right
        grid[ORIG+posX-1][ORIG+posY+1] = 'y'  # bottom left
        grid[ORIG+posX+1][ORIG+posY+1] = 'y'  # bottom right
    
    now = time.time()
    if now - last_save >= SAVE_INTERVAL:
        # scrive tutta la griglia così com'è
        with open("/home/khr0me/Documents/GITHUB/RescueLiberali/AllTest/Casonato/grid.txt", "w") as f:
            for row in grid:
                # ogni cella separata da uno spazio
                f.write(" ".join(str(cell) for cell in row) + "\n")

        last_save = now

    # return grid


# ************ #
# *** MAIN *** #
# ************ #

def main():
    stop(100)
    initPosition = getPosition()

    checkX = getPosition()[0] > 0
    checkY = getPosition()[1] > 0

    if checkX:
        if checkY:
            grid[MAP_SIZE-1][MAP_SIZE-1] = 5
        else:
            grid[MAP_SIZE-1][1] = 5
    else:
        if checkY: 
            grid[1][1] = 5
        else:
            grid[1][MAP_SIZE-1] = 5

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
        print(f" - MAP X: {getPosition()[0]}    - MAP Y: {getPosition()[1]}")
        updateMap()

        r, g, b = getColour()
        print(f" - R: {r}, - G: {g}, - B: {b}")
        # if 10 <= r <= 30 and 10 <= g <= 30 and 10 <= b <= 30:    # MI FUNZIONA SOLO COSì :(     (CIRCA)
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
