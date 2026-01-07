from controller import Robot, DistanceSensor, Camera, GPS, Emitter, Lidar

max_velocity = 6.28
timestep = 32

robot = Robot()

"""------ Wheels ------"""
Rwheel = robot.getDevice("wheel1 motor")
Lwheel = robot.getDevice("wheel2 motor")

Rwheel.setPosition(float("inf"))
Lwheel.setPosition(float("inf"))

Rwheel.setVelocity(0)
Lwheel.setVelocity(0)

"""------ Camera ------"""
Rcamera = robot.getDevice("camera1")
Lcamera = robot.getDevice("camera2")

Rcamera.enable(timestep)
Lcamera.enable(timestep)

"""------ Colour Sensor ------"""
Scolor = robot.getDevice("colour_sensor")
Scolor.enable(timestep)

""""------ Emitter & Receiver ------"""
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
receiver.enable(timestep)

"""------ Gps ------"""
gps = robot.getDevice("gps")
gps.enable(timestep)

"""------ Lidar ------"""
lidar = robot.getDevice("lidar")
lidar.enable(timestep)

"""------ Basic functions ------"""
def DriveTo(dx: int, sx: int):
        Rwheel.setVelocity(dx)
        Lwheel.setVelocity(sx)

def Stop(ms):
    Rwheel.setVelocity(0)
    Lwheel.setVelocity(0)
    Delay(ms)

def Delay(ms):
    intial_time = robot.getTime()
    while robot.step(timestep) != -1:
        if (robot.getTime() - intial_time) * 1000 >= ms:
            break

def getColor():
    image = Scolor.getImage()
    r = Scolor.imageGetRed(image, 1, 0, 0)
    g = Scolor.imageGetGreen(image, 1, 0, 0)
    b = Scolor.imageGetBlue(image, 1, 0, 0)
    return r, g, b

def getPosition():
    position = gps.getValues()
    x = position[0] * 100
    y = position[2] * 100
    return x, y

"""------ Main ------"""
def main():
    pass

if __name__ == "__main__":
    main()
