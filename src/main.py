from controller import Robot, DistanceSensor, PositionSensor, Camera, GPS, Emitter, Lidar

timestep = 32
max_velocity = 6.28
robot = Robot()

"""------ Wheels ------"""
wheel_right = robot.getDevice("wheel1 motor")
wheel_left = robot.getDevice("wheel2 motor")
wheel_right.setPosition(float("inf"))
wheel_left.setPosition(float("inf"))
wheel_right.setVelocity(0)
wheel_left.setVelocity(0)

"""------ Camera ------"""
camera_right = robot.getDevice("camera1")
camera_left = robot.getDevice("camera2")
camera_right.enable(timestep)
camera_left.enable(timestep)

"""------ Inertial unit ------"""
inertial_unit = robot.getDevice("inertial_unit")
inertial_unit.enable(timestep)

"""------ Colour Sensor ------"""
colour_sensor = robot.getDevice("colour_sensor")
colour_sensor.enable(timestep)

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

def forward():
    wheel_right.setVelocity(max_velocity)
    wheel_left.setVelocity(max_velocity)

def back():
    wheel_right.setVelocity(-max_velocity)
    wheel_left.setVelocity(-max_velocity)

def turn_right():
    wheel_right.setVelocity(-max_velocity)
    wheel_left.setVelocity(max_velocity)

def turn_left():
    wheel_right.setVelocity(max_velocity)
    wheel_left.setVelocity(-max_velocity)

def stop():
    wheel_right.setVelocity(0)
    wheel_left.setVelocity(0)

def main():
    pass

if __name__ == "__main__":
    main()