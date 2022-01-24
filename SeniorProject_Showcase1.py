# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Gets the Propellers
frontLeftMotor = robot.getDevice('front left propeller')
frontRightMotor = robot.getDevice('front right propeller')
backLeftMotor = robot.getDevice('rear left propeller')
backRightMotor = robot.getDevice('rear right propeller')

#Sets the position of the propellers
frontLeftMotor.setPosition(float('inf'))
frontRightMotor.setPosition(float('inf'))
backLeftMotor.setPosition(float('inf'))
backRightMotor.setPosition(float('inf'))

motors=[frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor]
	
#Sets the velocity of the propellers
motors[0].setVelocity(1)
motors[1].setVelocity(1)
motors[2].setVelocity(1)
motors[3].setVelocity(1)

# enable camera 
camera = robot.getDevice('camera')
camera.enable(timestep)

# enable gps
gps = robot.getDevice('gps')
gps.enable(timestep)

#enable compass
compass = robot.getDevice('compass')
compass.enable(timestep)

# enable gyro
gyro = robot.getDevice('gyro')
gyro.enable(timestep)

#different camera motors for movement
camera_roll_motor=robot.getDevice('camera roll')
camera_pitch_motor=robot.getDevice('camera pitch')
camera_yaw_motor=robot.getDevice('camera yaw')

#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

#thrust needed for droneto lift
vertical_thrust=68.5

#where robot targets to stabilize
vertical_offset = .6

verticalPID=3
rollPID=50
pitchPID=30
#def set_Values(motors):

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   val=imu.getRollPitchYaw()[2]
   print(val)
            
   pass
# Enter here exit cleanup code.
