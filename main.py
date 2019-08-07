#!/usr/bin/env pybricks-micropython


from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)

leftSensor = ColorSensor(Port.S1)
rightSensor = ColorSensor(Port.S4)
touchSensor = TouchSensor)Port.S2)
gyroSensor = GyroSensor(Port.S3)

stopWatch = stopWatch()

mainDriveBase = DriveBase(leftMotor, rightMotor)


def gyro_turn_left(time, target, kp):
    """
    Turns the robot left for a certain time and degrees
    :param time: The time in (ms) the program should stop. type: int
    :param target: How many degrees the robot should target. type: int
    :param kp: How sharply should the robot correct. type: int
    :return: null
    """
    gyroSensor.reset_angle(0)
    stopWatch.reset()
    # resets stopwatch to 0
    while time >= stopWatch.time():
        error = target + gyroSensor.angle()
        leftMotor.run(error * kp)
    leftMotor.stop(Stop.HOLD)

def gyro_turn_right(sec, target, kp):
    """
    Turns the robot right for a certain time and degrees
    :param sec: The time in (ms) the program should stop. type: int
    :param target: How many degrees the robot should target. type: int
    :param kp: How sharply should the robot correct. type: int
    :return: null
    """
    gyroSensor.reset_angle(0)
    stopWatch.reset()
    # resets stopwatch to 0
    while sec >= stopWatch.time():
        error = target + gyroSensor.angle()
        rightMotor.run(error * kp)
    rightMotor.stop(Stop.HOLD)

def move_forward_degrees(degrees, speed):
    """
    Moves the robot forward or backwards by certain degree at a certain speed
    :param degrees: how many degrees should the robot move. DO NOT PUT NEGATIVE NUMBERS! type: int
    :param speed: how fast the robot should compleate this opperation. type: int
    :return: null
    """
    leftMotor.reset_angle(0)
    mainDriveBase.drive(speed, 0)
    while degrees >= leftMotor.angle():
        pass
    mainDriveBase.stop(Stop.HOLD)



def pid_line_following(kp, ki, kd, degrees, speed):
    """
    Follows a line for a certain amount of degrees
    :param kp: Constant to adjust how much proportion affects the code. type: float
    :param ki: Constant to adjust how much integral affects the code. type: float
    :param kd: Constant to adjust how much derivative affects the code. type: float
    :param degrees: How many degrees the robot should go before stopping. type: int
    :param speed: How fast the robot should go. type: int
    :return: null
    """
    leftMotor.reset_angle(0)
    integral = 0
    last_error = 0
    while degrees >= leftMotor.angle():
         error = (leftMotor.reflection() - rightMotor.reflection())
         integral = integral + error
         derivative = error - last_error
         mainDriveBase.drive(speed, (error * kp) + (integral * ki) + (derivative * kd))
         last_error = error
    mainDriveBase.stop(Stop.HOLD)
    
    
def gyro_follow(speed, target_angle, exit_time):
    """
    Uses the Gyro Sensor to keep robot from Drifting
    :param speed: How fast the robot should go. type: int
    :param target_angle: The angle the Gyro should follow at. type: float
    :param exit_time: The time in (ms) the program should stop. type: int
    :return: null
    """
    stopWatch.time()
    # resets stopwatch to 0
    while exit_time >= stopWatch.time():
         rightMotor.run(gyroSensor.angle() - target_angle + speed)
         leftMotor.run(speed - (gyroSensor.angle() - target_angle))
         print("time()")
    rightMotor.stop(Stop.HOLD)
    leftMotor.stop(Stop.HOLD)
    
    # Code Below is if we want to exit for degrees
    """
    while degrees >= leftMotor.angle():
         rightMotor.run(gyroSensor.angle() - TargetAngle + speed)
         leftMotor.run(speed - (gyroSensor.angle() - TargetAngle))
         print("time()")
         pass
    rightMotor.stop(Stop.HOLD)
    leftMotor.stop(Stop.HOLD)
    """




