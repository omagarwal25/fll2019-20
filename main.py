#!/usr/bin/env pybricks-micropython


from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

l_i = 0
r_i = 0

leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)

leftSensor = ColorSensor(Port.S1)
rightSensor = ColorSensor(Port.S4)

gyroSensor = GyroSensor(Port.S3)

mainDriveBase = DriveBase(leftMotor, rightMotor)


def gyro_turn_left(sec, target, kp):
    """
    Turns the robot left for a certain time and degrees
    :param sec: How many seconds should the robot take. type: int
    :param target: How many degrees the robot should target. type: int
    :param kp: How sharply should the robot correct. type: int
    :return: null
    """
    gyroSensor.reset_angle(0)
    error = 0
    l_i = 0
    while l_i != sec:
        error = target - gyroSensor.angle()
        leftMotor.run(error * kp)
        wait(10)
        l_i = l_i + 1
    leftMotor.stop(Stop.BRAKE)

def gyro_turn_right(sec, target, kp):
    """
    Turns the robot right for a certain time and degrees
    :param sec: How many seconds should the robot take. type: int
    :param target: How many degrees the robot should target. type: int
    :param kp: How sharply should the robot correct. type: int
    :return: null
    """
    gyroSensor.reset_angle(0)
    error = 0
    r_i = 0
    while r_i != sec:
        error = target + gyroSensor.angle()
        rightMotor.run(error * kp)
        wait(10)
        r_i = r_i + 1
    rightMotor.stop(Stop.BRAKE)

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
    mainDriveBase.stop(Stop.BRAKE)



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
    error = 0
    integral = 0
    last_error = 0
    derivative = 0
    while degrees >= leftMotor.angle():
        error = (leftMotor.reflection() - rightMotor.reflection())
        integral = integral + error
        derivative = error - last_error
        mainDriveBase.drive(speed, (error * kp) + (integral * ki) + (derivative * kd))
        last_error = error
    mainDriveBase.stop(Stop.BRAKE)



