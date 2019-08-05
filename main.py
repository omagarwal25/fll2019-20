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
rightSensor = ColorSensor(Port.S2)

mainDriveBase = DriveBase(leftMotor, rightMotor)


def move_forward_degrees(degrees, speed):
    leftMotor.reset_angle()
    mainDriveBase.drive(speed, 0)
    while degrees >= leftMotor.angle():
        pass
    mainDriveBase.stop(Stop.BRAKE)

def pid_line_following(kP, kI, kD, degrees, speed):
    leftMotor.reset_angle()
    error = 0
    integral = 0
    last_error = 0
    derivative = 0
    while degrees >= leftMotor.angle():
        error = (leftMotor.reflection() - rightMotor.reflection())
        integral = integral + error
        derivative = error - last_error
        mainDriveBase.drive(speed, (error * kP) + (integral * kI) + (derivative * kD))
        last_error = error
    mainDriveBase.stop(Stop.BRAKE)

# Write your program here
brick.sound.beep()

