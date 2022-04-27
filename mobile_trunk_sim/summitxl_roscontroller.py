# coding: utf8
#!/usr/bin/env python3
import Sofa
from splib3.numerics import RigidDof, Quat
import std_msgs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


from math import *

receive_data = Twist()
def send(data):
    """This is a message to hold data from an IMU (Inertial Measurement Unit)

    Accelerations should be in m/s^2 (not in g's),
    and rotational velocity should be in rad/sec


    Args:
        data[0]: Orientation
        data[1] : angular_vel
        data[2] : linear_acceleration

    Returns:
        msg: return  data as ros msg
    """
    msg = Imu()
    msg.orientation.x = data[0].value[0]
    msg.orientation.y = data[0].value[1]
    msg.orientation.z = data[0].value[2]
    msg.orientation.w = data[0].value[3]

    msg.angular_velocity.x = data[1].value[0]
    msg.angular_velocity.y = data[1].value[1]
    msg.angular_velocity.z = data[1].value[2]

    msg.linear_acceleration.x = data[2].value[0]
    msg.linear_acceleration.y = data[2].value[1]
    msg.linear_acceleration.z= data[2].value[2]

    msg.header.stamp.sec =  int(data[3].value[0])
    msg.header.stamp.nanosec = int(data[3].value[1])
    #print("orientation = ",msg.orientation.x, msg.orientation.y, msg.orientation.z)
    #print(" msg.angular_velocity = ", msg.angular_velocity.x,  msg.angular_velocity.y,  msg.angular_velocity.z)
    #print(" msg.linear_acceleration = ", msg.linear_acceleration.x,  msg.linear_acceleration.y,  msg.linear_acceleration.z)
    #print(" msg.header.stamp = ", msg.header.stamp.sec, msg.header.stamp.nanosec)


    return msg

def vel_recv(data, datafield):
    datafield[0].value = [data.angular.z ,data.angular.y, data.angular.x]
    datafield[1].value = [data.linear.z ,data.linear.y, data.linear.x]

def time_recv(data, datafield):
    t = data.tolist()
    datafield.value = [t[0], t[1]]


class SummitxlROSController(Sofa.Core.Controller):
    """A Simple keyboard controller for the SummitXL
       Key UP, DOWN, LEFT, RIGHT to move
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.robot.linear_vel[0] = 0.
        self.robot.angular_vel[2] = 0.
        self.robot.linear_acceleration[0] = 0.
        self.robot.linear_acceleration[1] = 0.
        self.robot.linear_acceleration[2] = 0.
        self.wheel_ray = 0.0015

    def move(self, fwd, angle):
        """Move the robot using the forward speed and angular speed)"""
        robot = RigidDof(self.robot.Chassis.position)
        robot.translate(robot.forward * fwd)
        robot.rotateAround([0, 1, 0], angle)
        with self.robot.Chassis.WheelsMotors.angles.position.writeable() as angles:
            #Make the wheel turn according to forward speed
            # TODO: All the value are random, need to be really calculated
            angles += (fwd/self.wheel_ray)

            #Make the wheel turn in reverse mode according to turning speed
            # TODO: the value are random, need to be really calculated
            angles[0] += (angle)
            angles[2] += (angle)
            angles[1] -= (angle)
            angles[3] -= (angle)

    def onAnimateBeginEvent(self, event):
        """At each time step we move the robot by the given forward_speed and angular_speed)
           TODO: normalize the speed by the dt so it is a real speed
        """
        dt = event['dt']
        self.robot.linear_vel[0] = self.robot.linear_vel[0]*dt
        self.robot.angular_vel[2] = self.robot.angular_vel[2] * dt
        self.robot.linear_acceleration[0] = self.robot.linear_vel[0]/(dt)
        for i in range(0, 4):
            self.robot.orientation[i] = self.robot.Chassis.position.position.value[0][i+3]
        self.move(self.robot.linear_vel[0], self.robot.angular_vel[2])
