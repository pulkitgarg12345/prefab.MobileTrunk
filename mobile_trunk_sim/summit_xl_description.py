import os
from splib3.numerics import Quat
from math import pi
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'


front_left_wheel_position = [0.229, 0.235, 0.0]
back_left_wheel_position = [-0.229, 0.235, 0.0]
front_right_wheel_position = [0.229, -0.235, 0.0]
back_right_wheel_position = [-0.229, -0.235, 0.0]

front_left_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
front_right_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
back_left_wheel_orientation = [0.0, 0.0, 0.0, 0.0]
back_right_wheel_orientation = [0.0, 0.0, 0.0, 0.0]

front_rgbd_camera_offset = [0.35850, 0.0, 0.25062]

imu_offset = [-0.18, 0, 0.170]

gps_offset = [-0.22, -0.0, 0.275]

rear_laser_offset = [-0.2865, 0.20894, 0.2973]

def orientation(angle,axis):
    orientation = [0.0, 0.0, 0.0, 0.0]
    orientation = Quat.createFromAxisAngle(axis, angle)
    orientation.rotateFromEuler([angle, 0. , 0.])

    return orientation

camera_orientation = orientation(0.,[0., 1., 0.]) # 15*pi/180

front_right_wheel_orientation = orientation(pi/2, [1.0, 0., 0])

back_right_wheel_orientation = orientation(pi/2, [1.0, 0., 0])

front_left_wheel_orientation = orientation(pi, [1.0, 0., 0])

back_left_wheel_orientation = orientation(pi, [1.0, 0., 0])


sensor_dict = {"imu_offset":[-0.18, 0., 0.27], #correct
               "gps_offset" : [0.22, -0.0, 0.275],
               "rear_laser_offset" : [-0.2865, 0.20894, 0.2973],
               "front_rgbd_camera_offset": [0.19, 0.0, 0.245] #correct
            }


def sensorname_to_path(argument):
	switcher = {
		"imu_offset": dirPath+'meshes/antenna_3GO16.stl',
		"gps_offset": dirPath+'meshes/wheel.stl',
		"front_rgbd_camera_offset": dirPath+'meshes/axis_p5514.stl',
	}
	return switcher.get(argument, "nothing")
