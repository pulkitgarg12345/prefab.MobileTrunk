import os
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

sensor_dict = {"imu_offset":[-0.18, 0, 0.170],
               "gps_offset" : [-0.22, -0.0, 0.275],
               "rear_laser_offset" : [-0.2865, 0.20894, 0.2973],
               "front_rgbd_camera_offset": [0.35850, 0.0, 0.25062]
            }


def sensorname_to_path(argument):
	switcher = {
		"imu_offset": dirPath+'meshes/antenna_3GO16.stl',
		"gps_offset": dirPath+'meshes/axis_m5525.stl',
		"front_rgbd_camera_offset": dirPath+'meshes/camera_axis_q8641.stl',
	}
	return switcher.get(argument, "nothing")

# Driver program
if __name__ == "__main__":
    argument="imu_offset"
    path = sensorname_to_path(argument)
    print("-------->", path)
