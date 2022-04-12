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

def Floor(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[0, 0, 0],
          position=[0.0, 0.0, 0, 0.0, 0.0, 0.0, 1.], translation=[-1, -1, -0.15]):
    floor = parentNode.addChild('Floor')
    floor.addObject('MeshObjLoader', name='loader', filename='mesh/square1.obj', scale=2, rotation=rotation, translation=translation)
    floor.addObject('OglModel', src='@loader', color=color)
    floor.addObject('MeshTopology', src='@loader', name='topo')
    floor.addObject('MechanicalObject')
    floor.addObject('TriangleCollisionModel')
    floor.addObject('LineCollisionModel')
    floor.addObject('PointCollisionModel')
    return floor

def createWheel(parent, name, wheel_position, wheel_orientation):
    body = parent.addChild(name)
    body.addObject('MechanicalObject', name='dofs', showObject=True, template='Rigid3',
                    position=[wheel_position[0], wheel_position[1], wheel_position[2],
                              wheel_orientation[0], wheel_orientation[1], wheel_orientation[2],
                              wheel_orientation[3]],showObjectScale=0.09)
    visual = body.addChild('VisualModel')

    visual.addObject('MeshSTLLoader', name='loader1', filename='meshes/wheel.stl')

    visual.addObject('MeshTopology', src='@loader1')

    visual.addObject('OglModel', name='renderer', src='@loader1', color="0.2 0.2 0.2 1")

    visual.addObject('RigidMapping',
                        input=body.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return body

def create_sensor(parent, sensor_name, sensor_orientation = [0.0, 0.0, 0.0, 1.]):

    sensor_position = sensor_dict[sensor_name]

    body = parent.addChild(sensor_name)
    body.addObject('MechanicalObject', name='dofs',  template='Rigid3',
                    position=[sensor_position[0], sensor_position[1],
                              sensor_position[2],sensor_orientation[0],
                              sensor_orientation[1], sensor_orientation[2],
                              sensor_orientation[3]],showObjectScale=0.09,
                              showObject=True)
    visual = body.addChild('VisualModel')
    path = sensorname_to_path(sensor_name)
    visual.addObject('MeshSTLLoader' , name='loader1', filename=path)

    visual.addObject('MeshTopology', src='@loader1')

    visual.addObject('OglModel', name='renderer', src='@loader1', color="0.5 0.5 0.5 1")

    visual.addObject('RigidMapping',
                        input=body.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return body