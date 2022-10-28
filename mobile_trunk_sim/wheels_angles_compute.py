import numpy as np

wheel_ray = 0.113 # rayan de la roue
lz = 0.229 #distance entre le centre d'une des roues et le centre du chassis sur l'axe des z
lx = 0.2355 #distance entre le centre d'une des roues et le centre du chassis sur l'axe des x

def twistToWheelsAngularSpeed(wy, vz):
    """
        The robot moves along the z axis and rotates on the y axis.
        twistToWheelsAngles method calculate the angular velocity
            of each of the wheels
    """
    H = np.array([[1, -1, -(lx+ lz)],
                [1, 1, (lx + lz)],
                [1, 1, -(lx + lz)],
                [1, -1, (lx + lz)]])/wheel_ray
    twist = np.array([vz, 0, wy]) # vx = 0
    twist.shape = (3, 1)
    wheels_angular_speed = np.dot(H, twist) #calculate the angular speed of each wheel
    wheels_angular_speed = wheels_angular_speed.flatten().tolist()
    return wheels_angular_speed

def move(WheelsMotors_angles_rest_position, wheels_angular_speed, dt):
    """
    Move method allow the robot to move thanks to the angular velocity of 
    each of the wheels
    """
    with WheelsMotors_angles_rest_position.writeable() as angles:
        #Make the wheel turn according to their angular speed

        angles[0] += wheels_angular_speed[0] * dt
        angles[2] += wheels_angular_speed[2] * dt
        angles[1] += wheels_angular_speed[1] * dt
        angles[3] += wheels_angular_speed[3] * dt
    return WheelsMotors_angles_rest_position.value