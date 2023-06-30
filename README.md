# prefab.MobileTrunk
[![SoftRobots](https://img.shields.io/badge/info-on_website-purple.svg)](https://project.inria.fr/softrobot/) 
[![SOFA](https://img.shields.io/badge/SOFA-on_github-blue.svg)](https://github.com/sofa-framework)
[![Ronotnik Automation](https://img.shields.io/badge/RonotnikAutomation-on_github-green.svg)](https://github.com/RobotnikAutomation)
[![Ros](https://img.shields.io/badge/Ros-on_readthedocs-chocolate.svg)](https://docs.ros.org/en/foxy/index.html)

The *MobileTrunk's* robot prefab for SoftRobots. 

Index
-----

  * [Requirement](#requirement)
  * [Compile sofa](#compile-sofa)
  * [Install Required dependencies](#install-required-dependencies)
  * [Download and set up working space](#download-and-set-up-working-space)
  * [Install *ROS* and set up working space](#install-ros-and-set-up-working-space)
  * [Launch test](#launch-test)
  * [Connecting the digital twin to the physical robot](#connecting-the-digital-twin-to-the-physical-robot)
  * [License](#license)

Requirement
-----------
This prefab was developed as part of the SOMOROB project. The goal is to integrate a
deformable robotic trunk (softrobot) on a 4-wheeled mobile robot base equipped with 
a LIDAR and to develop its digital twin equivalent. The deformable trunk is a project
[Echelon 3](https://www.inria.fr/en/interface-inria-centre-university-lille-demonstration-space)
developed by the Defrost team at INRIA. The mobile base is developed by the company
[Robotnik automation](https://robotnik.eu/)

Software version
---------------
The softwares are use with the following version :
- Python 3

The following Sofa plugins are required. version V21.12 is needed.

- SofaPython3
- BeamAdapter
- SoftRobots

The following python module are required :

- time
- threading
- math
- numpy
- dataclasses
- sys

Compile *SOFA*
--------------

Install Required dependencies
------------------------------------
For that follow these [instructions](https://www.sofa-framework.org/download/) (We will install the linux one)
  ```

Download and set up working space
---------------------------------

### Download the mobile trunk's prefab

Now that you have a compiled & working SOFA with the required plugins, we can clone
prefab.MobileTrunk repository.

- Create a directory where you will put it
- Move into it and clone the last working version

    ```console
    foo@bar:~$  git clone https://github.com/CRIStAL-PADR/prefab.MobileTrunk
    ```

### Set up working space
Open your bashrc and add the following lines in order to setup your working space
- Add to the PATH the path to the bin folder contained in the SOFA build folder

    ```console
        export PATH="/path to buid folder/build/bin:$PATH"
    ```

- Tell to  *SOFA_ROOT* the path where to find the path to the buid folder of SOFA

    ```console
        export SOFA_ROOT=/path to buid folder/build
    ```

- Tell to  *SOFAPYTHON3_ROOT* where to find the path to the plugin SofaPython3

    ```console
        export SOFAPYTHON3_ROOT=/path to SofaPython3 plugin folder/SofaPython3
    ```

- Make an alias in order to be able to launch Sofa easily by doing a runSofa

    ```console
        runSofa="/path to buid folder/build/bin/runSofa"
    ```

- Add to the PYTHONPATH the path to *STLIB*
    ```console
        export PYTHONPATH=$PYTHONPATH:/path to build folder/build/lib/python3/site-packages:/path to STLIB plugin folder/STLIB/python3/src:/usr/local/lib/python3.8/dist-packages
    ```

Install *ROS* and set up working space
--------------------------------------

The digital twin has a *ros2* interface while the mobile platform (summit_xl) has a *ros1* interface.
It is possible to be able to link the digital twin to the real robot using rosbridge.

### Install ros2 and ros1

Install the [noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) version for ros1 in order to be able to use rosbridge

Install the [foxy](https://docs.ros.org/en/foxy/Installation.html) version for ros2.

If you are new to ros you can follow the [tutorials](https://docs.ros.org/en/foxy/Tutorials.html) before continuing

Install rosbridge
    
```console
    foo@bar:~$  sudo apt-get install -y ros-foxy-ros1-bridge
```

### Set up working space

Open your bashrc and add the following lines in order to setup your working space for ros

- Create an alias for noetic(ros1) and foxy(ros2) in order to be able to source your terminal more easily

    ```console
        alias foxy="source /opt/ros/foxy/setup.bash"
        alias noetic="source /opt/ros/noetic/setup.bash"
    ```

- Add to PYTHONPATH the path to sofaros api
 
     ```console
        export PYTHONPATH=$PYTHONPATH:/path to SoftRobot plugin folder/SoftRobots/docs/sofapython3/examples/sofaros
    ```

Launch test
-----------
To confirm all the previous steps and verify that the prefab is working properly you can :

#### Test using sofa controller

-  Launch the scene.py SOFA scene situated in prefab.MobileTrunk/mobile_trunk_sim by doing:
 ```console
    foo@bar:~$  runSofa scene.py --argv KeyboardController
```
- If everything went well you should see the GUI Sofa with the digital twin of

    ![somorob](/docs/somorob.png)


- Then with your keyboard send velocity and orientation command to the Sofa scene in order to see the robot
move.
 

#### Test using ros api

- source your terminal using : 
    ```console
    foo@bar:~$  foxy
    ```

- Go to the directory where the prefab.MobleTrunk/mobile_trunk_sim folder is located and run 
 the following command:

    ```console
    foo@bar:~$  runSofa scene.py --argv  RobotToSim
    ```
- Open a new terminal then source it as before then execute the commands to the following command:

    ```console
        foo@bar:~$  colcon build && . install/setup.bash && foxy
    ```

-  You can now launch the keyboard controller to interact with the robot in the simulation

    ```console
        foo@bar:~$  ros2 run summitxl summitxl_teleop_key

    ```

- If everything went well you should see this appear in the terminal
    
    ![ros_teleop_key](/docs/ros_teleopkey.png/)


## Echelon

### Controller

#### Forward kinematics

- ctl + i : To select the cable i (from 1 to 9) you want to actuate
- ctl + plus : if you want to pull the cable
- ctl + minus : if you want to release the cable

### Links

You can found the Echelon3 folder with the reference simulation there : https://gitlab.inria.fr/defrost/Robots/Echelon3.git with the branch Centrale_Dynamixel. The reference simulation is in the folder **simulation**.

There is also the ros2 package : https://gitlab.inria.fr/defrost/Robots/sim_echelon3_ros.git with the branch ros2.

You can find all the details and the explanation in this two repository.

Connecting the digital twin to the physical robot
-------------------------------------------------


License
-------

Creative Commons Zero v1.0 Universal see LICENSE
