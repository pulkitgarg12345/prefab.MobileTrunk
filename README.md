# prefab.MobileTrunk
[![SoftRobots](https://img.shields.io/badge/SoftRobots-on_github-firebrick.svg)](https://github.com/SofaDefrost) 
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


Requirement
-----------
This prefab was developed as part of the SOMOROB project. The goal is to integrate a
deformable robotic trunk (softrobot) on a 4-wheeled mobile robot base equipped with 
a LIDAR and to develop its digital twin equivalent. The deformable trunk is a project
[Echelon 3](https://www.inria.fr/en/interface-inria-centre-university-lille-demonstration-space)
developed by the Defrost team at INRIA. The mobile base is developed by the company
[Robotnik automation](https://robotnik.eu/)

Compile *SOFA*
-------------------
For that follow these [instructions]() (We will install the linux one)

Install Required dependencies
------------------------------------
For that follow these [instructions]() (We will install the linux one)
(The best way to add plugin to SOFA is explained [here]())

- STLIB

    ```console
    foo@bar:~$  git clone https://github.com/SofaDefrost/STLIB.git
    ```

- SoftRobots

    ```console
    foo@bar:~$  git clone https://github.com/SofaDefrost/SoftRobots.git
    ```

- SofaPython3

    ```console
    foo@bar:~$  git clone https://github.com/sofa-framework/SofaPython3
    ```
    

- BeamAdapter

    ```console
    foo@bar:~$  git clone https://github.com/SofaDefrost/BeamAdapter
    ```

Download and set up working space
----------------------------------

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
        export SOFA_ROOT=/path to SofaPython3 plugin folder/SofaPython3
    ```

- Faire un alias afin de pouvoir lancer Sofa faciement en faisant un runSofa
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

    Install the [noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) version for ros1 in order to
    be able to use rosbridge

    Install the [foxy](https://docs.ros.org/en/foxy/Installation.html) version for ros2.

    Install rosbridge
    
    ```console
    foo@bar:~$  sudo apt-get install -y ros-foxy-ros1-bridge
    ```
    If you are new to ros you can follow the [tutorials](https://docs.ros.org/en/foxy/Tutorials.html)
    before continuing

### Set up working space

Open your bashrc and add the following lines in order to setup your working space for ros

-  Crée un alias pour noetic(ros1) et foxy(ros2) afin de pouvoir sourcer ton terminal plus facilement
    
    ```console
        alias foxy="source /opt/ros/foxy/setup.bash"
        alias noetic="source /opt/ros/noetic/setup.bash"
    ```

- Add to PYTHONPATH the path to sofaros api
 
     ```console
        export PYTHONPATH=$PYTHONPATH:/path to SoftRobot plugin folder/SoftRobots/docs/sofapython3/examples/sofaros
    ```


    ```console
    foo@bar:~$  runSofa summit_xl
    ```
Launch test
-----------
To confirm all the previous steps and verify that the prefab is working properly you can : 

1. Launch the summit_xl.py SOFA scene situated in prefab.MobileTrunk/mobile_trunk_sim by doing:

    ```console
    foo@bar:~$  runSofa summit_xl
    ```
If everything went well you should see the GUI Sofa with the digital twin of

![somorob](/docs/sofa.png/ "somorob launch test")

Then with your keyboard send velocity and orientation command to the Sofa scene in order to see the robot
move. 



