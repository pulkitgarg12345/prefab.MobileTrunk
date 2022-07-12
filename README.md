# prefab.MobileTrunk
[![SoftRobots](https://img.shields.io/badge/SoftRobots-on_github-orange.svg)](https://github.com/SofaDefrost) 
[![SOFA](https://img.shields.io/badge/SOFA-on_github-blue.svg)](https://github.com/sofa-framework)


The MobileTrunk's robot prefab for SoftRobots. 


## Requirement
This prefab was developed as part of the SOMOROB project. The goal is to integrate a
deformable robotic trunk (softrobot) on a 4-wheeled mobile robot base equipped with 
a LIDAR and to develop its digital twin equivalent. The deformable trunk is a project
[Echelon 3](https://www.inria.fr/en/interface-inria-centre-university-lille-demonstration-space)
developed by the Defrost team at INRIA. The mobile base is developed by the company
[Robotnik automation](https://robotnik.eu/)

### Step1: Compile SOFA
For that follow these [instructions]() (We will install the linux one)

### Step2: Install Required dependencies
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

### Step3: Download and set up working space

#### Download the mobile trunk's prefab

Now that you have a compiled & working SOFA with the required plugins, we can clone
prefab.MobileTrunk repository.

- Create a directory where you will put it
- Move into it and clone the last working version

    ```console
    foo@bar:~$  git clone https://github.com/CRIStAL-PADR/prefab.MobileTrunk
    ```

#### Set up working space
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
        runSofa="/home/fabrice/Documents/CRIStAL/sofa/build/bin/runSofa"
    ```

- Add to the PYTHONPATH the path to *STLIB*
    ```console
        export PYTHONPATH=$PYTHONPATH:/path to build folder/build/lib/python3/site-packages:/path to STLIB plugin folder/STLIB/python3/src:/usr/local/lib/python3.8/dist-packages
    ```

### Step4: Launch test
To confirm all the previous steps and verify that the prefab is working properly you can launch the summit_xl.py.py SOFA scene situated in:

    ```console
        prefab.MobileTrunk/mobile_trunk_sim
    ```