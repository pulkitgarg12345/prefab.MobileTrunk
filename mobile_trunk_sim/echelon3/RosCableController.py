# coding: utf8
#!/usr/bin/env python3
import Sofa
from std_msgs.msg import Float64

def cable_displacement_recv(data, datafield):

   
    datafield.value = data.data
    

class CableROSController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.cables = kwargs["cables"]
        self.robot = kwargs["robot"]
        self.robotToSim = kwargs["robotToSim"]
        self.index = 0
        
    # action at beginning frame
    def onAnimateBeginEvent(self, event):
        dt = event['dt']
        print("--->", self.robot.effector_cable_data.value)

        if self.robotToSim:
            with self.cables[0].value.writeable() as t:

                displacement = t[0] + self.robot.effector_cable_data.value
                t[0]= displacement
                #print(t[0])        
            #print(self.cables[0].value.value)
