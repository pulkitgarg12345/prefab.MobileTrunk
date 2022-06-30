import Sofa
import numpy as np


class BaseController(Sofa.Core.Controller):   
    """
    Controller to control the base in forward kinematics
    Parameters :
        mechanicalObject : the mechanicalObject to control 
        name : the name of the component 
    """

    def __init__(self, mechanicalObject : Sofa.Core.Node, maxDisplacement : float, name : str = "BaseController"):
        Sofa.Core.Controller.__init__(self, name = name)
        
        self.mechanicalObject = mechanicalObject
        self.maxDisplacement = maxDisplacement
        self.displacement = 0

  
    # #action when key press
    def onKeypressedEvent(self, c):
        key = c['key']
        base = np.array(self.mechanicalObject.rest_position.value)
        print(base)

        if key == "+":
            if self.displacement <= self.maxDisplacement:
                base[0][0] +=1
                self.mechanicalObject.rest_position.value = base.tolist()
                self.displacement+=1

        if key == "-":
            if self.displacement >= -self.maxDisplacement:
                base[0][0] -=1
                self.mechanicalObject.rest_position.value = base.tolist()
                self.displacement-=1

