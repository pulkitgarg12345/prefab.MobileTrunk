import Sofa


class ForceController(Sofa.Core.Controller):   
    """
    Controller to control the cables in forward kinematics
    Parameters :
        mechanicalObject : the mechanicalObject to control
        dt : step time as in Sofa Force(Sofa)  = Force(N) *dt 
        name : the name of the component 
    """

    def __init__(self, mechanicalObject : Sofa.Core.Node, dt : float, name : str = "CableController"):
        Sofa.Core.Controller.__init__(self, name = name)
                
        self.Cables = mechanicalObject     
        self.forceMax = 49050000 # max 6kg
        self.forceMin = 0
        self.dt = dt

    # action at beginning frame
    def onAnimateBeginEvent(self, eventType):
            pass
  
    #action when key press
    def onKeypressedEvent(self, c):
        key = c['key']        
        ##### Section 1 ######
        
        # Cable 1 
        if (key == "1"):
            force = self.Cables.Section1Cable1.value.value + 981000 # F = g.mm.s-2 ==> 100g : 100*9810 =  g.mm.s-2
            if force[0] <= self.forceMax :
                self.Cables.Section1Cable1.value.value = [force*self.dt]
            print("force Section 1 Cable 1 : "+str(force[0]*10**-6.0))

        if (key == "4"):
            force = self.Cables.Section1Cable1.value.value - 981000 # F = g.mm.s-2 ==> 500g : 500*9810 = 4.905*10^6 g.mm.s-2
            if force[0] >= self.forceMin :
                self.Cables.Section1Cable1.value.value = [force * self.dt]
            print("force Section 1 Cable 1 : "+str(force[0]))

        # Cable 2 
        if (key == "2"):
            force = self.Cables.Section1Cable2.value.value + 981000 # F = g.mm.s-2 ==> 500g : 500*9810 = 4.905*10^6 g.mm.s-2
            if force[0] <= self.forceMax :
                self.Cables.Section1Cable2.value.value = [force * self.dt]
            print("force Section 1 Cable 2 : "+str(force[0]))

        if (key == "5"):
            force = self.Cables.Section1Cable2.value.value - 981000 # F = g.mm.s-2 ==> 500g : 500*9810 = 4.905*10^6 g.mm.s-2
            if force[0] >= self.forceMin :
                self.Cables.Section1Cable2.value.value = [force * self.dt]
            print("force Section 1 Cable 2 : "+str(force[0]))

        # cable 3
        if (key == "3"):
            force = self.Cables.Section1Cable3.value.value + 981000 # F = g.mm.s-2 ==> 500g : 500*9810 = 4.905*10^6 g.mm.s-2
            if force[0] <= self.forceMax :
                self.Cables.Section1Cable3.value.value = [force * self.dt]
            print("force Section 1 Cable 3 : "+str(force[0]))

        if (key == "6"): 
            force = self.Cables.Section1Cable3.value.value - 981000 # F = g.mm.s-2 ==> 500g : 500*9810 = 4.905*10^6 g.mm.s-2
            if force[0] >= self.forceMin :
                self.Cables.Section1Cable3.value.value = [force * self.dt]
            print("force Section 1 Cable 3 : "+str(force[0]))
