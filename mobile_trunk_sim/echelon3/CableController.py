import Sofa


class CableController(Sofa.Core.Controller):   
    """
    Controller to control the cables in forward kinematics
    Parameters :
        mechanicalObject : the mechanicalObject to control
        name : the name of the component 
    """

    def __init__(self, mechanicalObject : Sofa.Core.Node, name : str = "CableController"):
        Sofa.Core.Controller.__init__(self, name = name)
                
        self.Cables = mechanicalObject     

    # action at beginning frame
    def onAnimateBeginEvent(self, eventType):
            pass
  
    #action when key press
    def onKeypressedEvent(self, c):
        key = c['key']        
        ##### Section 1 ######
        
        # Cable 1 
        if (key == "1"):
            displacement = self.Cables.Section1Cable1.value.value+1
            if displacement[0] <= 20 :
                self.Cables.Section1Cable1.value.value = [displacement]
            print("length Section 1 Cable 1 : "+str(displacement[0]))

 
        if (key == "4"):
            displacement = self.Cables.Section1Cable1.value.value-1
            if displacement[0] >= -20 :
                self.Cables.Section1Cable1.value.value = [displacement]
            print("length Section 1 Cable 1 : "+str(displacement[0]))

        # Cable 2 
        if (key == "2"):
            displacement = self.Cables.Section1Cable2.value.value+1
            if displacement[0] <= 20 :
                self.Cables.Section1Cable2.value.value = [displacement]
            print("length Section 1 Cable 2 : "+str(displacement[0]))

        if (key == "5"):
            displacement = self.Cables.Section1Cable2.value.value-1
            if displacement[0] >= -20 :
                self.Cables.Section1Cable2.value.value = [displacement]
            print("length Section 1 Cable 2 : "+str(displacement[0]))

        # cable 3
        if (key == "3"):
            displacement = self.Cables.Section1Cable3.value.value+1
            if displacement[0] <= 20 :
                self.Cables.Section1Cable3.value.value = [displacement]
            print("length Section 1 Cable 3 : "+str(displacement[0]))

        if (key == "6"): 
            displacement = self.Cables.Section1Cable3.value.value-1
            if displacement[0] >= -20 :    
                self.Cables.Section1Cable3.value.value = [displacement]
            print("length Section 1 Cable 3 : "+str(displacement[0]))

        
        # ##### Section 2 ######

        # # Cable 1 
        # if (key == "A"):
        #     displacement = self.Cables.Section2Cable1.value.value+1
        #     if displacement[0] <= 20 :
        #         self.Cables.Section2Cable1.value.value = [displacement]
        #     print("length Section 2 Cable 1 : "+str(displacement[0]))

 
        # if (key == "B"):
        #     displacement = self.Cables.Section2Cable1.value.value-1
        #     if displacement[0] >= -20 :
        #         self.Cables.Section2Cable1.value.value = [displacement]
        #     print("length Section 2 Cable 1 : "+str(displacement[0]))

        # # Cable 2 
        # if (key == "C"):
        #     displacement = self.Cables.Section2Cable2.value.value+1
        #     if displacement[0] <= 20 :    
        #         self.Cables.Section2Cable2.value.value = [displacement]
        #     print("length Section 2 Cable 2 : "+str(displacement[0]))

        # if (key == "D"):
        #     displacement = self.Cables.Section2Cable2.value.value-1
        #     if displacement[0] >= -20 :
        #         self.Cables.Section2Cable2.value.value = [displacement]
        #     print("length Section 2 Cable 2 : "+str(displacement[0]))

        # # cable 3
        # if (key == "E"):
        #     displacement = self.Cables.Section2Cable3.value.value+1
        #     if displacement[0] <= 20 :
        #         self.Cables.Section2Cable3.value.value = [displacement]
        #     print("length Section 2 Cable 3 : "+str(displacement[0]))

        # if (key == "F"):
        #     displacement = self.Cables.Section2Cable3.value.value-1
        #     if displacement[0] >= -20 :
        #         self.Cables.Section2Cable3.value.value = [displacement]
        #     print("length Section 2 Cable 3 : "+str(displacement[0]))

        # ##### Section 3 ######

        # # Cable 1 
        # if (key == "G"):
        #     displacement = self.Cables.Section3Cable1.value.value+1
        #     if displacement[0] <= 20 :    
        #         self.Cables.Section3Cable1.value.value = [displacement]
        #     print("length Section 3 Cable 1 : "+str(displacement[0]))

 
        # if (key == "H"):
        #     displacement = self.Cables.Section3Cable1.value.value-1
        #     if displacement[0] >= -20 :    
        #         self.Cables.Section3Cable1.value.value = [displacement]
        #     print("length Section 3 Cable 1 : "+str(displacement[0]))

        # # Cable 2 
        # if (key == "I"):
        #     displacement = self.Cables.Section3Cable2.value.value+1
        #     if displacement[0] <= 20 :    
        #         self.Cables.Section3Cable2.value.value = [displacement]
        #     print("length Section 3 Cable 2 : "+str(displacement[0]))

        # if (key == "J"):
        #     displacement = self.Cables.Section3Cable2.value.value-1
        #     if displacement[0] >= -20 :
        #         self.Cables.Section3Cable2.value.value = [displacement]
        #     print("length Section 3 Cable 2 : "+str(displacement[0]))

        # # cable 3
        # if (key == "K"):
        #     displacement = self.Cables.Section3Cable3.value.value+1
        #     if displacement[0] <= 20 :    
        #         self.Cables.Section3Cable3.value.value = [displacement]
        #     print("length Section 3 Cable 3 : "+str(displacement[0]))

        # if (key == "L"):
        #     displacement = self.Cables.Section3Cable3.value.value-1
        #     if displacement[0] >= -20 :
        #         self.Cables.Section3Cable3.value.value = [displacement]
        #     print("length Section 3 Cable 3 : "+str(displacement[0]))

    # action at end frame
    def onAnimateEndEvent(self, eventType):
        pass