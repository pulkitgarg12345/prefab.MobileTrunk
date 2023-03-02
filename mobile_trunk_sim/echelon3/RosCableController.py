# coding: utf8
#!/usr/bin/env python3
import Sofa
from std_msgs.msg import Float64
import rclpy

def cable_displacement_recv(data, datafield):

   
    datafield.value = data.data
    

class CableROSController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.cables = kwargs["cables"]
        self.robot = kwargs["robot"]
        self.rosNode = kwargs["rosNode"]
        self.robotToSim = kwargs["robotToSim"]
        self.receiving_topic = None
        self.index = 0
        self.Flag = False

    def get_index_by_rosNode(self): 
        
        #Get active topic name and type
        topics_names_and_types = self.rosNode.get_topic_names_and_types()

        for topic_name, topic_type in topics_names_and_types:
            #check the topic_type std_msgs/msg/Float64 to get corresponding topic_name
            if topic_type ==['std_msgs/msg/Float64']:
                #get the list of publishers thanks to the get_publishers_info_by_topic method
                publishers = self.rosNode.get_publishers_info_by_topic(topic_name)
                if publishers:
                    #self.Flag = False
                    if not self.Flag:

                        # If publishers is True => whe have topic which is receiving message
                        self.receiving_topic = topic_name
                        #print(f"Topic {receiving_topic} is receiving messages of type std_msgs/msg/Float64")
                        print("when publishing -->",self.receiving_topic)
                        print("-----------")
                        topic_processing = self.receiving_topic.strip().split('/')
                        self.index = int(topic_processing[2].strip("Cable"))
            
                else:
                    self.Flag = True
                    print("+++++++++++")
        # self.Flag = False

        # if self.receiving_topic is not None:
        #     self.Flag = True
        #     # print("-----------")
        #     topic_processing = self.receiving_topic.strip().split('/')
        #     self.index = int(topic_processing[2].strip("Cable"))

    
    # action at beginning frame
    def onAnimateBeginEvent(self, event):
               
        self.get_index_by_rosNode()

        # if not self.Flag : 
        #     print("--------")
        #     with self.cables[self.index].value.writeable() as t:
        #         t[0] +=  self.robot.effector_cable_data.value
        # else:
        #     print("+++++++++")
        #     self.cables[self.index].value.value = 0.
        #     #print(self.cables[self.index].value.value)


                                                

