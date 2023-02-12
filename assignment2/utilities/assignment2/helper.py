#!/usr/bin/env python

# Import ROS libraries.
import rospy
from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# Import constant names that define the architecture's structure.
from assignment2 import architecture_name_mapper as anm
from assignment2.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse, GetBattery, SetBattery, GetBatteryResponse, SetBatteryResponse
from assignment2.msg import Point


import rospy
import time
import re

from armor_api.armor_client import ArmorClient
# Import ROS-based messages.
from std_msgs.msg import Bool

is_In = []
can_reach = []
query_isIn =[]
query_canReach =[]
rob_dataprop = []
loc_dataprop = []

class TopologicalMap:

    def __init__(self):
        self.client1 = ArmorClient('example', 'ontoRef')
        self.load_map()
        #self.update_othology()
        #self.get_location()
        #self.get_pose_client()

    def load_map(self):
        self.client1.utils.load_ref_from_file('/home/awais/my_ros_ws/src/EXPROB_Assgnment_02/assignment2/map/topological_map_abox_new.owl', 
                                'http://bnc/exp-rob-lab/2022-23', 
                                buffered_manipulation=True, 
                                reasoner='PELLET', 
                                buffered_reasoner=True, 
                                mounted=False)

        #client1.query.check_ind_exists('C1')
        self.client1.manipulation.add_ind_to_class('C1', 'CORRIDOR')
        self.client1.manipulation.add_ind_to_class('C2', 'CORRIDOR')
        self.client1.manipulation.add_ind_to_class('D1', 'DOOR')
        self.client1.manipulation.add_ind_to_class('D2', 'DOOR')
        self.client1.manipulation.add_ind_to_class('D3', 'DOOR')
        self.client1.manipulation.add_ind_to_class('D4', 'DOOR')
        self.client1.manipulation.add_ind_to_class('D5', 'DOOR')
        self.client1.manipulation.add_ind_to_class('D6', 'DOOR')
        self.client1.manipulation.add_ind_to_class('D7', 'DOOR')
        self.client1.manipulation.add_ind_to_class('E1', 'ROOM')
        self.client1.manipulation.add_ind_to_class('ROBOT1', 'ROBOT')

        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D6')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D5')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D2')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D1')

        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D5')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D3')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D4')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D7')

        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D7')
        self.client1.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D6')

        self.client1.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
        
        self.client1.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])

        time1 = str(int(time.time()))

        # Add time
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','E', 'Long', time1)
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','C1', 'Long', time1)
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','C2', 'Long', time1)
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','R1', 'Long', time1)
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','R2', 'Long', time1)
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','R3', 'Long', time1)
        self.client1.manipulation.add_dataprop_to_ind('visitedAt','R4', 'Long', time1)

        res = self.client1.utils.sync_buffered_reasoner() 
        print("Response : ", res)

        if res == True:
            print("Successfully Received the Map\n")
            print("Thank You\n")
            return
        else:
            print("Failed to load map")






    
