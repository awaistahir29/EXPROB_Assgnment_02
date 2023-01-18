#!/usr/bin/env python3

import roslib
import rospy
import smach
import smach_ros
import time
import random
import re
import sys

from armor_api.armor_client import ArmorClient
urgency = False


def user_action():
    return random.choice(['discharged','timeup','charged','loaded','relaxed'])

# define state Unlocked
class Map_Receiving(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['loaded'])
                             #input_keys=['unlocked_counter_in'],
                             #output_keys=['unlocked_counter_out'])
        
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # function called when exiting from the node, it can be blccking
        self.client1.utils.load_ref_from_file('/home/awais/my_ros_ws/src/survailence_robot/map/topological_map_abox_new.owl', 
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

        self.client1.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'C1')
        
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

        res = self.client1.call('REASON', '', '', '')
        print("Response : ", res)

        if res.is_consistent == True:
            print("Successfully Received the Map\n")
            print("Replacing Visited At with Now()\n")
            now = self.client1.query.dataprop_b2_ind('now', 'Robot1')
            #print("Now Value of robot is: ", now)
            new_list = []
            for string in now:
                new_list.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', new_list)
            ret = self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), new_list[0])
            res = self.client1.call('REASON', '', '', '')
            if ret == True:
                print("Replaced")

            now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
            #print("New now Value of robot is: ", now1)
            #print("The VisitedAt of E is: ", now2)
            new_list1 = []
            for string in now1:
                new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

            now2 = self.client1.query.dataprop_b2_ind('visitedAt', 'E')
            #print("The current VisitedAt of E is: ", now2)
            new_list2 = []
            for string in now2:
                new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT is: ', new_list2)

            ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'E', 'Long', new_list1[0], new_list2[0])
            res = self.client1.call('REASON', '', '', '')
            if ret1 == True:
                print("Replaced Visited at of E with new Now")
            #rospy.sleep(5)
            #rospy.loginfo('Executing state UNLOCKED (users = %f)'%userdata.unlocked_counter_in)
            #userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
            #return user_action()
            return 'loaded'

# define state Locked
class Normal(smach.State):
    def __init__(self):
        #self._helper = helper
        smach.State.__init__(self, 
                             outcomes=['discharged','timeup','charged','loaded','relaxed'])
                            #input_keys=['locked_counter_in'],
                            # output_keys=['locked_counter_out'])
        #self.sensor_input = 0
        
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        return 'timeup'

# define state Locked
class Urgent(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['relaxed','discharged','timeup'])
                            # input_keys=['locked_counter_in'],
                            # output_keys=['locked_counter_out'])
        #self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        return 'relaxed'

# define state Locked
class Battery_Low(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['charged','discharged'])
                             #input_keys=['locked_counter_in'],
                             #output_keys=['locked_counter_out'])
        #self.sensor_input = 0
        self.client1 = ArmorClient('example', 'ontoRef')
        self.rate = rospy.Rate(200)  # Loop at 200 Hz


    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        return 'charged'

        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MAP_RECEIVING', Map_Receiving(),
                               transitions={'loaded':'NORMAL'})
                               #remapping={'locked_counter_in':'sm_counter', 
                               #           'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'timeup':'URGENT',
                                            'discharged':'BATTERY_LOW',
                                            'charged':'NORMAL', 
                                            'loaded':'NORMAL',
                                            'relaxed':'NORMAL'})
                               #remapping={'unlocked_counter_in':'sm_counter',
                               #           'unlocked_counter_out':'sm_counter'})
        smach.StateMachine.add('URGENT', Urgent(), 
                               transitions={'relaxed':'NORMAL',
                                            'discharged':'BATTERY_LOW',
                                            'timeup':'URGENT'})
                               #remapping={'locked_counter_in':'sm_counter', 
                               #           'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('BATTERY_LOW', Battery_Low(), 
                               transitions={'charged':'NORMAL',
                                            'discharged':'BATTERY_LOW'})
                               #remapping={'unlocked_counter_in':'sm_counter',
                               #           'unlocked_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

























#!/usr/bin/env python3

import roslib
import rospy
import smach
import smach_ros
import time
import random
import re
import sys

from arch_skeleton.helper import TopologicalMap
from armor_api.armor_client import ArmorClient


urgency = False
tm = None

zzz
def user_action():
    return random.choice(['discharged','timeup','charged','loaded','relaxed'])

# define state Unlocked
class Map_Receiving(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['loaded'])
                             #input_keys=['unlocked_counter_in'],
                             #output_keys=['unlocked_counter_out'])
        
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # function called when exiting from the node, it can be blccking
            global tm
            tm = TopologicalMap()

            print("Replacing Visited At with Now()\n")
            now = self.client1.query.dataprop_b2_ind('now', 'Robot1')
            #print("Now Value of robot is: ", now)
            new_list = []
            for string in now:
                new_list.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', new_list)
            ret = self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), new_list[0])
            res = self.client1.utils.sync_buffered_reasoner() 
            if ret == True:
                print("Replaced")

            now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
            #print("New now Value of robot is: ", now1)
            #print("The VisitedAt of E is: ", now2)
            new_list1 = []
            for string in now1:
                new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

            now2 = self.client1.query.dataprop_b2_ind('visitedAt', 'E')
            #print("The current VisitedAt of E is: ", now2)
            new_list2 = []
            for string in now2:
                new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT is: ', new_list2)

            ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'E', 'Long', new_list1[0], new_list2[0])
            res = self.client1.utils.sync_buffered_reasoner() 
            if ret1 == True:
                print("Replaced Visited at of E with new Now")
            #rospy.sleep(5)
            #rospy.loginfo('Executing state UNLOCKED (users = %f)'%userdata.unlocked_counter_in)
            #userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
            #return user_action()
            return 'loaded'

# define state Locked
class Normal(smach.State):
    def __init__(self):
        #self._helper = helper
        smach.State.__init__(self, 
                             outcomes=['discharged','timeup','charged','loaded','relaxed'])
                            #input_keys=['locked_counter_in'],
                            # output_keys=['locked_counter_out'])
        #self.sensor_input = 0
        
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        while not rospy.is_shutdown():
            list7 = self.client1.query.objectprop_b2_ind('isIn', 'Robot1')
            new_list = []
            for string in list7:
                new_list.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('I am in: ', new_list)
            
            list5 = self.client1.query.objectprop_b2_ind('canReach', 'Robot1')
            new_list1 = []
            for string in list5:
                new_list1.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('I can reach: ', new_list1)

            urgent = self.client1.query.ind_b2_class('URGENT')
            urgentList = []
            for string in urgent:
                urgentList.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('The current VisitedAT of Corrridor 1 is: ', urgentList)
            urgency = False
            for string in urgentList:
                if urgentList[i] == 'R1' or 'R2' or 'R3' or 'R4':
                    urgency = True
                    if new_list1[i] == 'R1' or 'R2' or 'R3' or 'R4':
                        print("Urgency Occured")
                        return 'timeup'
                else:
                    i += 1
            
            i = 0
            canReachC2 = False
            canReachC1 = False
            for string in new_list1:
                if new_list1[i] == 'C2':
                    canReachC2 = True
                elif new_list1[i] == 'C1':
                    canReachC1 = True
                else:
                    i += 1
            
            if new_list[0] == 'C1' and canReachC2 == True:
                print('I can reach the Corridor 1')
                self.client1.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C2', 'C1')
                print('I am in corridoor  2')
                now0 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                prevNow = []
                for string in now0:
                    prevNow.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', prevNow)
                self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), prevNow[0])
                now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                new_list1 = []
                for string in now1:
                    new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

                visitedC2At = self.client1.query.dataprop_b2_ind('visitedAt', 'C2')
                new_list2 = []
                for string in visitedC2At:
                    new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 2 is: ', new_list2)
                ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'C2', 'Long', new_list1[0], new_list2[0])
                print("Replaced", ret1)
                res = self.client1.call('REASON', '', '', '')
                time.sleep(1)
            
            elif new_list[0] == 'C2' and canReachC1 == True:
                print('The New List is: ', new_list)
                print('I can reach the Corridor 2')
                self.client1.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C1', 'C2')
                print('I am in Corridor 2')
                now0 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                prevNow = []
                for string in now0:
                    prevNow.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', prevNow)
                self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), prevNow[0])
                now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                new_list1 = []
                for string in now1:
                    new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

                visitedC1At = self.client1.query.dataprop_b2_ind('visitedAt', 'C1')
                new_list2 = []
                for string in visitedC1At:
                    new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 1 is: ', new_list2)
                ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'C1', 'Long', new_list1[0], new_list2[0])
                print("Replaced", ret1)
                res = self.client1.call('REASON', '', '', '')
                time.sleep(1)
            
                

            """"
            now1 = self.client1.query.dataprop_b2_ind('urgencyThreshold', 'Robot1')
            new_list = []
            for string in now1:
                new_list.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The Threshold List is: ', new_list)
            """

            res = self.client1.call('REASON', '', '', '')
            if res.is_consistent == True:
                print("Successfully Reasond\n")
            #if self.sensor_input < 5: 
                #rospy.loginfo('Executing state LOCKED (users = %f)'%userdata.locked_counter_in)
                #userdata.locked_counter_out = userdata.locked_counter_in + 1
            #    return user_action()
            #self.sensor_input += 1
            #self.rate.sleep

# define state Locked
class Urgent(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['relaxed','discharged','timeup'])
                            # input_keys=['locked_counter_in'],
                            # output_keys=['locked_counter_out'])
        #self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        while not rospy.is_shutdown():

            global tm
            tm = TopologicalMap()
            list7 = self.client1.query.objectprop_b2_ind('isIn', 'Robot1')
            new_list = []
            for string in list7:
                new_list.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('I am in: ', new_list)
            
            list5 = self.client1.query.objectprop_b2_ind('canReach', 'Robot1')
            new_list1 = []
            for string in list5:
                new_list1.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('I can reach: ', new_list1)

            i = 0
            canReachR1 = False
            canReachR24 = False
            canReachC1 = False
            for string in new_list1:
                if new_list1[i] == 'R1' or 'C1':
                    canReachR13 = True
                    canReachC1 = True
                else:
                    i += 1
            
            if new_list[0] == 'C1' and canReachR13 == True:
                print('I can reach the Room')
                self.client1.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'R1', 'C1')
                print('I am in Room 1')
                now0 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                prevNow = []
                for string in now0:
                    prevNow.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', prevNow)
                self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), prevNow[0])
                now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                new_list1 = []
                for string in now1:
                    new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

                visitedC2At = self.client1.query.dataprop_b2_ind('visitedAt', 'R1')
                new_list2 = []
                for string in visitedC2At:
                    new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 1 is: ', new_list2)
                ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'R1', 'Long', new_list1[0], new_list2[0])
                print("Replaced", ret1)
                res = self.client1.call('REASON', '', '', '')
                time.sleep(1)
            
            elif new_list[0] == 'R1' and canReachC1 == True:
                print('The New List is: ', new_list)
                print('I can reach the Corridor again')
                self.client1.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C1', 'R1')
                print('I am in Corridor 1 Again')
                now0 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                prevNow = []
                for string in now0:
                    prevNow.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', prevNow)
                self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), prevNow[0])
                now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                new_list1 = []
                for string in now1:
                    new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

                visitedC1At = self.client1.query.dataprop_b2_ind('visitedAt', 'C1')
                new_list2 = []
                for string in visitedC1At:
                    new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 1 is: ', new_list2)
                ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'C1', 'Long', new_list1[0], new_list2[0])
                print("Replaced", ret1)
                res = self.client1.call('REASON', '', '', '')
                time.sleep(1)
                #return 'relaxed'

# define state Locked
class Battery_Low(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['charged','discharged'])
                             #input_keys=['locked_counter_in'],
                             #output_keys=['locked_counter_out'])
        #self.sensor_input = 0
        self.client1 = ArmorClient('example', 'ontoRef')
        self.rate = rospy.Rate(200)  # Loop at 200 Hz


    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        while not rospy.is_shutdown():  
            list7 = self.client1.query.objectprop_b2_ind('isIn', 'Robot1')
            new_list = []
            for string in list7:
                new_list.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('I am in: ', new_list)
            
            list5 = self.client1.query.objectprop_b2_ind('canReach', 'Robot1')
            new_list1 = []
            for string in list5:
                new_list1.append(re.search('#' + '(.+?)'+'>', string).group(1))
            print('I can reach: ', new_list1)

            i = 0
            canReachR1 = False
            canReachR24 = False
            canReachC1 = False
            for string in new_list1:
                if new_list1[i] == 'R1':
                    canReachR13 = True
                elif new_list1[i] == 'C1':
                    canReachC1 == True
                    canReachR24 = True
                else:
                    i += 1
            
            if new_list[0] == 'C1' and canReachR13 == True:
                print('I can reach the Room')
                self.client1.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'R1', 'C1')
                print('I am in Room 1')
                now0 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                prevNow = []
                for string in now0:
                    prevNow.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', prevNow)
                self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), prevNow[0])
                now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                new_list1 = []
                for string in now1:
                    new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

                visitedC2At = self.client1.query.dataprop_b2_ind('visitedAt', 'R1')
                new_list2 = []
                for string in visitedC2At:
                    new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 2 is: ', new_list2)
                ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'R1', 'Long', new_list1[0], new_list2[0])
                print("Replaced", ret1)
                res = self.client1.call('REASON', '', '', '')
                time.sleep(1)
            
            elif new_list[0] == 'R1' and canReachC1 == True:
                print('The New List is: ', new_list)
                print('I can reach the Corridor again')
                self.client1.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C1', 'R1')
                print('I am in Corridor 1 Again')
                now0 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                prevNow = []
                for string in now0:
                    prevNow.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current Now value is: ', prevNow)
                self.client1.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), prevNow[0])
                now1 = self.client1.query.dataprop_b2_ind('now', 'Robot1')
                new_list1 = []
                for string in now1:
                    new_list1.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The new Now value is: ', new_list1)

                visitedC1At = self.client1.query.dataprop_b2_ind('visitedAt', 'C1')
                new_list2 = []
                for string in visitedC1At:
                    new_list2.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 1 is: ', new_list2)
                ret1 = self.client1.manipulation.replace_dataprop_b2_ind('visitedAt', 'C1', 'Long', new_list1[0], new_list2[0])
                print("Replaced", ret1)
                res = self.client1.call('REASON', '', '', '')
                time.sleep(1)

        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MAP_RECEIVING', Map_Receiving(),
                               transitions={'loaded':'NORMAL'})
                               #remapping={'locked_counter_in':'sm_counter', 
                               #           'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'timeup':'URGENT',
                                            'discharged':'BATTERY_LOW',
                                            'charged':'NORMAL', 
                                            'loaded':'NORMAL',
                                            'relaxed':'NORMAL'})
                               #remapping={'unlocked_counter_in':'sm_counter',
                               #           'unlocked_counter_out':'sm_counter'})
        smach.StateMachine.add('URGENT', Urgent(), 
                               transitions={'relaxed':'NORMAL',
                                            'discharged':'BATTERY_LOW',
                                            'timeup':'URGENT'})
                               #remapping={'locked_counter_in':'sm_counter', 
                               #           'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('BATTERY_LOW', Battery_Low(), 
                               transitions={'charged':'NORMAL',
                                            'discharged':'BATTERY_LOW'})
                               #remapping={'unlocked_counter_in':'sm_counter',
                               #           'unlocked_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()













def urgency():
    client = ArmorClient('example', 'ontoRef')
    client.utils.sync_buffered_reasoner()

    list5 =client.query.objectprop_b2_ind('canReach', 'Robot1')
    new_list1 = []
    for string in list5:
        new_list1.append(re.search('#' + '(.+?)'+'>', string).group(1))
    print('I can reach: ', new_list1)

    urgent = client.query.ind_b2_class('URGENT')
    urgentList = []
    i = 0
    for string in urgent:
        urgentList.append(re.search('#' + '(.+?)'+'>', string).group(1))
        print('The urgent list is: ', urgentList)
        for string in urgentList:
            if urgentList[i] == 'R1' or 'R2' or 'R3' or 'R4':
                if new_list1[i] == 'R1' or 'R2' or 'R3' or 'R4':
                    print("Urgency Occured")
                    return new_list1[i]
                else:
                    i += 1

def urgencycheck():
    client = ArmorClient('example', 'ontoRef')
    client.utils.sync_buffered_reasoner()
    urgent = client.query.ind_b2_class('URGENT')
    urgentList = []
    i = 0
    for string in urgent:
        urgentList.append(re.search('#' + '(.+?)'+'>', string).group(1))
        print('The urgent list is: ', urgentList)
        for string in urgentList:
            if urgentList[i] == 'R1' or 'R2' or 'R3' or 'R4':
                print("Urgency Occured")
                return True
            else:
                i += 1
