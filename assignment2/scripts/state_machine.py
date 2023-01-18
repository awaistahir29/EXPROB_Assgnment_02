#!/usr/bin/env python3

import roslib
import rospy
import smach
import smach_ros
import time
import random
import re
import sys
import random

from survailence_robot.helper import TopologicalMap
from armor_api.armor_client import ArmorClient

from survailence_robot.srv import GetBattery, SetBattery
from survailence_robot import architecture_name_mapper as anm

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 2

#urgency = False
tm = None
stayinroomtime = 0.5
urgentflag = 1
sleeptime =2
batflag = 1
get_battery_level = {}
newLevel = 0
resp = 0

def _set_battery_level_client(battery_level):
    """
    Service client function for ``/state/set_battery_level`` Update the current robot battery level
    stored in the ``robot-state`` node
    Args:
        battery_level(int)
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_BATTERY)
    try:
        # Log service call.
        log_msg = f'Set current robot battery level to the `{anm.SERVER_SET_BATTERY}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_BATTERY, SetBattery)
        service(battery_level)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def _get_battery_level_client():
    global get_battery_level
    global resp
    rospy.wait_for_service(anm.SERVER_GET_BATTERY)
    try:
        # Log service call.
        log_msg = f'Get current robot battery level to the `{anm.SERVER_GET_BATTERY}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_BATTERY, GetBattery)
        resp = service()  # The `response` is not used.
        return resp
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def cutBattery():
    global newLevel
    global resp
    resp = _get_battery_level_client()
    print(resp)
    newLevel = resp.level - 1 
    _set_battery_level_client(newLevel)

def findindividual(list):
    """
    Function for finding the individual in a list from the return of a qureied proprity from armor.  
    Args:
        Individual(list): The individual in the armor resonse format ex. *['<http://bnc/exp-rob-lab/2022-23#R1>']*  
    Returns:
        Individual(string): The individual extarcted and changed to a string *ex. "R1"*
    """
    for i in list:
        if "R1" in i:
            return 'R1'
        elif "R2" in i:
            return 'R2'
        elif "R3" in i:
            return 'R3'
        elif "R4" in i:
            return 'R4'
        elif "C1" in i:
            return 'C1'
        elif "C2" in i:
            return 'C2'
        elif "E" in i:
            return 'E'

def moveto(location):
    client = ArmorClient('example', 'ontoRef')
    client.utils.sync_buffered_reasoner()

    is_In = client.query.objectprop_b2_ind('isIn', 'Robot1')
    oldlocation=findindividual(is_In)
    #can_Reach=client.query.objectprop_b2_ind('canReach', 'Robot1')
    #reachable_location=findindividual(can_Reach)
    #client.query.objectprop_b2_ind('canReach', 'Robot1')
    list5 = client.query.objectprop_b2_ind('canReach', 'Robot1')
    new_list1 = []
    for string in list5:
        new_list1.append(re.search('#' + '(.+?)'+'>', string).group(1))
    print('I can reach: ', new_list1)
    print('The desired Location is: ',location)
    i = 0
    for string in new_list1:
        if new_list1[i] == location:
            if oldlocation== 'R1' or oldlocation == 'R2' or oldlocation == 'E' or oldlocation == 'C2': 
                print("Moving from: " + oldlocation, "to: " + location)
                client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', location, oldlocation)
                break
            elif oldlocation == 'R3' or oldlocation == 'R4' or oldlocation == 'E' or oldlocation == 'C1':
                print("Moving from: " + oldlocation, "to: " + location)
                client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', location, oldlocation)
                break
            elif oldlocation == 'C1':
                print("Moving from: " + oldlocation, "to: " + location)
                client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', location, oldlocation)
                break
            elif oldlocation == 'C2':
                print("Moving from: " + oldlocation, "to: " + location)
                client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', location, oldlocation)
                break
            else:
                print("Not Moved")

        else:
            i += 1
    i = 0
    client.utils.sync_buffered_reasoner()

    if location == 'R1' or location == 'R2' or location == 'R3' or location == 'R4':
        old_time = client.query.dataprop_b2_ind('now', 'Robot1')
        robot_Now = []
        for string in old_time:
            robot_Now.append(re.search('"' + '(.+?)'+'"', string).group(1))
            print('The current Now value is: ', robot_Now)
            client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), robot_Now[0])
            client.utils.sync_buffered_reasoner() 
            oldVisitedAt = client.query.dataprop_b2_ind('visitedAt', location)
            VisitedAt = []
            for string in oldVisitedAt:
                VisitedAt.append(re.search('"' + '(.+?)'+'"', string).group(1))
                print('The current VisitedAT of Corrridor 2 is: ', VisitedAt)
                ret1 = client.manipulation.replace_dataprop_b2_ind('visitedAt', location, 'Long', robot_Now[0], VisitedAt[0])
                print("Replaced", ret1)
                client.utils.sync_buffered_reasoner()


def urgentupdate():
    """
    Function for checking if there is an urgent room to set the global *urgentflag*, also returns the nearby urgent room.  
    Args:
        void  
    Returns:
        Urgent room(string): The nearby urgent room according to the robot position in the corridors.
    """
    global urgentflag
    tobetrturned = '0'
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    req=client.call('QUERY','IND','CLASS',['URGENT'])
    req2=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req2.queried_objects)
    for i in req.queried_objects:
        if oldlocation=='E':
            if random.randint(1, 2)==1:
                moveto('C1')
            else:
                moveto('C2')
            client.call('REASON','','',[''])
            req2=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            oldlocation=findindividual(req2.queried_objects)
        if oldlocation == 'C1':
            if "R1" in i:
                urgentflag = 0
                tobetrturned = 'R1'
                break
            elif "R2" in i:
                urgentflag = 0
                tobetrturned = 'R2'
                break
        elif oldlocation == 'C2':
            if "R3" in i:
                urgentflag = 0
                tobetrturned = 'R3'
                break
            elif "R4" in i:
                urgentflag = 0
                tobetrturned = 'R4'
                break
    if  tobetrturned == '0':
        urgentflag = 1
    else:
        return tobetrturned

def urgency():
    client = ArmorClient('example', 'ontoRef')
    client.call('REASON','','',[''])

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
    client.call('REASON','','',[''])
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
            
            moveto('C1')
            time.sleep(1)
            moveto('R1')
            time.sleep(1)
            moveto('C1')
           # """"
            time.sleep(1)
            moveto('R2')
            time.sleep(1)
            moveto('C1')
            time.sleep(1)
            moveto('C2')
            time.sleep(1)
            moveto('R3')
            time.sleep(1)
            moveto('C2')
            time.sleep(1)
            moveto('R4')
            time.sleep(1)
            moveto('C2')
            time.sleep(1)
#"""

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
            global urgentflag
            global newLevel
            cutBattery()
            urgentupdate()
            print('The Remaining Battery is: ', newLevel)
            if newLevel <= 5:
                return 'discharged'
            if urgentflag == 0:
                print("Urgency Occured")
                return 'timeup'
            else:
                if random.randint(1, 2)==1:
                    moveto('C1')
                    rospy.sleep(stayinroomtime)
                else:
                    moveto('C2')
                    rospy.sleep(stayinroomtime)
                return 'charged'

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
        global batflag
        global urgentflag
        global newLevel
        cutBattery()
        print('The Remaining Battery is: ', newLevel)
        if newLevel <= 5:
            return 'discharged'
        client = ArmorClient("example", "ontoRef")
        urgentupdate()
        rospy.sleep(sleeptime)
        if urgentflag == 1:
            return 'relaxed'
        else:
            The_urgnet_room=urgentupdate()
            moveto(The_urgnet_room)
            rospy.sleep(stayinroomtime)
            return 'timeup'

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
        moveto('E')
        _set_battery_level_client(20)
        log_msg = f'Battery Charged.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        time.sleep(LOOP_TIME)
        return 'charged'

        
def main():
    rospy.init_node('finite_state_machine')

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


    #rospy.Subscriber("batterylevel", Bool, callbackbattery)
    #rospy.ServiceClient(anm.SERVER_GET_BATTERY, GetBattery)
    service = rospy.ServiceProxy(anm.SERVER_GET_BATTERY, GetBattery)


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