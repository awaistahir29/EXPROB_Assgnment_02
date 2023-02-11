#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
import random
import re

from survailence_robot.helper import TopologicalMap
from armor_api.armor_client import ArmorClient

from survailence_robot.srv import GetBattery, SetBattery
from survailence_robot import architecture_name_mapper as anm

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment2.msg import Point
from assignment2.srv import RoomInformation
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from aruco_msgs.msg import id
#include <assignment2/Marker.h>
from assignment2.srv import Marker

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 2


pub = None
mutex = None
rooms_id = []
rooms_name = []
rooms_center = []
rooms_connections = []
rooms_doors = []
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
        print('The urgent lisdatat is: ', urgentList)
        for string in urgentList:
            if urgentList[i] == 'R1' or 'R2' or 'R3' or 'R4':
                print("Urgency Occured")
                return True
            else:
                i += 1


def user_action():
    return random.choice(['discharged','timeup','charged','loaded','relaxed'])

def get_room_info(room_id):
    """
        Server client for ``marker_server``, gets information for each room using ``room_info`` service
        Args: 
            room_id(int)
        Returns:
            resp(RoomInformationResponse)
    """
    rospy.wait_for_service('room_info')
    try:
        srv = rospy.ServiceProxy('room_info', RoomInformation)
        resp = srv(room_id)
        return resp 
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def marker_id_callback(data):
    """
        Callback function for ``/image_id`` topic subscriber. Eveytime an image id is detected, checks 
        if image id is valuable and not already available, then saves the corresponding information of
        each room in the global variables by calling ``get_room_info(room_id)`` function, and modifies
        the ontology using ``add_room(room)``, ``add_door(door)``, ``assign_doors_to_room(room, doors)``
        ``disjoint_individuals()`` and ``add_last_visit_time(room, visit_time)`` functions from 
        ``topological_map.py`` helper script.
        Args:
            data(int32)
    """
    global rooms_id
    global rooms_name
    global rooms_center
    global rooms_connections
    global rooms_doors
    if data.data not in rooms_id and data.data > 10 and data.data < 18:
        rooms_id.append(data.data)
        log_msg = 'Image id detected: %d ' % (data.data)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        log_msg = 'Number of detected IDs: %d ' % (len(rooms_id))
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        
        room_info = get_room_info(data.data)
        rooms_name.append(room_info.room)
        log_msg = 'Semantic map updated, room '+ room_info.room + ' detected'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        #tm.add_room(room_info.room)

        rooms_center.append([room_info.x, room_info.y])
        log_msg = 'Center position is: [%f, %f]' % (room_info.x, room_info.y)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        for i in range(len(room_info.connections)):
            rooms_connections.append(room_info.connections[i].connected_to)
            rooms_doors.append(room_info.connections[i].through_door)
            log_msg = 'Room ' + room_info.room + ' is connected to ' + room_info.connections[i].connected_to + ' through door ' + room_info.connections[i].through_door
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            #tm.add_door(room_info.connections[i].through_door)
            #tm.assign_doors_to_room(room_info.room, room_info.connections[i].through_door)

        #tm.disjoint_individuals()
        #tm.add_last_visit_time(room_info.room, str(room_info.visit_time))



def move_to_pose(pose):
    """
        Action client function for ``move_base`` node, gets a pose as an argument and sends it as 
        ``MoveBaseGoal.msg`` to the action server
        Args:
            pose(Point)
        
        Returns:
            result(MoveBaseResult.msg)
    """
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.x
    goal.target_pose.pose.position.y = pose.y
    goal.target_pose.pose.orientation.w = 1.0
    client.wait_for_server()
    client.send_goal(goal)

def get_room_pose(room):
    """
    Detects the center postion by using the room information for corresponding room
    Args:
        room(string)
    Returns:
        room_pose(Point)
    """
    global rooms_name
    global rooms_center
    room_pose = Point()
    room_index = rooms_name.index(room)
    room_pose.x = rooms_center[room_index][0]
    room_pose.y = rooms_center[room_index][1]
    return room_pose

def set_arm_movement_state(arm_movement_state):
    rospy.wait_for_service('/move_arm')
    try:
        log_msg = 'Setting robot arm movement state to ' + str(arm_movement_state)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy('move_arm', Marker)
        resp = service(1)
        rospy.sleep(10)
        if resp.message == 'front':
            print("FRONT MOVEMENT SCCCEED\n")
            resp = service(2)
            rospy.sleep(10)
            if resp.message == 'up_right':
                print("UP_RIGHT MOVEMENT SCCCEED\n")
                resp = service(3)
                rospy.sleep(10)
                if resp.message == 'down_right':
                    print("DOWN_RIGHT MOVEMENT SCCCEED\n")
                    resp = service(4)
                    rospy.sleep(25)
                    if resp.message == 'back_left':
                        print("BACK_LEFT MOVEMENT SCCCEED\n")
                        resp = service(5)
                        rospy.sleep(10)
                        if resp.message == 'initial':
                            print("Initial MOVEMENT SCCCEED\n")
                            rospy.sleep(5)
                            return True
            

    except rospy.ServiceException as e:
        log_msg = f'Server can not set current arm movement state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def set_base_movement_state(base_movement_state):
    """
        Service client function for ``/base_movement_state``. Updates the current robot base movement state stored 
        in ``robot-states`` node
        Args:   
            base_movement_state(bool)
    """
    rospy.wait_for_service(anm.SERVER_SET_BASE_MOVEMENT_STATE)
    try:
        log_msg = 'Setting robot base movement state to ' + str(base_movement_state)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy(anm.SERVER_SET_BASE_MOVEMENT_STATE, SetBaseMovementState)
        service(base_movement_state)  
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current base movement state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


# define state Unlocked
class Map_Receiving(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['loaded'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global rooms_id
        check =  set_arm_movement_state(True)
        if(check == True):
            while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
                mutex.acquire()
                try:
                    print("The number of rooms are", len(rooms_id)) 
                    if len(rooms_id) > 6:
                        return 'loaded'
                finally:
                    mutex.release()
                rospy.sleep(LOOP_TIME)

# define state Locked
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['discharged','timeup','charged','loaded','relaxed'])
                            
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        [target_room, battery_low] = tm.update_ontology(now)
        target_room_pose = get_room_pose(target_room)
        set_base_movement_state(True)
        move_to_pose(target_room_pose)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                [next_target_room, battery_low] = tm.update_ontology(now)
                log_msg = 'target room: ' + target_room
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                if battery_low:
                    return 'discharged'
                else:
                    target_reached = check_target_reached(target_room_pose)
                    if target_reached:
                        set_base_movement_state(False)
                        return 'timeup'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class ExploreRoom(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['explored'])
        
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # function called when exiting from the node, it can be blccking
            global mutex
            global tm
            
            while not rospy.is_shutdown():
                mutex.acquire()
                try:
                    #set_arm_movement_state(True)
                    return 'explored'
                finally:
                    mutex.release()




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
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        [target_room, battery_low] = tm.update_ontology(now)
        target_room_pose = get_room_pose(target_room)
        set_base_movement_state(True)
        move_to_pose(target_room_pose)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                [next_target_room, battery_low] = tm.update_ontology(now)
                log_msg = 'target room: ' + target_room
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                if battery_low:
                    return 'discharged'
                else:
                    #target_reached = check_target_reached(target_room_pose)
                    #if target_reached:
                    #    set_base_movement_state(False)
                    return 'relaxed'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

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
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        tm.update_ontology(now)
        target_room_pose = get_room_pose('E')
        set_base_movement_state(True)
        move_to_pose(target_room_pose)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                tm.update_ontology(now)
                target_reached = check_target_reached(target_room_pose)
                if target_reached:
                    set_base_movement_state(False)
                    return 'target_reached'              

            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

        
def main():
    rospy.init_node('finite_state_machine')

    global mutex
    # Get or create a new mutex.
    if mutex is None:
        mutex = Lock()
    else:
        mutex = mutex

    # Subscribe image id to get rooms information
    rospy.Subscriber('/image_id', Int32, marker_id_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MAP_RECEIVING', Map_Receiving(),
                               transitions={'loaded':'NORMAL'})
        smach.StateMachine.add('ExploreRoom', ExploreRoom(),
                               transitions={'explored':'NORMAL'})
                               
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'timeup':'URGENT',
                                            'discharged':'BATTERY_LOW',
                                            'charged':'NORMAL', 
                                            'loaded':'NORMAL',
                                            'relaxed':'NORMAL'})
        smach.StateMachine.add('URGENT', Urgent(), 
                               transitions={'relaxed':'NORMAL',
                                            'discharged':'BATTERY_LOW',
                                            'timeup':'URGENT'})
        smach.StateMachine.add('BATTERY_LOW', Battery_Low(), 
                               transitions={'charged':'NORMAL',
                                            'discharged':'BATTERY_LOW'})


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