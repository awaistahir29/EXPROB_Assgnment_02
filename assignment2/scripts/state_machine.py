#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
import random
import re

from assignment2.helper import TopologicalMap
from armor_api.armor_client import ArmorClient
from assignment2.srv import GetBattery, SetBattery
from assignment2 import architecture_name_mapper as anm
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment2.msg import Point
from assignment2.srv import RoomInformation
from std_msgs.msg import Int32
from assignment2.srv import Marker
import actionlib

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
tm = None
stayinroomtime = 0.5
urgentflag = 1
sleeptime =2
batflag = 1
get_battery_level = {}
newLevel = 0
resp = 0

client_mvbs = actionlib.SimpleActionClient('move_base',MoveBaseAction)

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
    """
    Getting the Battery Level Service
    """
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
    """
    Cutting the Battery Level Service
    """
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
    """
    Detects the markers by sending requests after appropriate time
    Args:
        Boolean: True 
    Returns:
        Boolean: True
    """
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
        global tm
        tm = TopologicalMap()
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
## The Normal class.The robots keeps moving between C1 and C2 for infinit time, the robots keep cheking the 
# state of the battery, if the battery is low the robot goes to CHARGING state
# trough the transition tired. if the  battery is not low, the robot cheks if there is an urgent room by quering the
# individuals of the class urgent. If there is an urgent room the robot goes to URGENT state through the transition
# timeup.
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['discharged','timeup','charged','loaded','relaxed'])
                            
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
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
                client_mvbs.wait_for_server()
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = -1.5
                goal.target_pose.pose.position.y = 0.0
                goal.target_pose.pose.orientation.w = 1.0
                result = client_mvbs.send_goal_and_wait(goal)
                set_arm_movement_state(True)
                rospy.sleep(stayinroomtime)
            else:
                moveto('C2')
                client_mvbs.wait_for_server()
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = 3.5
                goal.target_pose.pose.position.y = 0.0
                goal.target_pose.pose.orientation.w = 1.0
                result = client_mvbs.send_goal_and_wait(goal)
                set_arm_movement_state(True)
                rospy.sleep(stayinroomtime)
            return 'charged'

# define state Locked
class Urgent(smach.State):
## The visting_urgent class.First, the robots checks if the battery is low goes to CHARGING state, else it checkes it the
# urgent room is reacheable by quering the object properties of the robot canReach, if the room can be reached it visit it
# and return back to the  MOVING_IN_CORRIDORS trough the transition visited. If the urgent room is not reacheable it goes 
# to MOVING_IN_CORRIDORS state through the transition not_reached, in this case the robot will change the corridors and 
# visit it again.
    def __init__(self):
        """! A method of the class visiting_urgent
           @param userdata
           @return  one of its outcomes
        """
        smach.State.__init__(self, 
                             outcomes=['relaxed','discharged','timeup'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        self.client1 = ArmorClient('example', 'ontoRef')

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        """
        Implements the execution of the tasks while this state gets active.
        """
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
            if The_urgnet_room == 'R1':
                client_mvbs.wait_for_server()
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = -7.0
                goal.target_pose.pose.position.y = 3.0
                goal.target_pose.pose.orientation.w = 1.0
                result = client_mvbs.send_goal_and_wait(goal)
                set_arm_movement_state(True)
            elif The_urgnet_room == 'R2':
                client_mvbs.wait_for_server()
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = -7.0
                goal.target_pose.pose.position.y = -4.0
                goal.target_pose.pose.orientation.w = 1.0
                result = client_mvbs.send_goal_and_wait(goal)
                set_arm_movement_state(True)
            elif The_urgnet_room == 'R3':
                client_mvbs.wait_for_server()
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = 9.0
                goal.target_pose.pose.position.y = 3.0
                goal.target_pose.pose.orientation.w = 1.0
                result = client_mvbs.send_goal_and_wait(goal)
                set_arm_movement_state(True)
            elif The_urgnet_room == 'R4':
                client_mvbs.wait_for_server()
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = 9.0
                goal.target_pose.pose.position.y = -4.0
                goal.target_pose.pose.orientation.w = 1.0
                result = client_mvbs.send_goal_and_wait(goal)
                set_arm_movement_state(True)

            rospy.sleep(stayinroomtime)
            return 'timeup'

# define state Locked
class Battery_Low(smach.State):
    """
    Defines the state when robot has reached the charger and chargers battery after some time using
    ``set_battery_level(battery_level)`` function and then returns ``charged`` transition.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['charged','discharged'])

        self.client1 = ArmorClient('example', 'ontoRef')
        self.rate = rospy.Rate(200)  # Loop at 200 Hz


    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        # simulate that we have to get 5 data samples to compute the outcome
        moveto('E')
        client_mvbs.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1.5
        goal.target_pose.pose.position.y = 8.0
        goal.target_pose.pose.orientation.w = 1.0
        result = client_mvbs.send_goal_and_wait(goal)   
        _set_battery_level_client(20)
        log_msg = f'Battery Charged.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        time.sleep(LOOP_TIME)
        return 'charged'

        
def main():
    """
    The main function for finite_state_machine node, initialises the node defines the subscriner to the ``/image_id`` topic
    , defines the states and transitions of the finite state machine for topological map and finally 
    starts the finite state machine process
    """
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