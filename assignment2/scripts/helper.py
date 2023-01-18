import sys
import roslib
import rospy
import actionlib
import smach
import smach_ros
import random
import time
import re
from ASSIGMENT_01 import name_mapper as nm
from survailence_robot.msg import Point, PlanAction, PlanGoal, ControlAction
from armor_api.armor_client import ArmorClient

from threading import Thread
from threading import Lock

class Helper:
	def __init__(self, done_callback = None, feedback_callback = None, mutex = None):
            if mutex is None:
                self.mutex = Lock()
            else:
                    self.mutex = mutex
            self.action_for_change = ''
            self.robot_timestamp_value = -1
		
		# client for the arMOR server
            self.client = ArmorClient('armor_client', "reference")

        def reason_changes(self):
		"""
		Function to apply the modifications to the ontology.
		It uses the aRMOR client to do this and it is called every time the program changes something related to the ontology entities (timestamps, robot position etc ...)
		
		Args:
			none
		
		Returns:
			none
		"""
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		
	def format(self, oldlist, start, end):
		"""
		Function to format a list of strings.
		For all the elements in the list, it takes two character, a start and a finish one. In each element i-th it is looked for these two characters and it is taken just the portion of the string between them.
		The return is a new list with the elements "cleaned" by not useful characters.
		
		Args:
			oldlist(List): the list that has to be re-written in a better way
			start(Char): the start character for the list member cut
			end(Char): the end characted for the list memeber cut
		
		Returns:
			newList(List): the new list with all the elements written in a proper way
		"""
		newlist = []
		for string in oldlist:
			newlist.append(re.search(start + '(.+?)' + end, string).group(1))
		return newlist

	def _robot_timestamp_value(self):
		"""
		Function to retrieve the timestamp of the robot.
		It is used the aRMOR server with a query and the timestamp is returned as output.
		
		Args:
			none
			
		Returns:
			_timestamp(String): timestamp of the robot casted as a string
		"""
		_timestamp = self.client.query.dataprop_b2_ind('now', 'Robot1')
		return str(self.format(_timestamp, '"', '"')[0])
				
	def _location_old_timestamp(self, _location):
		"""
		Function to retrieve the timestamp of the lcoation.
		It is used the aRMOR server with a query and the timestamp is returned as output.
		The output represents the last time the location was visited by the robot.
		
		Args:
			_location(String): name of the location the timestamp is required for
			
		Returns:
			_timestamp(String): timestamp of the location casted as a string
		"""
		_timestamp = self.client.query.dataprop_b2_ind('visitedAt', _location)
		return str(self.format(_timestamp, '"', '"')[0])

	def update_timestamp(self):
		"""
		Function to update the timestamp of an entity in the ontology.
		It is replaced the timestamp of the robot with the acutal time.
		This timestamp is then used to replace the location timestamp to set where the robot is at the actual time (the actual robot timestamp is now taken from the private method since it has just been updated)
		
		Args:
			none
			
		Returns:
			none
		"""
		self.client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), self._robot_timestamp_value())
		self.client.manipulation.replace_dataprop_b2_ind('visitedAt', str(self.choice), 'Long', self._robot_timestamp_value(), self._location_old_timestamp(str(self.choice)))
					