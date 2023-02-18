.. EXPROB_Assignment_02 documentation master file, created by
   sphinx-quickstart on Sun Feb 12 04:00:20 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to EXPROB_Assignment_02's documentation!
================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Introduction
=============
This package is an experiment to use a topological map ontology for controling a robot using ROS.
The ontology consists of an indoor environment with multiple rooms and a mobile robot.

.. image:: diagrams/UML_Assignment_02.jpg
  :width: 300
  :align: center
  :alt: topological_map

The robot starts in room E and by scanning the provided markers, it receives the information to build the
semantic map, i.e., the name and center position of each room and the connections between them.

Once the semantic map is built, robot has to start moving among the rooms with the policy that each
room that has not been visited for a long time, would be selected as the target room. Everytime the robot
gets to the target room, it has to scan the room environment as it did for the first time with the markers.

When the robot battery is low, it goes to the charger which is placed in room E, 
sand wait for some times before to start again with the above behavior.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

This is the documentation of the EXPROB_Assignment_02 package!

Robot State Module
=====================
.. automodule:: scripts.robot_states
	:members:

Finite State Machine Module
=============================
.. automodule:: scripts.state_machine
	:members:
	
My Moveit Module
=====================
.. doxyfile:: mymoveit.cpp
	:project: EXPROB_Assignment_02
	
	


