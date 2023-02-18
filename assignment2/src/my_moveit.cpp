/**
 * @file my_moveit.cpp
 * @brief Implements a node to control the arm of robot using moveit
 * 
 * 
 * @details
 *
 * Services: <BR>
 *  /move_arm
 *
 * Description:
 * 
 * There are 7 markers in the environment, the joints configuration for each marker is found and once
 * the ``move_arm`` service is called with arg true, ``reach(req, resp)`` function tries to find the 
 * plan to the corresponding joints configuration and executes it.
**/
#include <ros/ros.h>
//! MoveIt headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
//#include "patrol_robot/MarkerRoutine.h"
#include <assignment2/Marker.h>

//! Global variable to store the current position of the robot arm
int position=0;

/**
 * @brief Service callback function for the move_arm service.
 *
 * @param req Service request message. Contains the position to move the arm to.
 * @param resp Service response message. Contains the name of the target position.
 *
 * @return True if the service call was successful.
 */

bool reach(assignment2::Marker::Request &req, assignment2::Marker::Response &resp){
  
  //! Check the requested position and set the position variable and response message accordingly

  if(req.pos == 1){
    //! Set position to 1 and fill out the response message with the name of the target position
  	position=1;
  	resp.message = "front";
  	}
  else if (req.pos == 2){ 
    //! Set position to 2 and fill out the response message with the name of the target position
  	position=2;
  	resp.message = "up_right";
  	}
  else if (req.pos == 3){ 
    //! Set position to 3 and fill out the response message with the name of the target position
    position=3;
    resp.message = "down_right";
    }
  else if (req.pos == 4){
    //! Set position to 4 and fill out the response message with the name of the target position 
    position=4;
    resp.message = "back_left";
    }
  else if (req.pos == 5){ 
    //! Set position to 5 and fill out the response message with the name of the target position
    position=5;
    resp.message = "initial";
    }
  else{ 
     //! Return false if the requested position is not recognized
    return false; 
  }
  return true;
  
}


int main(int argc, char** argv)
{
  //! Initialize ROS and the node
  ros::init(argc, argv, "robot_model_and_robot_state");
  ros::NodeHandle nh;

  //! Advertise the move_arm service
  ros::ServiceServer service = nh.advertiseService("move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //! Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //! Set all joint values to their default values
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  //! Set the starting state to the current state of the robot
  group.setStartStateToCurrentState();
  // Set the tolerance for orientation and position goals
  group.setGoalOrientationTolerance(1);
  group.setGoalPositionTolerance(1);
  
  //! Main loop
  while(ros::ok()){
  	 if (position==1){
      //! Move the arm to the "MR_left" position
  	  group.setNamedTarget("front");
  	  group.move();
      //! Reset the position variable
  	  position=0;
      }
  	 if (position==2){
      //! Move the arm to the "home" position
  	  group.setNamedTarget("up_right");
  	  group.move();
      //! Reset the position variable
  	  position=0;
      }
     if (position==3){
      //! Move the arm to the "MR_right" position
      group.setNamedTarget("down_right");
      group.move();
      position=0;
      }
     if (position==4){
      //! Move the arm to the "MR_back" position
      group.setNamedTarget("back_left");
      group.move();
      //! Reset the position variable
      position=0;
      }
     if (position==5){
      //! Move the arm to the "stop" position
      group.setNamedTarget("initial");
      group.move();
      //! Reset the position variable
      position=0;
      }

  }
  return(0);
}
