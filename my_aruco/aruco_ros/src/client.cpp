#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <assignment2/RoomInformation.h>
#include <assignment2/RoomConnection.h>
#include <std_msgs/Int32.h>
#include <aruco_msgs/id.h>

class MarkerClient
{
    private:
        std::vector<std::uint32_t> roomID;
        ros::Subscriber sub;
        ros::ServiceClient client;
    
    
    public:
        MarkerClient()
        {
            ros::NodeHandle nh;
            sub = nh.subscribe("/id", 1, &MarkerClient::marker_callback, this);
            client = nh.serviceClient<assignment2::RoomInformation>("room_info");
        }

        void marker_callback(const aruco_msgs::id::ConstPtr& msg)
        {
            if(!std::count(roomID.begin(), roomID.end(), msg->id) && msg->id > 10 && msg->id <18 )
            {
                roomID.push_back(msg->id);
                ROS_INFO("Image ID Detected: [%d]", msg->id);
                assignment2::RoomInformation srv;
                srv.request.id = msg->id;
                if(client.call(srv))
                {
                    ROS_INFO_STREAM("Room "<< srv.response.room << "detected");
                    ROS_INFO("Centre Position is: [%f, %f]", srv.response.x, srv.response.y);
                    for(int i = 0; i < srv.response.connections.size(); i++)
                    {
                        ROS_INFO_STREAM("Room" << srv.response.room << "is connected to "<< srv.response.connections[i].connected_to << " through door "<<
                        srv.response.connections[i].through_door);
                    }
                }
                else{
                    ROS_ERROR("Failed to call the service room_info");
                }
            }
            
        }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_client");

  MarkerClient node;  

  ros::spin();
  ros::shutdown();
  return 0;
}
