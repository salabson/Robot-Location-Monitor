#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


class LocationMonitor{

public:
 LocationMonitor(){}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){             
float x = msg->pose.pose.position.x;                                    
float y = msg->pose.pose.position.y;                                    
ROS_INFO("X: %f y: %f",x,y);                                            
}   

 };


int main(int argc, char **argv){
// Initialize node
ros::init(argc, argv, "location_monitor");
ros::NodeHandle nh;

// creat subscriber
LocationMonitor monitor;
ros::Subscriber location_sub = nh.subscribe("odom",10, &LocationMonitor::OdomCallback, &monitor);

// continue checking for call back
ros::spin();
return 0;

}
