#include<vector>
#include<string>
#include "math.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"

using namespace std;
using location_monitor::LandmarkDistance;

class Landmark{
public:
  Landmark(string name, double x, double y):name(name), x(x), y() {}
  string name;
  double x;
  double y;
};

class LocationMonitor{

public:
 LocationMonitor(): landmarks_(){
  InitLandmarks();
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){             
double  x = msg->pose.pose.position.x;                                    
double  y = msg->pose.pose.position.y;                                    
ROS_INFO("X: %f y: %f",x,y);                                            
}   

LandmarkDistance FindClosest(double x, double y){
LandmarkDistance result;
result.distance = -1;
for(int i=0; i<landmarks_.size(); i++){
double dx = landmarks_[i].x - x;
double dy= landmarks_[i].y - y;
double distance = sqrt(dx*dx-dy*dy);
if(result.distance<0 | distance < result.distance){
result.name = landmarks_[i].name;
result.distance = distance;
}
}
return result;
}

private:
  vector<Landmark> landmarks_;
void InitLandmarks()
{
  landmarks_.push_back(Landmark("cube",0.31,-0.99));
  landmarks_.push_back(Landmark("Dumpster",0.11,-2.42));
  landmarks_.push_back(Landmark("Cylinder",-1.14,-2.88));
  landmarks_.push_back(Landmark("Barrier",-2.59,-0.83));
  landmarks_.push_back(Landmark("Bookshelf",-0.09,0.53));
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
