#include "multirov/resource_listener_hd.hpp"

resource_listener_hd::resource_listener_hd(ros::NodeHandle& nodeHandle){
}
  
bool resource_listener_hd::eequal(geometry_msgs::Point a, geometry_msgs::Point b){
  if((int)round(a.x*10) == (int)round(b.x*10) & (int)round(a.y*10) == (int)round(b.y*10) & (int)round(a.z*10) == (int)round(b.z*10)){
    return true; 
  }else{
    return false;
  }
}
void resource_listener_hd::subsCallback(const geometry_msgs::Point::ConstPtr& p){
  resource_location_point.x = (int)round(p->x); 
  resource_location_point.y = (int)round(p->y); 
  resource_location_point.z = (int)round(p->z); 
  resource_location_publ.publish(resource_location_point); 
  ROS_INFO("Command received, x = %f y = %f z = %f", resource_location_point.x, resource_location_point.y, resource_location_point.z); 
}
