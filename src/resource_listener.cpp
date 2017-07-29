#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>

bool eequal(geometry_msgs::Point a, geometry_msgs::Point b){
  if((int)round(a.x*10) == (int)round(b.x*10) & (int)round(a.y*10) == (int)round(b.y*10) & (int)round(a.z*10) == (int)round(b.z*10)){
    return true; 
  }else{
    return false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  //ros::Publisher turtle_vel =
 //   node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  ros::Publisher resource_location_publ = node.advertise<geometry_msgs::Point>("/resource_location", 10); 
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  geometry_msgs::Point resource_location_point_previousloop;
  resource_location_point_previousloop.x = -1; 
  resource_location_point_previousloop.y = -1;
  resource_location_point_previousloop.z = 10;
  while (node.ok()){
    tf::StampedTransform transform;
    try{ 
      listener.waitForTransform("/world", "/rexrov0/base_stabilized", ros::Time(0), ros::Duration(3.0) );
      listener.lookupTransform("/world", "/rexrov0/base_stabilized", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
/*
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
*/

    geometry_msgs::Point resource_location_point; 
    resource_location_point.x = transform.getOrigin().x(); 
    resource_location_point.y = transform.getOrigin().y(); 
    resource_location_point.z = transform.getOrigin().z(); 
    //turtle_vel.publish(vel_msg);
    if(not eequal(resource_location_point_previousloop, resource_location_point)){
      resource_location_publ.publish(resource_location_point); 
      ROS_INFO("x = %f y = %f z = %f", resource_location_point.x, resource_location_point.y, resource_location_point.z); 
      resource_location_point_previousloop = resource_location_point; 
    }
    rate.sleep();
  }
  return 0;
};
