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
  ros::init(argc, argv, "agent_listener");

  ros::NodeHandle node;

  std::string ns; 
  if(argc != 2){
    ROS_WARN("Warning! Didn't pass correct namespace format");
  }else{
    ns = argv[1];
  }

int num; 
char* cstr = &ns[0u];
sscanf(cstr, "/rexrov%d", &num); // must be char[], not string
printf("%d\n", num);

  std::string pre(ns); 

  ns.append("/base_stabilized");
  pre.append("/agent_location");
  
  //if(node.getParam("namespace", ns)){
  //  ROS_INFO("Get namespace");
  //} else{
  //  ROS_ERROR("Can't get namespace");
  //}

  ros::Publisher agent_location_publ = node.advertise<geometry_msgs::Point>(pre, 10); 
  tf::TransformListener listener;
  
  
  std::cout << ns << std::endl; 
  std::cout << pre << std::endl;
  ros::Rate rate(1.0);
  geometry_msgs::Point agent_location_point_previousloop;
  agent_location_point_previousloop.x = -1; 
  agent_location_point_previousloop.y = -1;
  agent_location_point_previousloop.z = 10;
  while (node.ok()){
    tf::StampedTransform transform;
    try{ 
      listener.waitForTransform("/world", ns, ros::Time(0), ros::Duration(3.0) );
      listener.lookupTransform("/world", ns, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Point agent_location_point; 
    agent_location_point.x = transform.getOrigin().x(); 
    agent_location_point.y = transform.getOrigin().y(); 
    agent_location_point.z = (float)num; /////////////////////////////////////// Mark which rov gives the info, as z is not used anyway 
    //turtle_vel.publish(vel_msg);
    //if(not eequal(agent_location_point_previousloop, agent_location_point)){
      agent_location_publ.publish(agent_location_point); 
      //ROS_INFO("x = %f y = %f z = %f", agent_location_point.x, agent_location_point.y, agent_location_point.z); 
    //  agent_location_point_previousloop = agent_location_point; 
    //}
    rate.sleep();
  }
  return 0;
};
