#include <ros/ros.h>
#include "multirov/resource_heatmap.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "resource_heatmap_node");
  ros::NodeHandle nh;
  resource_heatmap resourceheatmap(nh);
  ros::spin();
  return 0;
}
