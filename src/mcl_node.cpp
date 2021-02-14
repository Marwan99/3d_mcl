#include <ros/ros.h>
#include <mcl/mcl.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "home_made_mcl");
  ros::NodeHandle nh;

  // ros::Rate loop_rate(1000);

  MCL mcl(nh);

  while (ros::ok())
  {
    mcl.filter();
    ros::spinOnce();
    // loop_rate.sleep();
  }

  return 0;
}