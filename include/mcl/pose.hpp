#pragma once

struct pose
{
  double x;
  double y;
  double yaw;
  double weight = 1;

  pose()
  {
    x=0;
    y=0;
    yaw=0;
  }

  pose(double x_pose, double y_pose, double theta)
  {
    x = x_pose;
    y = y_pose;
    yaw = theta;
  }
};
