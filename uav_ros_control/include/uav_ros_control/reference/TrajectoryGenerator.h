#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECOTRY_GENERATOR_H
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <vector>

namespace uav_reference { namespace traj_gen {

static std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>
generateCircleTrajectoryAroundPoint(const double t_x, const double t_y, const double t_z,
    const int t_numberOfPoints = 100, const int t_circleRadius = 1) {
  
  std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajPoints;
  const double DEG_TO_RAD = M_PI / 180.0;
  for(int i = 0; i < t_numberOfPoints; i++) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.transforms = std::vector<geometry_msgs::Transform>(1);
    point.velocities = std::vector<geometry_msgs::Twist>(1);
    point.accelerations = std::vector<geometry_msgs::Twist>(1);
    point.transforms[0].translation.x = t_x + t_circleRadius * cos(360.0 / i * DEG_TO_RAD);
    point.transforms[0].translation.y = t_x + t_circleRadius * sin(360.0 / i * DEG_TO_RAD);
    point.transforms[0].translation.z = t_z;
    trajPoints.push_back(point);
  }
}

}
}

#endif