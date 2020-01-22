#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECOTRY_GENERATOR_H
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <list>

namespace uav_reference { namespace traj_gen {

static trajectory_msgs::MultiDOFJointTrajectoryPoint 
toTrajectoryPointMsg(const double x, const double y, const double z) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.transforms = std::vector<geometry_msgs::Transform>(1);
  point.velocities = std::vector<geometry_msgs::Twist>(1);
  point.accelerations = std::vector<geometry_msgs::Twist>(1);
  point.transforms[0].translation.x = x;
  point.transforms[0].translation.y = y;
  point.transforms[0].translation.z = z;
  return point;
}

static std::list<trajectory_msgs::MultiDOFJointTrajectoryPoint>
generateCircleTrajectoryAroundPoint(const double t_x, const double t_y, const double t_z,
    const int t_numberOfPoints = 100, const int t_circleRadius = 1) {
  
  std::list<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajPoints;
  const double DEG_TO_RAD = M_PI / 180.0;
  double angleInc = 360.0 / t_numberOfPoints * DEG_TO_RAD;
  for(int i = 0; i < t_numberOfPoints; i ++) {
    trajPoints.push_back(toTrajectoryPointMsg(
      t_x + t_circleRadius * cos(i * angleInc), 
      t_y + t_circleRadius * sin(i * angleInc),
      t_z
    ));
  }
  return trajPoints;
}

static bool isCloseToReference(const trajectory_msgs::MultiDOFJointTrajectoryPoint& ref,
    const nav_msgs::Odometry& odom, const double tol = 1e-2) {
  return sqrt(
    pow(ref.transforms[0].translation.x - odom.pose.pose.position.x, 2) + 
    pow(ref.transforms[0].translation.y - odom.pose.pose.position.y, 2) + 
    pow(ref.transforms[0].translation.z - odom.pose.pose.position.z, 2)) < tol;
}

static std::list<double> inRange(const double from, const double to, const int numberOfPOints = 100) {
  double step = (to - from) / numberOfPOints;
  std::list<double> points;
  for (int i = 0; i < numberOfPOints; i ++) {
    points.push_back(i * step + from);
  }
  return points;
}

static std::list<trajectory_msgs::MultiDOFJointTrajectoryPoint> generateLinearTrajctory(const double x, const double y, const double z, 
    const nav_msgs::Odometry& odom, const int numberOfPOints = 100) {
  
  std::list<double> rangeX(inRange(odom.pose.pose.position.x, x)),
    rangeY(inRange(odom.pose.pose.position.y, y)),
    rangeZ(inRange(odom.pose.pose.position.z, z));
  
  std::list<trajectory_msgs::MultiDOFJointTrajectoryPoint> points;
  auto itX = rangeX.begin(), itY = rangeY.begin(), itZ = rangeZ.begin();
  while (itX != rangeX.end() && itY != rangeY.end() && itZ != rangeZ.end()) {
    points.push_back(toTrajectoryPointMsg(*itX, *itY, *itZ));
    itX++; 
    itY++;
    itZ++;
  }
  return points;
}

}
}

#endif