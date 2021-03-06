#pragma once

#include <map>
#include <string>

#include <tf2/utils.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "gazebo_msgs/ModelStates.h"

class WorldObjects {
public:
 WorldObjects() : tableH(0.675) {}

  void update(const gazebo_msgs::ModelStates::ConstPtr& msg);

  int numObjects() { return objectPoses.size(); }
  bool isInScene(std::string subName);
  std::string nameInScene(std::string subName);
  std::vector<std::string> allObjectNames();

  tf2::Transform getXformOf(std::string name);
  tf2::Vector3 getPositionOf(std::string name);
  float getPosX(std::string name);
  float getPosY(std::string name);
  float getPosZ(std::string name);
  tf2::Quaternion getRotationOf(std::string name);

  tf2::Transform getWorldXform() { return worldXform; }
  float getTableH() { return tableH; }

  tf2::Vector3 worldXformTimesPos(std::string name);
  tf2::Quaternion worldXformTimesRot(std::string name);
  tf2::Transform worldXformTimesTrans(std::string name);

private:
  std::map<std::string, tf2::Vector3> objectPoses;
  std::map<std::string, tf2::Quaternion> objectRotations;
  tf2::Transform worldXform;
  float tableH;
  boost::mutex objMutex;
};
