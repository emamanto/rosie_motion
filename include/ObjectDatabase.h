#pragma once

#include <string>
#include <map>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <shape_msgs/SolidPrimitive.h>

typedef std::pair<tf2::Transform, tf2::Transform> GraspPair;

class ObjectDatabase {
public:
  ObjectDatabase() {
    init();
  }

  bool isInDatabase(std::string objectID);
  bool dbHasGrasps(std::string objectID);
  bool dbHasModel(std::string objectID);
  std::string findDatabaseName(std::string objectID);

  shape_msgs::SolidPrimitive getCollisionModel(std::string dbName);
  int getNumGrasps(std::string dbName);
  std::vector<GraspPair> getAllGrasps(std::string dbName);
  GraspPair getGraspAtIndex(std::string dbName, int index);

private:
  void init();

  std::map<std::string, shape_msgs::SolidPrimitive> collisionModels;
  std::map<std::string, std::vector<GraspPair> > grasps;
};
