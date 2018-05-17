#pragma once

#include <string>
#include <map>
#include <fstream>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <shape_msgs/SolidPrimitive.h>
#include "rapidjson/document.h"

typedef std::pair<tf2::Transform, tf2::Transform> GraspPair;
typedef std::pair<tf2::Transform, shape_msgs::SolidPrimitive> SubShape;

class ObjectDatabase {
public:
  ObjectDatabase() {
    init();
  }

  void reload();

  bool isInDatabase(std::string objectID);
  bool dbHasGrasps(std::string objectID);
  bool dbHasModel(std::string objectID);
  std::string findDatabaseName(std::string objectID);

  std::vector<SubShape> getCollisionModel(std::string dbName);
  int getNumGrasps(std::string dbName);
  std::vector<GraspPair> getAllGrasps(std::string dbName);
  GraspPair getGraspAtIndex(std::string dbName, int index);

private:
  void init();

  std::map<std::string, std::vector<SubShape> > collisionModels;
  std::map<std::string, std::vector<GraspPair> > grasps;
};
