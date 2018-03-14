#include "ObjectDatabase.h"

bool ObjectDatabase::isInDatabase(std::string objectID) {
  for (std::map<std::string, std::vector<GraspPair> >::iterator j =
         grasps.begin(); j != grasps.end(); j++) {
    if (objectID.find(j->first) != std::string::npos) {
      return true;
    }
  }
  for (std::map<std::string, shape_msgs::SolidPrimitive>::iterator p =
         collisionModels.begin(); p != collisionModels.end(); p++) {
    if (objectID.find(p->first) != std::string::npos) {
      return true;
    }
  }

  return false;
}

std::string ObjectDatabase::findDatabaseName(std::string objectID) {
  for (std::map<std::string, std::vector<GraspPair> >::iterator j =
         grasps.begin(); j != grasps.end(); j++) {
    if (objectID.find(j->first) != std::string::npos) {
      ROS_INFO("Found %s under GRASP database name %s", objectID.c_str(),
               j->first.c_str());
      return j->first;
    }
  }

  for (std::map<std::string, shape_msgs::SolidPrimitive>::iterator p =
         collisionModels.begin(); p != collisionModels.end(); p++) {
    if (objectID.find(p->first) != std::string::npos) {
      ROS_INFO("Found %s under MODEL database name %s", objectID.c_str(),
               p->first.c_str());
      return p->first;
    }
  }

  ROS_INFO("Could not find database entry for %s", objectID.c_str());
  return "";
}

shape_msgs::SolidPrimitive ObjectDatabase::getCollisionModel(std::string dbName) {
  return collisionModels[dbName];
}

int ObjectDatabase::getNumGrasps(std::string dbName) {
  return grasps[dbName].size();
}

std::vector<GraspPair> ObjectDatabase::getAllGrasps(std::string dbName) {
  return grasps[dbName];
}
GraspPair ObjectDatabase::getGraspAtIndex(std::string dbName, int index) {
  return grasps[dbName][index];
}

// THE BELOW METHOD IS GROSS AND MESSY AND SPECIFIES MAGIC NUMBERS FOR ALL THE
// OBJECTS THAT SHOULD BE IN THE ENVIRONMENTS
void ObjectDatabase::init() {
  // GLASS CUP
  shape_msgs::SolidPrimitive glassCup;
  glassCup.type = glassCup.CYLINDER;
  glassCup.dimensions.resize(2);
  glassCup.dimensions[0] = 0.09;
  glassCup.dimensions[1] = 0.04;
  collisionModels.insert(std::pair<std::string,
                         shape_msgs::SolidPrimitive>("cup_glass",
                                                     glassCup));
  tf2::Quaternion glassRot;
  glassRot.setRPY(0, M_PI/2, 0);
  GraspPair glassP = std::make_pair(tf2::Transform(glassRot,
                                                   tf2::Vector3(0.0, 0.0, 0.15)),
                                    tf2::Transform(glassRot,
                                                   tf2::Vector3(0.0, 0.0, 0.10)));
  std::vector<GraspPair> glassGrasps;
  glassGrasps.push_back(glassP);
  grasps.insert(std::pair<std::string, std::vector<GraspPair> >("cup_glass",
                                                                glassGrasps));

  // COKE CAN
  shape_msgs::SolidPrimitive coke;
  coke.type = coke.CYLINDER;
  coke.dimensions.resize(2);
  coke.dimensions[0] = 0.12;
  coke.dimensions[1] = 0.035;
  collisionModels.insert(std::pair<std::string,
                         shape_msgs::SolidPrimitive>("coca_cola",
                                                     coke));

  // ALL THE BLOCKS
  for (int i = 3; i <= 13; i += 2) {
    shape_msgs::SolidPrimitive block;
    block.type = block.BOX;
    block.dimensions.resize(3);
    block.dimensions[0] = (double)i/100;
    block.dimensions[1] = (double)i/100;
    block.dimensions[2] = (double)i/100;
    std::string name = "cube" + std::to_string(i) + "cm";
    collisionModels.insert(std::pair<std::string,
                           shape_msgs::SolidPrimitive>(name,
                                                       block));
  }
}
