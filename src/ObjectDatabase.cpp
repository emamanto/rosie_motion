#include "ObjectDatabase.h"

bool ObjectDatabase::isInDatabase(std::string objectID) {
  return (dbHasGrasps(objectID) || dbHasModel(objectID));
}

bool ObjectDatabase::dbHasGrasps(std::string objectID) {
  for (std::map<std::string, std::vector<GraspPair> >::iterator j =
         grasps.begin(); j != grasps.end(); j++) {
    if (j->first.find(objectID) != std::string::npos ||
        objectID.find(j->first) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool ObjectDatabase::dbHasModel(std::string objectID) {
  for (std::map<std::string, shape_msgs::SolidPrimitive>::iterator p =
         collisionModels.begin(); p != collisionModels.end(); p++) {
    if (p->first.find(objectID) != std::string::npos ||
        objectID.find(p->first) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string ObjectDatabase::findDatabaseName(std::string objectID) {
  for (std::map<std::string, std::vector<GraspPair> >::iterator j =
         grasps.begin(); j != grasps.end(); j++) {
    if (j->first.find(objectID) != std::string::npos ||
        objectID.find(j->first) != std::string::npos) {
      //ROS_INFO("Found %s under GRASP database name %s", objectID.c_str(),
      //         j->first.c_str());
      return j->first;
    }
  }

  for (std::map<std::string, shape_msgs::SolidPrimitive>::iterator p =
         collisionModels.begin(); p != collisionModels.end(); p++) {
    if (p->first.find(objectID) != std::string::npos ||
        objectID.find(p->first) != std::string::npos) {
      //ROS_INFO("Found %s under MODEL database name %s", objectID.c_str(),
      //         p->first.c_str());
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

// Reads in json specifying data about the objects the robot may find
void ObjectDatabase::init() {
  std::ifstream jsonfile("/home/mamantov/catkin_ws/src/rosie_motion/config/object_info.json");
  int length = 0;
  if (jsonfile) {
    jsonfile.seekg(0, jsonfile.end);
    length = jsonfile.tellg();
    jsonfile.seekg(0, jsonfile.beg);
  }
  else {
    ROS_WARN("Could not init object database at all!");
    return;
  }

  char* buf = new char[length];
  jsonfile.read(buf, length);
  // Make sure there is nothing else after the json object
  for (int i = length-1; i > 0; i--) {
    if (buf[i] != '}') buf[i] = 0;
    else break;
  }
  rapidjson::Document d;
  d.Parse(buf);
  delete[] buf;

  if (d.HasParseError()) {
    ROS_WARN("Failed to parse json file with error code %i at %i",
             d.GetParseError(),
             (int)d.GetErrorOffset());
  }

  if (d.HasMember("objects")) {
    ROS_INFO("ObjectDatabase successfully read object database json file!");
  }

  const rapidjson::Value& objs = d["objects"];
  assert(objs.IsArray());

  ROS_INFO("There are %i objects in the object_info file.", (int)objs.Size());

  for (int i = 0; i < objs.Size(); i++) {
    shape_msgs::SolidPrimitive shape;
    if (objs[i]["shape"] == "cylinder") {
      shape.type = shape.CYLINDER;
    }
    else if (objs[i]["shape"] == "box") {
      shape.type = shape.BOX;
    }
    else {
      ROS_WARN("Unknown object type %s found!", objs[i]["shape"].GetString());
      continue;
    }

    assert(objs[i]["dimensions"].IsArray());
    if (shape.type == shape.CYLINDER) {
      if (objs[i]["dimensions"].Size() != 2) {
        ROS_WARN("Cylinders need two elements in their dimensions.");
        continue;
      }
      shape.dimensions.resize(2);
    }
    else if (shape.type == shape.BOX) {
      if (objs[i]["dimensions"].Size() != 3) {
        ROS_WARN("Boxes need three elements in their dimensions.");
        continue;
      }
      shape.dimensions.resize(3);
    }
    for (int j = 0; j < shape.dimensions.size(); j++) {
      shape.dimensions[j] = objs[i]["dimensions"][j].GetDouble();
    }

    collisionModels.insert(std::pair<std::string,
                           shape_msgs::SolidPrimitive>(objs[i]["name"].GetString(),
                                                       shape));

    assert(objs[i]["grasps"].IsArray());
    std::vector<GraspPair> allGrasps;
    for (int j = 0; j < objs[i]["grasps"].Size(); j++) {
      if (objs[i]["grasps"][j]["first"].Size() != 6) {
        ROS_WARN("Wrong number of elements in first grasp xyzrpy.");
      }
      tf2::Quaternion rot1;
      rot1.setRPY(objs[i]["grasps"][j]["first"][3].GetDouble(),
                  objs[i]["grasps"][j]["first"][4].GetDouble(),
                  objs[i]["grasps"][j]["first"][5].GetDouble());
      tf2::Vector3 vec1(objs[i]["grasps"][j]["first"][0].GetDouble(),
                        objs[i]["grasps"][j]["first"][1].GetDouble(),
                        objs[i]["grasps"][j]["first"][2].GetDouble());
      tf2::Transform t1(rot1, vec1);

      if (objs[i]["grasps"][j]["second"].Size() != 6) {
        ROS_WARN("Wrong number of elements in second grasp xyzrpy.");
      }
      tf2::Quaternion rot2;
      rot2.setRPY(objs[i]["grasps"][j]["second"][3].GetDouble(),
                  objs[i]["grasps"][j]["second"][4].GetDouble(),
                  objs[i]["grasps"][j]["second"][5].GetDouble());
      tf2::Vector3 vec2(objs[i]["grasps"][j]["second"][0].GetDouble(),
                        objs[i]["grasps"][j]["second"][1].GetDouble(),
                        objs[i]["grasps"][j]["second"][2].GetDouble());
      tf2::Transform t2(rot2, vec2);

      GraspPair gp = std::make_pair(t1, t2);
      allGrasps.push_back(gp);
    }
    grasps.insert(std::pair<std::string, std::vector<GraspPair> >(objs[i]["name"].GetString(),
                                                                  allGrasps));
  }

  ROS_INFO("ObjectDatabase loaded collision info for %i objects and grasp info for %i objects",
           (int)collisionModels.size(),
           (int)grasps.size());
}
