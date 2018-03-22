#include "WorldObjects.h"

void WorldObjects::update(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  boost::lock_guard<boost::mutex> guard(objMutex);

  objectPoses.clear();
  objectRotations.clear();

  for (int i = 0; i < msg->name.size(); i++) {
    geometry_msgs::Pose p = msg->pose[i];

    if (msg->name[i].find("fetch") != std::string::npos) {
      tf2::Vector3 fetchVec;
      tf2::fromMsg(p.position, fetchVec);
      tf2::Quaternion fetchQuat;
      tf2::fromMsg(p.orientation, fetchQuat);
      worldXform = tf2::Transform(fetchQuat, fetchVec).inverse();
      continue;
    }
    tf2::Vector3 v;
    tf2::fromMsg(p.position, v);
    objectPoses.insert(std::pair<std::string, tf2::Vector3>(msg->name[i],
                                                            v));

    tf2::Quaternion quat;
    tf2::fromMsg(p.orientation, quat);
    objectRotations.insert(std::pair<std::string, tf2::Quaternion>(msg->name[i],
                                                                   quat));
  }
}

bool WorldObjects::isInScene(std::string subName) {
  if (nameInScene(subName) == "") return false;
  else return true;
}

std::string WorldObjects::nameInScene(std::string subName) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  for (std::map<std::string, tf2::Vector3>::iterator i = objectPoses.begin();
       i != objectPoses.end(); i++) {
    if (i->first.find(subName) != std::string::npos) {
      return i->first;
    }
  }
  return "";
}

std::vector<std::string> WorldObjects::allObjectNames() {
  std::vector<std::string> names;
  for (std::map<std::string, tf2::Vector3>::iterator i = objectPoses.begin();
       i != objectPoses.end(); i++) {
    names.push_back(i->first);
  }
  return names;
}

tf2::Transform WorldObjects::getXformOf(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return tf2::Transform(objectRotations[name], objectPoses[name]);
}

tf2::Vector3 WorldObjects::getPositionOf(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return objectPoses[name];
}

float WorldObjects::getPosX(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return objectPoses[name].x();
}

float WorldObjects::getPosY(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return objectPoses[name].y();
}

float WorldObjects::getPosZ(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return objectPoses[name].z();
}

tf2::Quaternion WorldObjects::getRotationOf(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return objectRotations[name];
}

tf2::Vector3 WorldObjects::worldXformTimesPos(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return worldXform*objectPoses[name];
}

tf2::Quaternion WorldObjects::worldXformTimesRot(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return worldXform*objectRotations[name];
}

tf2::Transform WorldObjects::worldXformTimesTrans(std::string name) {
  boost::lock_guard<boost::mutex> guard(objMutex);
  return worldXform*tf2::Transform(objectRotations[name], objectPoses[name]);
}
