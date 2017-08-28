/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include <middleware/util/proto/dragon.pb.h>
#include "middleware/hardware/hw_unit.h"
#include "middleware/middleware.h"
#include "middleware/util/log.h"


namespace middleware {

HwUnit::HwUnit() : node_id_(INVALID_ID) {
  HwManager::instance()->add(this);
}

bool HwUnit::init(TiXmlElement* root) {
  if ((!root) || (!root->Attribute("name"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong!";
    return false;
  }
  hw_name_ = root->Attribute("name");

  if ((!root->Attribute("id")) && (!root->Attribute("node_id"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong!";
    return false;
  }
  unsigned int id = 0;
  std::string id_str = (root->Attribute("id") ?
      root->Attribute("id") : root->Attribute("node_id"));
  std::string template_str;
  if (('0' == id_str[0]) && ('x' == id_str[1])) {
    // Hex to id
    template_str = "0x%x";
  } else {
    template_str = "%d";
  }
  sscanf(id_str.c_str(), template_str.c_str(), &id);
  node_id_ = id;

  return true;
}

void HwUnit::handleMsg(const Packet&) {
  ;
}

HwUnit::~HwUnit()                 { }
bool HwUnit::generateCmd(Packet&) { return false;  }
bool HwUnit::requireCmdDeliver()  { return true;   }

} /* namespace quadruped_robot_driver */


