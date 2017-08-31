/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include <middleware/util/proto/dragon.pb.h>
#include "system/utils/log.h"
#include "system/utils/cfg_reader.h"


#include "middleware/propagate/propagate.h"
#include "propagate_manager.h"

namespace middleware {

#define PROPA_TAG_NAME  ("propagates")

Propagate::Propagate(const MiiString& name)
  : Label(name), propa_name_(name) {
  PropagateManager::instance()->add(this);
}

Propagate::~Propagate() {

}

bool Propagate::init() {
  auto item = MiiCfgReader::instance()->find_first_item(PROPA_TAG_NAME);
  if (!item.is_valid()) {
    LOG_ERROR << "There is no hyper-parameter about " << PROPA_TAG_NAME;
    return false;
  }
  std::string name = item.get_value("name");
  propa_name_ = name;
  return true;
}

bool Propagate::write(class Packet*) {
  LOG_ERROR << "Call the base method 'write'";
  return false;
}

bool Propagate::read(class Packet*) {
  LOG_ERROR << "Call the base method 'write'";
  return false;
}

} /* namespace middleware */
