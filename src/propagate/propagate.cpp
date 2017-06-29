/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "middleware/propagate/propagate.h"
#include "middleware/util/log.h"

namespace middleware {

Propagate::Propagate() : Propagate("") {
  ;
}

Propagate::Propagate(const std::string& name)
  : propa_name_(name)
{ }

Propagate::~Propagate()
{ }

void Propagate::registerHandle(boost::shared_ptr<HwUnit> unit) {
  if ((unit->cmd_channel_.empty()) || (0 == unit->cmd_channel_.compare(propa_name_))) {
    LOG_INFO << "Register the COMMAND handle of " << unit->hw_name_
        << " into " << propa_name_ << "'s command buffer successful!";
    cmd_composite_.add(unit->hw_name_, unit->getCmdHandle());
  }
  if ((unit->state_channel_.empty()) || (0 == unit->state_channel_.compare(propa_name_))) {
    LOG_INFO << "Register the STATE handle of " << unit->hw_name_
        << " into " << propa_name_ << "'s state buffer successful!";
    state_composite_.add(unit->hw_name_, unit->getStataHandle());
  }
}

} /* namespace middleware */
