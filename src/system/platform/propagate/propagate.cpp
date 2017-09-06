/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include <system/platform/protocol/proto/dragon.pb.h>
#include <system/platform/propagate_manager.h>
#include <system/foundation/utf.h>
#include "system/foundation/cfg_reader.h"


#include "system/platform/propagate/propagate.h"

namespace middleware {

#define PROPA_TAG_NAME  ("propagates")

Propagate::Propagate(const MiiString& l)
  : Label(l), propa_name_("") {
  PropagateManager::instance()->add(this);
}
bool Propagate::init() {
  MiiString __p;
  Label::split_label(label_, __p, propa_name_);
  return true;
}

bool Propagate::write(const Packet&) {
  LOG_ERROR << "Call the base method 'write'";
  return false;
}

bool Propagate::read(Packet&) {
  LOG_ERROR << "Call the base method 'write'";
  return false;
}

bool Propagate::start() {
  LOG_WARNING << "This method (Propagate::start()) should not be called.";
  return false;
}

void Propagate::stop() {
  LOG_WARNING << "This method (Propagate::stop()) should not be called.";
}

} /* namespace middleware */
