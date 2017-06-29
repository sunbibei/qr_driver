/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include "middleware/propagate/com.h"

#include "middleware/propagate/propagate.h"
#include "middleware/util/log.h"
#include "middleware/util/composite.h"
#include "middleware/hardware/hw_unit.h"

namespace middleware {

ComChannel::ComChannel(const std::string& name)
  :Propagate(name)
{ }

ComChannel::~ComChannel() { }

// 完成PCAN的初始化
// 以及act_state_map_, act_cmd_map_, enc_state_map_三个MAP的从cmd_map_和state_map_中初始化
bool ComChannel::init(TiXmlElement*) {
  return true;
}

void ComChannel::stop() {
  return;
}

// 完成数据的读写. (下面全是测试代码)
bool ComChannel::write(const std::vector<std::string>& names) {
  if (names.empty()) return true;

  return true;
}

// (下面全是测试代码)
bool ComChannel::read() {

  return true;
}

} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::ComChannel, middleware::Propagate)
