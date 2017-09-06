/*
 * auto_instanceor.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#include <system/foundation/utf.h>
#include "system/foundation/auto_instanceor.h"

namespace middleware {

std::map<MiiString, Label::LabelPtr> AutoInstanceor::s_inst_table_;
AutoInstanceor* AutoInstanceor::instance_ = nullptr;

bool AutoInstanceor::create_instance(const MiiString& lib) {
  if (nullptr != instance_) {
    // LOG_WARNING << "Create the AutoInstanceor instance twice!";
    std::cout << "Create the AutoInstanceor instance twice!" << std::endl;
  }
  instance_ = new AutoInstanceor(lib);
  return true;
}

void AutoInstanceor::destroy_instance() {
  ;
}

AutoInstanceor* AutoInstanceor::instance() {
  if (nullptr == instance_) {
    // LOG_WARNING << "Call instance() method after create_instance()";
    std::cout  << "Call instance() method after create_instance()" << std::endl;
  }

  return instance_;
}


AutoInstanceor::AutoInstanceor(const MiiString& lib_path)
  : class_loader_(nullptr) {
  class_loader_ = new class_loader::ClassLoader(lib_path);
}

AutoInstanceor::~AutoInstanceor() {
  // The s_inst_table_ will be automatic dealloc.
  if (nullptr != class_loader_) {
    delete class_loader_;
    class_loader_ = nullptr;
  }
}

bool AutoInstanceor::make_instance(const MiiString& __type, const MiiString& __l) {
  auto class_list = class_loader_->getAvailableClasses<Label>();
  bool found = false;
  std::cout << "Available Classes: ";
  for (const auto& c : class_list) {
    std::cout << c << " ";
    if (c == __type)
      found = true;
  }
  std::cout << std::endl;
  if (found) std::cout << "FOUND" << std::endl;
  else std::cout << "NO FOUND" << std::endl;

  if (!found) return false;

  Label::LabelPtr __inst = class_loader_->createInstance<Label>(__type);
  if (nullptr != __inst) {
    // 情非得已， 不应该出现这样的代码。
    // 但当前自动实例使用class_loader的方式，仅能如此妥协处理。
    // 庆幸的是，所有问题都还控制在AutoInstance中
    __inst->label_ = __l;
    __inst->init();
    s_inst_table_.insert(std::make_pair(__inst->getLabel(), __inst));
    Label::registerClass(__inst, __l);
    std::cout << "Create Instance successful, Address: "
        << __inst.get() << ", Count: " << __inst.use_count() << std::endl;
    return true;
  } else {
    std::cout << "What FUNK!" << std::endl;
    return false;
  }
}

} /* namespace middleware */
