/*
 * auto_instanceor.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#include <system/foundation/utf.h>
#include "system/foundation/auto_instanceor.h"

// Cancel the namespace middleware
// namespace middleware {

// std::map<MiiString, Label::LabelPtr> AutoInstanceor::s_inst_table_;
SINGLETON_IMPL_NO_CREATE(AutoInstanceor)

AutoInstanceor* AutoInstanceor::create_instance(const MiiString& lib) {
  if (nullptr != instance_) {
    LOG_WARNING << "Create the AutoInstanceor instance twice!";
    // std::cout << "Create the AutoInstanceor instance twice!" << std::endl;
  } else
    instance_ = new AutoInstanceor(lib);
  return instance_;
}


AutoInstanceor::AutoInstanceor(const MiiString& lib_path)
  : class_loader_(nullptr) {
  class_loader_ = new class_loader::ClassLoader(lib_path);

  // Just for debug
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-";
    auto class_list = class_loader_->getAvailableClasses<Label>();
    LOG_INFO << "Available Classes: ";
    int count = 0;
    for (const auto& c : class_list)
      LOG_INFO << "    " << count++ << ")  " << c;
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-";
  }
}

AutoInstanceor::~AutoInstanceor() {
  // s_inst_table_.clear();
  Label::s_label_table_.clear();
  // The s_inst_table_ will be automatic dealloc.
  if (nullptr != class_loader_) {
    delete class_loader_;
    class_loader_ = nullptr;
  }
}

bool AutoInstanceor::make_instance(const MiiString& __p, const MiiString& __type) {
  auto class_list = class_loader_->getAvailableClasses<Label>();
  bool found = false;
  for (const auto& c : class_list) {
    if (0 == c.compare(__type))
      found = true;
  }
  if (!found) {
    LOG_WARNING << "The type " << __type
        << " has not found in the available class list.";
    return false;
  }/* else {
    LOG_DEBUG << "Instance " << __type << " named by " << __p;
  }*/

  Label::LabelPtr __inst = class_loader_->createInstance<Label>(__type);
  // LOG_DEBUG << "createInstance<Label> OK! address: " << __inst.get();
  if (nullptr != __inst.get()) {
    // 情非得已， 不应该出现这样的代码。
    // 但当前自动实例使用class_loader的方式，仅能如此妥协处理。
    // 庆幸的是，所有问题都还控制在AutoInstance中
    __inst->label_ = __p;
    /*LOG_DEBUG << "Ready to call this instance '" << __inst->getLabel()
        << "'s init()";*/
    __inst->init();
    /*LOG_DEBUG << "Finished to call this instance '" << __inst->getLabel()
        << "'s init()";*/
    // s_inst_table_.insert(std::make_pair(__inst->getLabel(), __inst));
    Label::registerClass(__inst);
    /*LOG_DEBUG << "Instance " << __inst->getLabel() << ", Address: "
        << __inst.get() << ", Count: " << __inst.use_count();*/
    return true;
  } else {
    LOG_WARNING << "What FUNK! The '" << Label::make_label(__p, __type)
        << " instances fail.";
    return false;
  }
}

//} /* namespace middleware */
