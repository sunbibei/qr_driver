/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include <middleware/util/proto/dragon.pb.h>
#include "middleware/propagate/propagate.h"
#include "middleware/util/log.h"


namespace middleware {

Propagate::Propagate() : Propagate("") {
  ;
}

Propagate::Propagate(const std::string& name)
  : propa_name_(name),
    propa_r_cache_size_(1024),
    propa_w_cache_size_(1024),
    cache_r_offset_(0),
    cache_w_offset_(0),
    cache_w_base_(0),
    propa_r_cache_(new uint8_t[propa_r_cache_size_]),
    propa_w_cache_(new uint8_t[propa_w_cache_size_]),
    tmp_ret_(false),
    tmp_read_size_(-1),
    proto_cmd_(new Command()),
    proto_fb_(new Feedback())
{ }

Propagate::~Propagate() {
  if (nullptr != propa_r_cache_) delete propa_r_cache_;
  if (nullptr != propa_w_cache_) delete propa_w_cache_;
  if (nullptr != proto_cmd_)     delete proto_cmd_;
  if (nullptr != proto_fb_)      delete proto_fb_;

  propa_r_cache_ = nullptr;
  propa_w_cache_ = nullptr;
  proto_cmd_     = nullptr;
  proto_fb_      = nullptr;
}

bool Propagate::init(TiXmlElement* root) {
  if (nullptr == root->Attribute("name")) {
    LOG_FATAL << "Could not found the 'name' attribute of the 'propagates/channel' block";
  }
  propa_name_ = root->Attribute("name");
  return true;
}

void Propagate::registerHandle(const std::string& hw_name, HwStateSp s_sp) {
  LOG_INFO << "Register the STATE handle of " << hw_name
      << " into " << propa_name_ << "'s state buffer successful!";
  state_composite_.add(hw_name, s_sp);
}

void Propagate::registerHandle(const std::string& hw_name, HwCmdSp c_sp) {
  LOG_INFO << "Register the COMMAND handle of " << hw_name
      << " into " << propa_name_ << "'s command buffer successful!";
  cmd_composite_.add(hw_name, c_sp);
}

bool Propagate::send(const std::vector<std::string>& jnt_names) {
  tmp_ret_ = true;
  // memset(propa_w_cache_, '\0', propa_w_cache_size_);
  // TODO
  for (const auto& jnt : jnt_names) {
    cmd_composite_[jnt]->parseTo(proto_cmd_);
    tmp_ret_ &= proto_cmd_->SerializeToArray(propa_w_cache_ + cache_w_offset_,
        propa_w_cache_size_ - cache_w_offset_);
    tmp_ret_ &= write(propa_w_cache_ + cache_w_offset_, proto_cmd_->ByteSize(), 20);
    cache_w_offset_ += proto_cmd_->ByteSize();

    if (propa_w_cache_size_ < 8 * proto_cmd_->ByteSize() + cache_w_offset_) {
      // 以8倍的当前长度作为一个参考
      memset(propa_w_cache_, '\0', propa_w_cache_size_ * sizeof(uint8_t));
      cache_w_offset_ = 0;
    }
  }
  if (!tmp_ret_)
    LOG_ERROR << "write " << propa_name_ << "ERROR!";

  return tmp_ret_;
}

bool Propagate::recv() {
  tmp_ret_ = true;
  tmp_read_size_ = read(propa_r_cache_ + cache_r_offset_, propa_r_cache_size_ - cache_r_offset_, recv_index_);
  if (tmp_read_size_ < 0) {
    LOG_ERROR << "read " << propa_name_ << "ERROR (ERROR CODE: "
        << tmp_read_size_ << ")!";
    return false;
  }

  cache_r_offset_ += tmp_read_size_;
  if (!proto_fb_->ParseFromArray(propa_r_cache_ + cache_w_base_,
      cache_r_offset_ - cache_w_base_)) return true; // waiting more data
  else cache_w_base_ += proto_fb_->ByteSize();

  for (auto& state : state_composite_) {
    state.second->updateFrom(proto_fb_);
  }

  if (propa_r_cache_size_ - cache_r_offset_ < 8 * proto_fb_->ByteSize()) {
    // 以8倍的当前长度作为一个参考
    memset(propa_r_cache_, '\0', propa_r_cache_size_ * sizeof(uint8_t));
    cache_r_offset_ = 0;
  }

  return true;
}

/*bool Propagate::parse(Command* cmd) {
  if (!cmd->ParsePartialFromArray(propa_buf_, CACHE_SIZE)) {
    cmd->Clear();
    return false;
  }
  return true;
}

bool Propagate::parse(Feedback* fb) {
  if (!fb->ParsePartialFromArray(propa_buf_, CACHE_SIZE)) {
    fb->Clear();
    return false;
  }
  return true;
}*/

} /* namespace middleware */
