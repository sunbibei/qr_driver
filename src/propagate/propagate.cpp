/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "middleware/propagate/propagate.h"
#include "middleware/util/log.h"

#include "middleware/propagate/proto/dragon.pb.h"

namespace middleware {

Propagate::Propagate() : Propagate("") {
  ;
}

Propagate::Propagate(const std::string& name)
  : propa_name_(name),
    propa_r_cache_size_(1024),
    propa_w_cache_size_(1024),
    propa_r_cache_(new uint8_t[propa_r_cache_size_]),
    propa_w_cache_(new uint8_t[propa_w_cache_size_]),
    proto_cmd_(new Command()),
    proto_fb_(new Feedback())
{ }

Propagate::~Propagate() {
  if (nullptr != propa_r_cache_) delete propa_r_cache_;
  if (nullptr != propa_w_cache_) delete propa_w_cache_;
  if (nullptr != proto_cmd_) delete proto_cmd_;
  if (nullptr != proto_fb_)  delete proto_fb_;

  propa_r_cache_ = nullptr;
  propa_w_cache_ = nullptr;
  proto_cmd_   = nullptr;
  proto_fb_    = nullptr;
}

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

void Propagate::registerHandle(HwUnit* unit) {
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

bool Propagate::send(const std::vector<std::string>& jnt_names) {
  bool tmp_ret_ = true;
  // memset(propa_w_cache_, '\0', propa_w_cache_size_);
  int cache_w_offset_ = 0;
  for (const auto& jnt : jnt_names) {
    cmd_composite_[jnt]->update(proto_cmd_);
    tmp_ret_ &= proto_cmd_->SerializeToArray(propa_w_cache_ + cache_w_offset_,
        propa_w_cache_size_ - cache_w_offset_);
    tmp_ret_ &= write(propa_w_cache_ + cache_w_offset_, proto_cmd_->ByteSize());
    if (propa_w_cache_size_ - cache_w_offset_ < 8 * proto_cmd_->ByteSize()) {
      // 以8倍的当前长度作为一个参考
      memset(propa_w_cache_, '\0', propa_w_cache_size_ * sizeof(uint8_t));
      cache_w_offset_ = 0;
    }
    cache_w_offset_ += proto_cmd_->ByteSize();
  }
  if (!tmp_ret_)
    LOG_ERROR << "write " << propa_name_ << "ERROR!";

  return tmp_ret_;
}

bool Propagate::recv() {
  bool tmp_ret_ = true;
  int read_size = read(propa_r_cache_, propa_r_cache_size_);
  if (read_size < 0) {
    LOG_ERROR << "read " << propa_name_ << "ERROR!";
    return false;
  }

  int cache_r_offset_ = 0;
  proto_fb_->ParseFromArray(propa_r_cache_ + cache_r_offset_,
      propa_r_cache_size_ - cache_r_offset_);




  return tmp_ret_;
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
