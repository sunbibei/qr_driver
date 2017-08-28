/*
 * touchdown.h
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_

#include "hw_unit.h"

#include <atomic>

namespace middleware {

struct TDState : public HwState {
  TDState(double d = 0) : data(0) {}
  std::atomic<double> data;
};


class TouchDown: public HwUnit {
public:
  typedef TDState                      StateType;
  typedef boost::shared_ptr<TDState>   StateTypeSp;

public:
  TouchDown(const std::string& name = "touchdown");
  virtual ~TouchDown();

  virtual bool init(TiXmlElement*)   override;
  virtual bool      requireCmdReg()  override;
  virtual HwStateSp getStateHandle() override;
  virtual HwCmdSp   getCmdHandle()   override;
  virtual HwStateSp getState()       override;
  virtual HwCmdSp   getCommand()     override;
  virtual void setState(const HwState&)     override;
  virtual void setCommand(const HwCommand&) override;

  virtual void publish() override;

  // for debug
  virtual void check() override;

private:
  LegType     leg_;
  StateTypeSp td_sp_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_ */
