/*
 * touchdown.h
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_RESOURCES_TOUCHDOWN_H_
#define INCLUDE_SYSTEM_RESOURCES_TOUCHDOWN_H_

#include <system/foundation/label.h>

namespace middleware {

class ForceSensor : public Label {
  friend class LegNode;
public:
  ForceSensor(const MiiString& __l = Label::null);
  // 妥协方案
  virtual bool init() override;
  ~ForceSensor();

  /**
   * Interface for user layer
   */
  const LegType& leg_type() const;

  double force_data();
  const double* force_data_const_pointer();
  const double& force_data_const_ref();

protected:

  /**
   * Interface for communication layer
   */
  // data = count * scale + offset
  void updateForceCount(short count);

  LegType           leg_type_;
  class ForceState* td_state_;

  double         scale_;
  double         offset_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_RESOURCES_TOUCHDOWN_H_ */