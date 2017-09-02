/*
 * joint_manager.h
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_H_

#include "system/utils/resource_manager.h"
#include "middleware/hardware/joint.h"

#include <vector>
#include <string>
#include <map>

namespace middleware {

class JointManager: public ResourceManager<Joint> {
public:
  static JointManager* create_instance();
  static JointManager* instance();
  static void          destroy_instance();

  typedef struct {

    JntCmdType mode;
    double     val;
  } JointCommand;

public:
  virtual void add(Joint* _res) override;

  void   addJointCommand(LegType, JntType, double);
  void   addJointCommand(LegType, JntType, double);
  Joint* getJointByType(LegType, JntType);


protected:
  // Owner Size * Joint Size
  std::vector<std::vector<Joint*>>   jnt_list_by_type_;
  std::map<std::string, Joint*>      jnt_list_by_name_;

protected:
  static JointManager* instance_;
  JointManager();
  virtual ~JointManager();
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_H_ */
