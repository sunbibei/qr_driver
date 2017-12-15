/*
 * mii_robot.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "repository/resource/force_sensor.h"
#include "repository/resource/imu_sensor.h"
#include "repository/resource/joint.h"
#include "repository/resource/joint_manager.h"
#include "repository/registry.h"

#include "system/platform/master.h"
#include "system/platform/propagate/propagate_manager.h"
#include "system/robot/mii_robot.h"
#include "system/platform/thread/threadpool.h"

#include "foundation/label.h"
#include "foundation/cfg_reader.h"
#include "foundation/auto_instanceor.h"

namespace middleware {

#define OWNER_CTRL_THREAD  ("mii-control-support")

struct __RegJntRes {
  ///! Order by { JntType::KNEE, JntType::HIP, JntType::YAW }
  Eigen::VectorXd* resource[LegType::N_LEGS][JntDataType::N_JNT_DATA_TYPES];
  Eigen::VectorXd* command[LegType::N_LEGS];
  double*          command_pointer[LegType::N_LEGS][JntType::N_JNTS];
  __RegJntRes() {
    for (auto& c : command)
      c = nullptr;
    for (auto& rs : resource)
      for (auto& r : rs)
        r = nullptr;
    for (auto& cps : command_pointer)
      for (auto& cp : cps)
        cp = nullptr;
  }

  ~__RegJntRes() {
    for (auto& c : command)
      if (c) delete c;
    for (auto& rs : resource)
      for (auto& r : rs)
        if (r) delete r;
    for (auto& cps : command_pointer)
      for (auto& cp : cps)
        if (cp) delete cp;
  }
};

struct __RegForceRes {
  ; // const double* force_pointer[LegType::N_LEGS];
};

struct __RegImuRes {
  Eigen::VectorXd*   orientation;
  Eigen::VectorXd*   ang_vel;
  Eigen::VectorXd*   lin_acc;

  Eigen::MatrixXd*   orientation_cov;
  Eigen::MatrixXd*   ang_vel_cov;
  Eigen::MatrixXd*   lin_acc_cov;

  __RegImuRes() {
    orientation     = nullptr;
    ang_vel         = nullptr;
    lin_acc         = nullptr;
    orientation_cov = nullptr;
    ang_vel_cov     = nullptr;
    lin_acc_cov     = nullptr;
  }

  ~__RegImuRes() {
    if (orientation)     delete orientation;
    if (ang_vel)         delete ang_vel;
    if (lin_acc)         delete lin_acc;
    if (orientation_cov) delete orientation_cov;
    if (ang_vel_cov)     delete ang_vel_cov;
    if (lin_acc_cov)     delete lin_acc_cov;
  }
};

void MiiRobot::auto_inst(const MiiString& __p, const MiiString& __type) {
  if (!AutoInstanceor::instance()->make_instance(__p, __type)) {
    LOG_ERROR << "Create instance(" << __type << " " << __p << ") fail!";
  }
}

MiiRobot::MiiRobot(const MiiString& __tag)
: prefix_tag_(__tag), jnt_manager_(nullptr), imu_sensor_(nullptr),
  mii_ctrl_alive_(false), tick_interval_(20),use_mii_control_(false),
  jnt_reg_res_(new __RegJntRes), td_reg_res_(new __RegForceRes),
  imu_reg_res_(new __RegImuRes) {
  ;
}

/**
 * This method creates the part of singleton.
 */
void MiiRobot::create_system_instance() {
  if (nullptr == ThreadPool::create_instance())
    LOG_FATAL << "Create the singleton 'ThreadPool' has failed.";

  if (nullptr == Master::create_instance())
    LOG_FATAL << "Create the singleton 'Master' has failed.";

  if (nullptr == Registry::create_instance())
    LOG_FATAL << "Create the singleton 'Registry' has failed.";

  if (nullptr == JointManager::create_instance())
    LOG_FATAL << "Create the singleton 'JointManager' has failed.";
}

bool MiiRobot::init(bool use_mii_control) {
  create_system_instance();

  auto cfg = MiiCfgReader::instance();
  if (nullptr == cfg) {
    LOG_FATAL << "The MiiCfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before MiiRobot::init()";
  }

  MiiString str;
  cfg->get_value(prefix_tag_, "control_mode", str);
  if (str.empty() || (0 == str.compare("position")))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_POS);
  else if (0 == str.compare("velocity"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_VEL);
  else if (0 == str.compare("torque"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_TOR);
  else if (0 == str.compare("pos-vel"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_POS_VEL);
  else if (0 == str.compare("motor-velocity"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_MOTOR_VEL);
  else
    ;

  use_mii_control_ = use_mii_control;
  // cfg->get_value(prefix_tag_, "mii_control", use_mii_control_);
  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", MiiRobot::auto_inst);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  if (!use_mii_control_) Label::printfEveryInstance();

//  master_  = Master::instance();
//  master_->init();
  Master::instance()->init();

  jnt_manager_ = JointManager::instance();

  MiiVector<MiiString> vec_str;
  cfg->get_value_fatal(
      Label::make_label(prefix_tag_, "touchdowns"), "labels", vec_str);
  td_list_by_type_.resize(vec_str.size());
  for (const auto& td : vec_str) {
    ForceSensor* p_td = Label::getHardwareByName<ForceSensor>(td);
    if (nullptr != p_td) {
      auto leg = p_td->leg_type();
      td_list_.push_back(p_td);
      td_list_by_type_[leg] = p_td;
    }
  }

  MiiString imu_name;
  cfg->get_value_fatal(Label::make_label(prefix_tag_, "imu"), "labels", imu_name);
  imu_sensor_  = Label::getHardwareByName<ImuSensor>(imu_name);

  if (use_mii_control_) {
    __reg_resource_and_command(Label::make_label(prefix_tag_, "registry"));
    Registry::instance()->print();
    double frequency = 50;
    cfg->get_value(prefix_tag_, "frequency", frequency);
    tick_interval_ = std::chrono::milliseconds((int)(1000.0/frequency));
    ThreadPool::instance()->add(OWNER_CTRL_THREAD, &MiiRobot::supportRegistry, this);
  }

  return true;
}

void MiiRobot::__reg_resource_and_command(const MiiString& _prefix) {
  for (const auto& t : {LegType::FL, LegType::HL, LegType::FR, LegType::HR}) {
    jnt_reg_res_->command[t] = new Eigen::VectorXd((int)JntType::N_JNTS);
    for (const auto& d : {JntDataType::POS, JntDataType::VEL, JntDataType::TOR})
      jnt_reg_res_->resource[t][d] = new Eigen::VectorXd((int)JntType::N_JNTS);
    for (const auto& j : {JntType::KNEE, JntType::HIP, JntType::YAW})
      jnt_reg_res_->command_pointer[t][j] = new double;
  }

  imu_reg_res_->orientation     = new Eigen::VectorXd(4);
  imu_reg_res_->orientation_cov = new Eigen::MatrixXd(3, 3);
  imu_reg_res_->lin_acc         = new Eigen::VectorXd(3);
  imu_reg_res_->lin_acc_cov     = new Eigen::MatrixXd(3, 3);
  imu_reg_res_->ang_vel         = new Eigen::VectorXd(3);
  imu_reg_res_->ang_vel_cov     = new Eigen::MatrixXd(3, 3);

  // Register the single joint constant pointer
  /*for (const auto& j : *jnt_manager_) {
    MiiString post = std::to_string(j->owner_type()) + "-" + std::to_string(j->joint_type());
    REG_RESOURCE("jnt-pos-" + post, j->joint_position_const_pointer());
    REG_RESOURCE("jnt-vel-" + post, j->joint_velocity_const_pointer());
    REG_RESOURCE("jnt-tor-" + post, j->joint_torque_const_pointer());

    // REG_COMMAND ("jnt-cmd-" + post, jnt_reg_res_->command_pointer[j->owner_type()][j->joint_type()]);
  }*/

  auto cfg = MiiCfgReader::instance();
  int count = 0;
  LegType leg = LegType::UNKNOWN_LEG;
  MiiString _leg_tag = Label::make_label(
      Label::make_label(_prefix, "legs"), "leg_" + std::to_string(count));
  while (cfg->get_value(_leg_tag, "leg", leg)) {
    MiiString str;
    cfg->get_value_fatal(_leg_tag, "command", str);
    REG_COMMAND(str, jnt_reg_res_->command[leg]);

    cfg->get_value_fatal(_leg_tag, "td_resource", str);
    REG_RESOURCE(str, td_list_by_type_[leg]->force_data_const_pointer());

    cfg->get_value_fatal(_leg_tag, "pos", str);
    REG_RESOURCE(str, jnt_reg_res_->resource[leg][JntDataType::POS]);

    cfg->get_value_fatal(_leg_tag, "vel", str);
    REG_RESOURCE(str, jnt_reg_res_->resource[leg][JntDataType::VEL]);

    cfg->get_value_fatal(_leg_tag, "tor", str);
    REG_RESOURCE(str, jnt_reg_res_->resource[leg][JntDataType::TOR]);

    _leg_tag = Label::make_label(
        Label::make_label(_prefix, "legs"), "leg_" + std::to_string(++count));
  }

  MiiVector<MiiString> strs;
  MiiString _imu_tag = Label::make_label(_prefix, "imu");
  cfg->get_value_fatal(_imu_tag, "quaternion", strs);
  REG_RESOURCE(strs[0],        imu_reg_res_->orientation);
  if (2 == strs.size()) REG_RESOURCE(strs[1], imu_reg_res_->orientation_cov);

  cfg->get_value_fatal(_imu_tag, "linear_acc", strs);
  REG_RESOURCE(strs[0],     imu_reg_res_->lin_acc);
  if (2 == strs.size()) REG_RESOURCE(strs[1], imu_reg_res_->lin_acc_cov);

  cfg->get_value_fatal(_imu_tag, "angular_vel", strs);
  REG_RESOURCE(strs[0],     imu_reg_res_->ang_vel);
  if (2 == strs.size()) REG_RESOURCE(strs[1], imu_reg_res_->ang_vel_cov);
}

void MiiRobot::supportRegistry() {
  TIMER_INIT

  mii_ctrl_alive_ = true;
  while (mii_ctrl_alive_) {
    /// read joint states
    for (const auto& l : {LegType::FL, LegType::HL, LegType::FR, LegType::HR})
      for (const auto& d : {JntDataType::POS, JntDataType::VEL, JntDataType::TOR})
        for (const auto& j : {JntType::KNEE, JntType::HIP, JntType::YAW})
          (*(jnt_reg_res_->resource[l][d]))(j) = (*jnt_manager_)(l, j, d);
    
    // TODO IMU

    // write
    if (use_mii_control_) {
      for (const auto& l : {LegType::FL, LegType::HL, LegType::FR, LegType::HR}) {
        auto& cmd_ref = *jnt_reg_res_->command[l];
        for (const auto& j : {JntType::KNEE, JntType::HIP, JntType::YAW})
          jnt_manager_->addJointCommand(l, j, cmd_ref(j));
      }
    }

    TIMER_CONTROL(tick_interval_)
  }

}

MiiRobot::~MiiRobot() {
  // LOG_DEBUG << "The deconstructor of MiiRobot is starting to work.";
  mii_ctrl_alive_ = false;
  Master::destroy_instance();
  JointManager::destroy_instance();
  ThreadPool::destroy_instance();

  // destroy the auto_instancor.
  AutoInstanceor::destroy_instance();

  if (jnt_reg_res_) delete jnt_reg_res_;
  if (td_reg_res_)  delete td_reg_res_;
  if (imu_reg_res_) delete imu_reg_res_;
  // LOG_DEBUG << "The deconstructor of MiiRobot almost finished.";
  // Label::printfEveryInstance();
}


bool MiiRobot::start() {
  return (Master::instance()->run() && ThreadPool::instance()->start());
}


void MiiRobot::addJntCmd(const MiiString& name, double command) {
  jnt_manager_->addJointCommand(name, command);
}

void MiiRobot::addJntCmd(LegType _owner, JntType _jnt, double _command) {
  jnt_manager_->addJointCommand(_owner, _jnt, _command);
}

void MiiRobot::getJointNames(MiiVector<MiiString>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_name());
  }
}

void MiiRobot::getJointPositions(MiiVector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_position());
  }
}

void MiiRobot::getJointVelocities(MiiVector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_velocity());
  }
}

void MiiRobot::getJointTorques(MiiVector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_torque());
  }
}

} /* namespace middleware */
