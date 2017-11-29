/*
 * registry.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#include <repository/registry.h>

namespace middleware {

SINGLETON_IMPL(Registry)


Registry::Registry() {
  ;
}

Registry::~Registry() {
  ;
}

bool Registry::registerResource(const MiiString& _n, ResType _handle) {
  if (res_origin_.end() != res_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table."
        << ", now it will be replaced.";
  }

  res_origin_[_n] = _handle;
  return true;
}

bool Registry::registerCommand(const MiiString& _n, CmdType _handle) {
  if (cmd_origin_.end() != cmd_origin_.find(_n)) {
    LOG_WARNING << "The named command '" << _n << "' has registered in the command table."
        << ", now it will be replaced.";
  }

  cmd_origin_[_n] = _handle;
  return true;
}

inline MiiString getTypeName(const ResType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(const int*) == t.type()) {
    return "int     ";
  } else if (typeid(const double*) == t.type()) {
    return "double  ";
  } else if (typeid(const Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(const Eigen::MatrixXi*) == t.type()) {
    return "matrixXi";
  } else if (typeid(const Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else if (typeid(const Eigen::MatrixXd*) == t.type()) {
    return "matrixXd";
  } else {
    return t.type().name();
  }
}

inline MiiString getTypeName(const CmdType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(int*) == t.type()) {
    return "int     ";
  } else if (typeid(double*) == t.type()) {
    return "double  ";
  } else if (typeid(Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(Eigen::MatrixXi*) == t.type()) {
    return "matrixXi";
  } else if (typeid(Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else if (typeid(Eigen::MatrixXd*) == t.type()) {
    return "matrixXd";
  } else {
    return t.type().name();
  }
}

///! print the all of registry.
void Registry::print() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "The size of resource Registry: " << res_origin_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "COUNT\tNAME\t\tTYPE\t\tADDR";
    int count = 0;
    for (const auto& l : res_origin_) {
      LOG_INFO << count++ << "\t" << l.first << "\t" << getTypeName(l.second)
          << "  " << l.second;
    }
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "The size of command Registry: " << cmd_origin_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "COUNT\tNAME\t\tTYPE\t\tADDR";
    count = 0;
    for (const auto& l : cmd_origin_) {
      LOG_INFO << count++ << "\t" << l.first << "\t" << getTypeName(l.second)
          << "  " << l.second;
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
}

} /* namespace middleware */
