/*
 * registry.h
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_REGISTRY_H_
#define INCLUDE_REPOSITORY_REGISTRY_H_

#include <foundation/utf.h>
#include <boost/variant.hpp>
#include <Eigen/Dense>

namespace middleware {

#define REG_RESOURCE(_n, _var)  ( middleware::Registry::instance()->registerResource((_n), (_var)) )

#define REG_COMMAND(_n, _var)   ( middleware::Registry::instance()->registerCommand ((_n), (_var)) )

#define GET_RESOURCE(_n, _type) ( middleware::Registry::instance()->resource< _type >(_n) )

#define GET_COMMAND(_n, _type)  ( middleware::Registry::instance()->command< _type > (_n) )

typedef boost::variant<const short*, const int*, const double*,
    const Eigen::VectorXi*, const Eigen::MatrixXi*,
    const Eigen::VectorXd*, const Eigen::MatrixXd*> ResType;

typedef boost::variant<short*, int*, double*,
    Eigen::VectorXi*, Eigen::MatrixXi*,
    Eigen::VectorXd*, Eigen::MatrixXd*> CmdType;

class Registry {
  SINGLETON_DECLARE(Registry)

public:
  bool registerResource(const MiiString&, ResType);
  bool registerCommand(const MiiString&, CmdType);

  ///! The boost static assert fail! so we need split into two methods.
  template<typename _DataType>
  _DataType resource(const MiiString&);
  template<typename _DataType>
  _DataType command(const MiiString&);

public:
  ///! Query the given name whether register in the registry.
  // bool query(const MiiString&);

  ///! Search by the key words and return the results.
  // MiiString search(const MiiString&);

  ///! print the all of registry.
  void print();

protected:
  MiiMap<MiiString, ResType>   res_origin_;
  MiiMap<MiiString, CmdType>   cmd_origin_;
};

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template <typename _DataType>
_DataType Registry::resource(const MiiString& _res_name) {
  if (res_origin_.end() == res_origin_.find(_res_name)) {
    return _DataType(nullptr);
  }
  auto var_data = res_origin_[_res_name];
  assert(var_data.type() == typeid(_DataType));

  return boost::get<_DataType>(var_data);
}

template <typename _DataType>
_DataType Registry::command(const MiiString& _res_name) {
  if (cmd_origin_.end() != cmd_origin_.find(_res_name)) {
    return _DataType(nullptr);
  }

  auto var_cmd = cmd_origin_[_res_name];
  assert(var_cmd.type() == typeid(_DataType));

  return boost::get<_DataType>(var_cmd);
}

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_REGISTRY_H_ */
