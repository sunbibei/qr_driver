/*
 * manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_UTIL_RESOURCE_MANAGER_H_
#define INCLUDE_MIDDLEWARE_UTIL_RESOURCE_MANAGER_H_

#include <vector>

namespace middleware {

template<class _Resource>
class ResourceManager {
public:
  virtual ~ResourceManager() {
    for (auto res : res_list_) {
      if (nullptr != res) {
        delete res;
        res = nullptr;
      }
    }
  }

  static ResourceManager<_Resource>* instance() {
    if (nullptr == instance_) instance_ = new ResourceManager<_Resource>;

    return instance_;
  }

  virtual void add(_Resource* _res) {
    res_list_.push_back(_res);
  }

  virtual void remove(_Resource* _res) {
    for (auto res : res_list_) {
      if (res == _res) res_list_.erase(res);
    }
  }

protected:
  ResourceManager() { };
  static ResourceManager<_Resource>* instance_;

  // store all of the resource
  std::vector<_Resource*> res_list_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_UTIL_RESOURCE_MANAGER_H_ */
