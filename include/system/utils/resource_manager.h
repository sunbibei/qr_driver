/*
 * manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_RESOURCE_MANAGER_H_
#define INCLUDE_SYSTEM_UTILS_RESOURCE_MANAGER_H_

#include <vector>

namespace middleware {

template<class _Resource>
class ResourceManager {
public:
  virtual ~ResourceManager() {
    // Should not to dealloc
    /*for (auto res : res_list_) {
      if (nullptr != res) {
        delete res;
        res = nullptr;
      }
    }*/
  }

  virtual void add(_Resource* _res) {
    res_list_.push_back(_res);
  }

  virtual void remove(_Resource* _res) {
    /*for (auto res : res_list_) {
      if (res == _res) res_list_.erase(res);
    }*/
    for (auto iter = res_list_.begin(); iter != res_list_.end();) {
      if (_res == *iter) iter = res_list_.erase(iter);
      else  ++iter;
    }
  }

protected:
  ResourceManager() { };

  // store all of the resource
  std::vector<_Resource*> res_list_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_RESOURCE_MANAGER_H_ */
