/*
 * manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_RESOURCE_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_RESOURCE_MANAGER_H_

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

public:
  typedef typename std::vector<_Resource*>::iterator  iterator;
  typedef typename std::vector<_Resource*>::reference reference;
  // typedef typename MiiVector<_Resource*>::iterator iterator;
  /**
   * @brief Return the number of the registered resource.
   */
  virtual unsigned int   size()  { return res_list_.size(); };
  virtual bool empty() { return res_list_.empty(); };
  virtual reference operator[](int i) { return res_list_[i]; };
  /**
   * @brief These methods offer the ability of the range-based loop for classes.
   */
  virtual iterator begin() { return res_list_.begin(); };
  virtual iterator end()   { return res_list_.end();   };

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

#endif /* INCLUDE_SYSTEM_ROBOT_RESOURCE_MANAGER_H_ */