/*
 * noncopyable.h
 *
 *  Created on: Jul 19, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_UTIL_NONCOPYABLE_H_
#define INCLUDE_MIDDLEWARE_UTIL_NONCOPYABLE_H_

namespace middleware {

class NonCopyable {
public:
  NonCopyable() {};
  ~NonCopyable() {};
private:
  NonCopyable(const NonCopyable&) { };
  NonCopyable& operator=(const NonCopyable&) {return *this;};
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_UTIL_NONCOPYABLE_H_ */
