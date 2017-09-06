/*
 * utf.h
 * Uniform Type define File defines all of the typedef or macro.
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_UTF_H_
#define INCLUDE_SYSTEM_UTILS_UTF_H_

#ifndef USING_STD_IO
#include <glog/logging.h>
#include <glog/log_severity.h>
#endif

#include <vector>
#include <string>
#include <map>
#include <system/platform/protocol/proto/dragon.pb.h>
// #include <iostream>

namespace middleware {
template<class _Type>
using MiiVector =  std::vector<_Type>;

template<class _Key, class _Value>
using MiiMap    =  std::map<_Key, _Value>;

using MiiString = std::string;

#ifndef USING_STD_IO
#define LOG_INFO      LOG(INFO)     << "\t"
#define LOG_WARNING   LOG(WARNING)  << "\t"
#define LOG_ERROR     LOG(ERROR)    << "\t"
#define LOG_FATAL     LOG(FATAL)    << "\t"
#else
#define LOG_INFO      std::cout    << "\t"
#define LOG_WARNING   std::cout    << "\t"
#define LOG_ERROR     std::cout    << "\t"
#define LOG_FATAL     std::cout    << "\t"
#endif
} /* namespace middleware */



#endif /* INCLUDE_SYSTEM_UTILS_UTF_H_ */
