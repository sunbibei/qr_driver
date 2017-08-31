/*
 * utils.h
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_LOG_H_
#define INCLUDE_SYSTEM_UTILS_LOG_H_

#include <glog/logging.h>
#include <glog/log_severity.h>

// #include <iostream>

namespace middleware {

#define LOG_INFO      LOG(INFO)     << "\t"
#define LOG_WARNING   LOG(WARNING)  << "\t"
#define LOG_ERROR     LOG(ERROR)    << "\t"
#define LOG_FATAL     LOG(FATAL)    << "\t"

/*#define LOG_INFO      std::cout    << "\t"
#define LOG_WARNING   std::cout    << "\t"
#define LOG_ERROR     std::cout    << "\t"
#define LOG_FATAL     std::cout    << "\t"*/

} /* namespace middleware */



#endif /* INCLUDE_SYSTEM_UTILS_LOG_H_ */
