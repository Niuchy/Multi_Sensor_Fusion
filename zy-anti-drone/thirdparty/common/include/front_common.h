#ifndef _FRONT_COMMON_H_
#define _FRONT_COMMON_H_

#include <string>
#include <functional>

using FrontendDisplayCallback = std::function<void(std::string &)>;

void RegisterFrontendDisplayCallBack(FrontendDisplayCallback func);
void FrontendDisplay(std::string &s);

#define FRONT_DISPLAY_TYPE_HEALTH_STATUS 0

#endif
