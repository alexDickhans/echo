#pragma once

#define Log_Desc(desc, msg) printf("%s %lu: %s\n", std::string(desc).c_str(), pros::millis(), std::string(msg).c_str());
#define Log(msg) Log_Desc(std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__), msg);
