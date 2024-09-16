#ifndef CODE_DEBUG_H
#define CODE_DEBUG_H

#include "topic_info.h"
#include <iomanip>
#include <sstream>
#include <unordered_map>

class CodeDebug {
public:
    CodeDebug();
    void printRobotState(std::unordered_map<int, RobotAttributes> states);

private:


};


#endif
