/**
 * @file ControlHelper.hpp
 *
 * @brief This file contains all the ControlHelper class definitions
 *
 */

#ifndef CONTROLHELPER_HPP
#define CONTROLHELPER_HPP

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

#include "data_sets.h"
#include "InertialSense.h"

#include "TopicHelper.h"

#include <chrono>
#include <memory>
#include <cassert>
 
class ControlHelper
{
public:
    ControlHelper();
    ~ControlHelper() { terminate(); }

    void someFunction();
    void terminate();

private:
    // Private members (accessible only within the class)
    int privateVariable;
 
};
 
#endif //CONTROLHELPER_HPP
 