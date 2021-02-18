#ifndef _SET_VALUE_
#define _SET_VALUE_
#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono{
namespace vehicle{

class Set_value{
    protected:
    //set function
    std::string Set_str_value(std::stringstream &ss);
    int Set_int_value(std::stringstream &ss);
    double Set_double_value(std::stringstream &ss);
    bool Set_bool_value(std::stringstream &ss);
    ChVector<> Set_ChVector(std::stringstream &ss);
    ChQuaternion<> Set_ChQuaternion(std::stringstream &ss);
    std::vector<std::string> Set_str_vec(std::stringstream &ss);

};




}
}
#endif