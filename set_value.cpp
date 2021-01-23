#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<algorithm>
#include"inp_init_data.h"

namespace chrono{
namespace vehicle{
//set function
std::string Set_value::Set_str_value(std::stringstream &ss){
    std::string str;
    ss >> str;
    return str;
}

int Set_value::Set_int_value(std::stringstream &ss){
    std::string str;
    int num;
    ss >> str;
    num = std::stoi(str);
    return num;  
}

double Set_value::Set_double_value(std::stringstream &ss){
    std::string str;
    double dbl;
    ss >> str;
    dbl = std::stod(str);
    return dbl;    
}

bool Set_value::Set_bool_value(std::stringstream &ss){
    std::string str;
    bool active;
    ss >> str;

    if(str == "true"){
        return true;
    }else if(str == "false"){
        return false;
    }else{
        GetLog() << "This is not bool type  \t input value = " << str << "\n";
    }
}

ChVector<> Set_value::Set_ChVector(std::stringstream &ss){
    ChVector<double> chv;
    std::string st_x,st_y,st_z;

    ss >> st_x;
    ss >> st_y;
    ss >> st_z; 
  
    chv.Set(std::stod(st_x), std::stod(st_y), std::stod(st_z));
    return chv;
}

ChQuaternion<> Set_value::Set_ChQuaternion(std::stringstream &ss){
    ChQuaternion<double> chq;
    std::string st_e1, st_e2, st_e3, st_e4;
    
    ss >> st_e1;
    ss >> st_e2;
    ss >> st_e3;
    ss >> st_e4;
    chq.Set(std::stod(st_e1), std::stod(st_e2), std::stod(st_e3), std::stod(st_e4));
    return chq;
}

std::vector<std::string> Set_value::Set_str_vec(std::stringstream &ss){
    std::vector<std::string> str_vec;
    GetLog() << str_vec.size() << "\n";
    while(!ss.eof()){
        std::string tmp;
        ss >> tmp;        
        str_vec.push_back(tmp);
    }
    str_vec.pop_back(); 
    return str_vec;
}

}
}