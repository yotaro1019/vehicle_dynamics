#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<algorithm>
#include "point_vel_acc.h"

Point_vel_acc::Point_vel_acc(bool c_switch, std::string fname){
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;

    this->read_param(fname);
}

void Point_vel_acc::read_param(std::string fname){
    if(!this->c_switch)
        return;
    
    GetLog() << "read point vel and acc  in cabin\n";
    GetLog() << "input file : " << fname << "\n";

    std::ifstream inp_param_file(fname);
    if(inp_param_file.fail()){
        std::cout << "ERROR unable to open " << fname << "\n";
    }  

    std::string str;
    
    while(getline(inp_param_file,str)){
        

        if(str[0] == '#')
            continue;

        if(str[0] == '!')
            continue;
        
        if(str.empty())
            continue;

        std::stringstream ss;
        std::string name;           

        std::replace(str.begin(), str.end(), '=', ' ');
        std::replace(str.begin(), str.end(), ',', ' ');

        ss << str;
        ss >> name;




    }
    exit(1);
}