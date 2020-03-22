#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<iomanip>
#include"outputlist.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

using namespace chrono;
using namespace chrono::vehicle;

void Chassis_vel_fout::initialize(bool c_switch, const std::string fname)
{
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;
    


    fout.reset(new std::ofstream(fname.c_str()) );
    check_file_status(fout, fname);
    GetLog() << "!\n";
    char header[500];
    sprintf(header, "%12s%12s%12s%12s%12s%12s%12s%12s%12s%12s%12s%12s", "time", "x", "y", "z", "vel", "u", "v", "w", "roll", "pitch", "yaw", "grobal_yaw");
    *fout << header << "\n";

}

void Chassis_vel_fout::write(double time){
    if(!c_switch)
        return;
    GetLog() << "!!\n";
    char output_value[500];
    sprintf(output_value,"%12.5f",  time );
    *fout << output_value << "\n";
}