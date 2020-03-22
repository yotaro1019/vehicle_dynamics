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
    sprintf(header, "%12s%12s%12s%12s%12s%12s%12s%12s%12s%12s%12s%12s", "time", "x", "y", "z", "vel", "u", "v", "w", "roll", "pitch", "yaw", "yaw_2D");
    *fout << header << "\n";

}

void Chassis_vel_fout::write(double time, WheeledVehicle &veh){
    if(!c_switch)
        return;
    
    ChVector<> com_pos = veh.GetVehicleCOMPos();
    double vel = veh.GetVehicleSpeedCOM();
    ChVector<> vel_axis = veh.GetVehiclePointVelocity(com_pos);
    ChQuaternion<> angle_q = veh.GetVehicleRot();
    ChVector<> angle_euler = angle_q.Q_to_Euler123();
    ChVector<> angle_q_xaxis = angle_q.GetXaxis();
    double yaw_2D = atan( angle_q_xaxis.y() / angle_q_xaxis.x() );
    

    char output_value[500];
    sprintf(output_value,"%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f",  time, com_pos.x(), com_pos.y(), com_pos.z(),
    vel, vel_axis.x(), vel_axis.y(), vel_axis.z(), angle_euler.x(), angle_euler.y(), angle_euler.z(), yaw_2D );
    *fout << output_value << "\n";
}