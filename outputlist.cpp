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

void Driver_fout::initialize(bool c_switch, const std::string fname)
{
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;
    
    fout.reset(new std::ofstream(fname.c_str()) );
    check_file_status(fout, fname);
    GetLog() << "!\n";
    char header[500];
    sprintf(header, "%12s%12s%12s%12s", "time", "steering", "throttle", "brake");
    *fout << header << "\n";

}

void Driver_fout::write(double time, ChPathFollowerDriver &dvr){
    if(!c_switch)
        return;

    char output_value[500];
    sprintf(output_value,"%12.5f%12.5f%12.5f%12.5f",  time, dvr.GetSteering(), dvr.GetThrottle(), dvr.GetBraking() );
    *fout << output_value << "\n";

}

void Powertrain_fout::initialize(bool c_switch, const std::string fname)
{
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;
    
    fout.reset(new std::ofstream(fname.c_str()) );
    check_file_status(fout, fname);

    char header[500];
    sprintf(header, "%12s%12s%12s%12s%12s%12s%12s%12s", "time", "engine_spd", "engine_trq", "TC_slipage", "TC_in_trq", "TC_out_trq", "TM_gear", "out_trq");
    *fout << header << "\n";

}

void Powertrain_fout::write(double time, ChPowertrain &pt){
    if(!c_switch)
        return;   
    char output_value[500];

    sprintf(output_value,"%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12d%12.5f",  time, pt.GetMotorSpeed(), pt.GetMotorTorque(), pt.GetTorqueConverterSlippage(), pt.GetTorqueConverterInputTorque(), pt.GetTorqueConverterOutputTorque(), pt.GetCurrentTransmissionGear(), pt.GetOutputTorque() );

    *fout << output_value << "\n";    
}

void Tire_fout::initialize(bool c_switch, const std::string fname){
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;
    
    fout.reset(new std::ofstream(fname.c_str()) );
    check_file_status(fout, fname);

    char header[500];
    sprintf(header, "%12s%17s%17s%17s%17s%17s%17s%17s%17s%17s%17s%17s%17s%17s", "time","fx","fy","fz","mx","my","mz","slip","Longslip","camber","posX", "posY", "posZ", "daflection");
    *fout << header << "\n";

}

void Tire_fout::write(double time, ChWheel &wheel , RigidTerrain &terrain){
    if(!c_switch)
        return; 

    std::shared_ptr<ChTire> tire = wheel.GetTire();
    TerrainForce terrain_force = tire->ReportTireForce(&terrain);
    ChVector<> force_global = terrain_force.force;  //force @global frame
    ChVector<> moment_global = terrain_force.moment;//moment @global frame

    //convert force and moment (form grobal to wheel local)
    ChQuaternion<> rot_q = wheel.GetState().rot;
    ChVector<> force_loc, moment_loc;
    ChVector<> yaxis = rot_q.GetYaxis();
    double theta = atan( yaxis.y() / yaxis.x() );

    //convert force
    double fx_loc = force_global.x()*cos(theta) + force_global.y()*sin(theta);
    double fy_loc = -force_global.x()*sin(theta) + force_global.y()*cos(theta);
    force_loc.Set(fx_loc, fy_loc, force_global.z());

    //convert moment
    double mx_loc = moment_global.x()*cos(theta) + moment_global.y()*sin(theta);
    double my_loc = -moment_global.x()*cos(theta) + moment_global.y()*cos(theta);
    moment_loc.Set(mx_loc, my_loc, moment_global.z());


    double slip = tire->GetSlipAngle();
    double lng_slip = tire->GetLongitudinalSlip();
    double cmb_angle = tire->GetCamberAngle();
    double deflection = tire->GetDeflection();

    ChVector<> pos = wheel.GetPos();
    char output_value[500];
    sprintf(output_value,"%12.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f%17.5f",  time, force_loc.x(), force_loc.y(), force_loc.z(),
    moment_loc.x(), moment_loc.y(), moment_loc.z(), slip, lng_slip, cmb_angle, pos.x(), pos.y(), pos.z(), deflection );    
    *fout << output_value << "\n";
    
}


//1WAY-info
void Vehicle2CFD_info::initialize(bool c_switch, const std::string fname){
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;

    fout.reset(new std::ofstream(fname.c_str()) );
    check_file_status(fout, fname);  
}

void Vehicle2CFD_info::write(double time, double comp1[], double comp2[]){
    if(!c_switch)
        return; 
        
    char output_value[500];
    sprintf(output_value,"%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f%12.5f",  time, comp1[0], comp1[1], comp1[2],
    comp2[0], comp2[1], comp2[2]);
    *fout << output_value << "\n";
}
