#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"
#include"outputlist.h"

Chassis_vel_fout chassis_log;
Driver_fout dvr_log;
Output::Output(Input_data &inp){
    chassis_log.initialize(inp.Get_chassis_com_bool(), GetChronoOutputPath() + inp.Get_chassis_COM_fname());
    dvr_log.initialize(inp.Get_driver_input_bool(), GetChronoOutputPath() + inp.Get_driver_input_fname());
}
void Output::write(double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr){
    chassis_log.write(time, veh);
    dvr_log.write(time, dvr);
}