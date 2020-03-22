#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"
#include"outputlist.h"

Chassis_vel_fout chassis_log;

Output::Output(Input_data &inp){
    GetLog() << "call output constructor\n";
    chassis_log.initialize(inp.Get_chassis_com_bool(), "./" + inp.Get_chassis_COM_fname());
}
void Output::write(double time, WheeledVehicle &veh){

    GetLog() << " Output::write time = " << time << "\n";
    chassis_log.write(time);
}