#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"
#include"outputlist.h"

Chassis_vel_fout chassis_log;

Output::Output(Input_data &inp){

}
void Output::write(double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr){

    chassis_log.write(time, veh);
}