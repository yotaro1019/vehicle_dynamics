#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"

void Output::write(double time, WheeledVehicle &veh){

    GetLog() << "time = " << time << "\n";

}