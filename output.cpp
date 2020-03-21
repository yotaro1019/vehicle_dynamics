#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"

Output::Output(Input_data &inp){
    GetLog() << "call output constructor\n";

}
void Output::write(double time, WheeledVehicle &veh){

    GetLog() << "time = " << time << "\n";

}