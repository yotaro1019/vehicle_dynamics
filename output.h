#ifndef _output_log_
#define _output_log_
#include<iostream>
#include<string>
#include<fstream>
#include"inp_init_data.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/PacejkaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"


using namespace chrono;
using namespace chrono::vehicle;
class  CH_VEHICLE_API  Output{    
  
  public:
    Output(Input_data &inp, WheeledVehicle &veh);
    void write(double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr);  
};

#endif