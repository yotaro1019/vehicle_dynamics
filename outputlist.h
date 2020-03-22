#ifndef _outputlist_log_
#define _outputlist_log_
#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"baseout.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"


#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/PacejkaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
using namespace chrono;
using namespace chrono::vehicle;


class Chassis_vel_fout : public Baseout{
  public:
    Chassis_vel_fout() { Baseout(); };
    void initialize(bool c_switch, const std::string fname);
    void write(double time, WheeledVehicle &veh);  
};

class Driver_fout : public Baseout{
  public:
    Driver_fout() { Baseout(); };
    void initialize(bool c_switch, const std::string fname);
    void write(double time, ChPathFollowerDriver &dvr);
};

#endif