#ifndef _output_log_
#define _output_log_
#include<iostream>
#include<string>
#include<fstream>
#include"inp_init_data.h"
#include"exchange_data.h"
#include"driver_model_controller.h"

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
#include "chrono_thirdparty/filesystem/path.h"
#include<memory>

using namespace chrono;
using namespace chrono::vehicle;
class  CH_VEHICLE_API  Output{    
  private:
    bool info_1way_bool;
  public:
    Output(Input_data &inp, WheeledVehicle &veh);

    void initialize_veh_status(Input_data &inp, WheeledVehicle &veh);
    void initialize_1way_info(Input_data &inp, WheeledVehicle &veh);
    void initialize_fforce_info(Input_data &inp);
    void write(int step, double time, WheeledVehicle &veh, Driver_model_controller dvr, RigidTerrain &ter, Cfd2Vehicle &c2v, Vehicle2Cfd &v2c);  
    void write_veh_status(int step, double time, WheeledVehicle &veh, Driver_model_controller dvr, RigidTerrain &ter);
    void write_1way_info(int step, double time, WheeledVehicle &veh, Vehicle2Cfd &v2c);
    void write_fforce(int step, double time, Cfd2Vehicle &c2v);  

    void restart(int restart_step);
};

#endif