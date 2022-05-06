#ifndef _driver_model_controller_
#define _driver_model_controller_
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "inp_init_data.h"


using namespace chrono;
using namespace chrono::vehicle;

class Driver_model_controller{
public:
//valiables for PathFollowerDriver
std::shared_ptr<ChPathFollowerDriver> path_follower_driver; //object of PathFollower
std::shared_ptr<ChBezierCurve> path;

//constructor
Driver_model_controller();

void setup_path_follower_driver(Input_data &inp, WheeledVehicle &veh);

ChDriver::Inputs GetInputs();
void Synchronize(double time);
void Advance(double adv_step_size);

std::shared_ptr<ChPathFollowerDriver> Get_path_follower_driver(){ return this->path_follower_driver; }; 


};

#endif