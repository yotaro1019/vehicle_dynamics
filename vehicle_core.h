#ifndef _vehicle_core_
#define _vehicle_core_
#include "inp_init_data.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;

class Vehicle_model{
private:
//shared_ptr
std::shared_ptr<Input_data> inp;
std::shared_ptr<WheeledVehicle> veh;
std::shared_ptr<RigidTerrain> terrain;
std::shared_ptr<ChPathFollowerDriver> driver_follower;
std::shared_ptr<ChWheeledVehicleIrrApp> app;
irr::scene::IMeshSceneNode* ballS;  //sentinel point(driver)
irr::scene::IMeshSceneNode* ballT;  // target point(driver)
ChVector<> driver_pos;
//params
double step_size, tire_step_size;
//function
void initialize();      //initialize vehicle system
void advance(double adv_step_size, double fforce[6] = {0}); //advance vehicle step(adv_step_size : current time step )

void irricht_initialize(double step_size); //initialize irricht

void conv_axis(double array[6]);   //coordinate transformation fo rotation and translatiion direction

public:
void vehicle_initialize();
void vehicle_advance(double fforce[6]); //fforce(fx,fy,fz,mx,my,mz)
};

#endif