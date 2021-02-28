#ifndef _vehicle_core_
#define _vehicle_core_
#include "inp_init_data.h"
#include "fforce_map.h"
#include "calculation_mode.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"

#include"output.h"
#include"restart.h"
#include"exchange_data.h"
#include"veh_visualization.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;


class Vehicle_model{
private:
//shared_ptr
std::shared_ptr<Input_data> inp;    //object of input params
std::shared_ptr<WheeledVehicle> veh;    //object of WheeledVehicle
std::shared_ptr<WheeledVehicle> tlr;    //object of WheeledVehicle (Trailer)
std::shared_ptr<chrono::ChLinkLockSpherical> m_puller;  ///< joint between trailer and pulling vehicle

std::shared_ptr<RigidTerrain> terrain;  //object of RigidTerrain

std::shared_ptr<ChPathFollowerDriver> driver_follower;  //object of PathFollower
ChVector<> driver_pos;

std::shared_ptr<Exchange_data> exc_data;

//visualization
std::shared_ptr<Veh_Visualization> veh_viz; 

std::shared_ptr<Output> out;
std::shared_ptr<Restart> restart;

std::shared_ptr<FForce_map> fmap;       //calc fforce from aero-coef map
//params
double step_size, tire_step_size;

//status
double current_time;
int current_step;



enum Calculation_mode calc_mode;
enum Calculation_section calc_sec;

//function
void setup_system();
void initialize();      //initialize vehicle system
void advance(double adv_step_size, Cfd2Vehicle *cfd2veh_data); //advance vehicle step(adv_step_size : current time step )



void disp_current_status(); //Display time and other current data as text

void conv_axis(double array[6]);   //coordinate transformation fo rotation and translatiion direction


public:
//coupling
void vehicle_initialize();
void vehicle_advance(Cfd2Vehicle *cfd2veh_data, Vehicle2Cfd *veh2cfd_data); 

//stand_alone
void vehicle_initialize_stand_alone();
void vehicle_advance_stand_alone();
};

#endif