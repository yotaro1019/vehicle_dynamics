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

#include"output.h"
#include"restart.h"
#include"exchange_data.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;

class Vehicle_model{
private:
//shared_ptr
std::shared_ptr<Input_data> inp;    //object of input params
std::shared_ptr<WheeledVehicle> veh;    //object of WheeledVehicle
std::shared_ptr<RigidTerrain> terrain;  //object of RigidTerrain

std::shared_ptr<ChPathFollowerDriver> driver_follower;  //object of PathFollower
irr::scene::IMeshSceneNode* ballS;  //sentinel point(driver)
irr::scene::IMeshSceneNode* ballT;  // target point(driver)
ChVector<> driver_pos;

std::shared_ptr<ChWheeledVehicleIrrApp> app;    //object of Irricht

std::shared_ptr<Output> out;
std::shared_ptr<Restart> restart;


//params
double step_size, tire_step_size;

//statud
double current_time;

//function
void setup_system();
void initialize();      //initialize vehicle system
void advance(double adv_step_size, double fforce[6] = {0}); //advance vehicle step(adv_step_size : current time step )

//use realtime rendering (Irrlicht)
void irricht_initialize(double step_size); //Must be called once before real-time visualization to initialize the Irrlicht system
void irricht_advance(double step_size, ChDriver::Inputs driver_inputs);//Calling this function once will update the visualization of Irricht by one step 

//use Povray rendering(PovRay)
std::string pov_dir;
void initialize_pov();
void output_pov(int render_frame);

void disp_current_status(); //Display time and other current data as text

void conv_axis(double array[6]);   //coordinate transformation fo rotation and translatiion direction

public:
void vehicle_initialize();
void vehicle_advance(double fforce[6], Vehicle2Cfd *veh2cfd_data); //fforce(fx,fy,fz,mx,my,mz)
};

#endif