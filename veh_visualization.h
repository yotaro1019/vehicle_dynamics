#ifndef _vehicle_visualization_
#define _vehicle_visualization_

#include "calculation_mode.h"
#include "inp_init_data.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;

class Veh_Visualization{
public:
    Veh_Visualization(enum Calculation_mode calc_mode, Input_data &inp, WheeledVehicle &veh, RigidTerrain &terrain, ChPathFollowerDriver &driver_follower);
    void viz_advance(double step_dt, double time, int step, WheeledVehicle &veh,  ChPathFollowerDriver &driver_follower);
    
private:
    bool irricht_switch;
    double step_dt;

    //irricht valiables
    std::shared_ptr<ChWheeledVehicleIrrApp> app;    //object of Irricht   
    irr::scene::IMeshSceneNode* ballS;  //sentinel point(driver)
    irr::scene::IMeshSceneNode* ballT;  // target point(driver)
    void irricht_initialize(Input_data &inp, WheeledVehicle &veh);
    void irricht_advance(double step_dt, ChPathFollowerDriver &driver_follower);

    //POV-Ray valiables
    bool pov_switch;
    std::string pov_dir;
    int  pov_out_itvl;
    void povray_initialize(RigidTerrain &terrain, ChPathFollowerDriver &driver_follower);
    void output_pov(int step, WheeledVehicle &veh);

};


#endif