#include "driver_model_controller.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/ChVehicleModelData.h"


Driver_model_controller::Driver_model_controller(){
    GetLog() << "Call constructor @Driver model controller\n";
}

void Driver_model_controller::setup_path_follower_driver(Input_data &inp, WheeledVehicle &veh){
    this->path = ChBezierCurve::read(vehicle::GetDataFile(inp.Get_path_txt_fname()));

    path_follower_driver.reset(new ChPathFollowerDriver (veh, path, "follow_path", inp.Get_target_speed()) );
    path_follower_driver->GetSteeringController().SetLookAheadDistance(5);
    path_follower_driver->GetSteeringController().SetGains(0.8, 0, 0);
    path_follower_driver->GetSpeedController().SetGains(0.4, 0, 0);
    path_follower_driver->Initialize();
}

ChDriver::Inputs Driver_model_controller::GetInputs(){
    ChDriver::Inputs path_follower_inputs = path_follower_driver->GetInputs();
    
    ChDriver::Inputs inputs = path_follower_inputs;
    return inputs;
}

void Driver_model_controller::Synchronize(double time){
    path_follower_driver->Synchronize(time);
}

void Driver_model_controller::Advance(double adv_step_size){
    path_follower_driver->Advance(adv_step_size);    
}


