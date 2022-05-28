#include "driver_model_controller.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/ChVehicleModelData.h"


Driver_model_controller::Driver_model_controller(){
    //initialize final_driver_input
    final_driver_input.m_steering = 0.0;
    final_driver_input.m_throttle = 0.0;
    final_driver_input.m_braking = 0.0;

}

void Driver_model_controller::setup_path_follower_driver(Input_data &inp, WheeledVehicle &veh){
    this->path = ChBezierCurve::read(vehicle::GetDataFile(inp.Get_path_txt_fname()));

    GetLog() << "STR\t" <<  vehicle::GetDataFile(inp.Get_steering_JSON_fname()) << "\n";
    GetLog() << "SPD\t" << vehicle::GetDataFile(inp.Get_speed_JSON_fname()) << "\n";
    path_follower_driver.reset(new ChPathFollowerDriver (veh, vehicle::GetDataFile(inp.Get_steering_JSON_fname()), 
        vehicle::GetDataFile(inp.Get_speed_JSON_fname()), path, "follow_path", inp.Get_target_speed()) );
    path_follower_driver->Initialize();
    //exit(1);
}

ChDriver::Inputs Driver_model_controller::GetInputs(){
    return this->final_driver_input;
}

void Driver_model_controller::SetInputs(double steering, double throttle, double braking){
    this->final_driver_input.m_steering = ChClamp(steering, -1.0, 1.0);
    this->final_driver_input.m_throttle = ChClamp(throttle, 0.0, 1.0);
    this->final_driver_input.m_braking = ChClamp(braking, 0.0, 1.0);
}

void Driver_model_controller::Synchronize(double time){
    if(activate_path_follower_driver){
        path_follower_driver->Synchronize(time);
    }
}

void Driver_model_controller::Advance(double adv_step_size){
    this->Advance_all_driver_model(adv_step_size); 
    this->final_driver_input = this->mediation();  //調停
}

void Driver_model_controller::Advance_all_driver_model(double adv_step_size){
    if(activate_path_follower_driver){
        path_follower_driver->Advance(adv_step_size); 
    }
}

//調停
ChDriver::Inputs Driver_model_controller::mediation(){
    return path_follower_driver->GetInputs();
}

void Driver_model_controller::ExportPathPovray(std::string pov_dir){
    if(activate_path_follower_driver){
        path_follower_driver->ExportPathPovray(pov_dir);
    }
}

void Driver_model_controller::reset(WheeledVehicle &veh){
    if(activate_path_follower_driver){
        
        path_follower_driver->Reset();
        
        //steering controller
        path_follower_driver->GetSteeringController().CalcTargetLocation(); 

        //speed controler
    
    }  
}


ChVector<> Driver_model_controller::GetSentinelLocation(){
    
    ChVector<> pS;

    if(activate_path_follower_driver){
        pS = path_follower_driver->GetSteeringController().GetSentinelLocation();
    }else{
        pS.SetNull();
    }
    return pS;

}
ChVector<> Driver_model_controller::GetTargetLocation(){
    ChVector<> pT;

    if(activate_path_follower_driver){    
        pT = path_follower_driver->GetSteeringController().GetTargetLocation();    
    }else{
        pT.SetNull();
    }
    return pT;
}

