// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke, Yotaro Nomoto
// =============================================================================
//
// Demonstration of a steering path-follower PID controller with two alternatives.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "inp_init_data.h"
#include "vehicle_core.h"
#include"exchange_data.h"

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

void Vehicle_model::setup_system(){
    inp.reset(new Input_data("vehicle_params.inp") ); //read params

    filesystem::path current_dir(filesystem::path().getcwd());
    const std::string current_dir_path = current_dir.str();
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);
    chrono::vehicle::SetDataPath(current_dir_path + "/" + inp->Get_inp_dir_name() + "/");    
    SetChronoOutputPath("./" + inp->Get_out_dir_name() + "/");

}

void Vehicle_model::initialize(){
     //==========================================
     //setup params
     step_size = inp->Get_coupling_dt();
     // ------------------------------
     // Create the vehicle and terrain
     // ------------------------------
     // Create the vehicle system
     veh.reset(new WheeledVehicle (vehicle::GetDataFile(inp->Get_vehicle_JSON_fname()) ));
     veh->Initialize(ChCoordsys<>(inp->Get_vehicle_init_loc(), inp->Get_vehicle_init_rot()));
     ////veh->GetChassis()->SetFixed(true);
     veh->SetChassisVisualizationType(inp->Get_chassis_viz_type());
     veh->SetSuspensionVisualizationType(inp->Get_parts_viz_type());
     veh->SetSteeringVisualizationType(inp->Get_parts_viz_type());
     veh->SetWheelVisualizationType(inp->Get_wheel_viz_type());

     // Create the ground
     terrain.reset(new RigidTerrain(veh->GetSystem(), vehicle::GetDataFile(inp->Get_terrain_JSON_fname())) );
     // Create and initialize the powertrain system
     std::shared_ptr<ChPowertrain> powertrain = ReadPowertrainJSON( vehicle::GetDataFile(inp->Get_powertrain_JSON_fname()) ); 
     veh->InitializePowertrain(powertrain);

     //create and initialize the tires
     {int naxle = 0;
     int ntire_file = inp->Get_ntire_JSON();
         for (std::shared_ptr< ChAxle > axle : veh->GetAxles()) {
             std::string tire_fname;
             if(ntire_file == 1){
                 tire_fname = inp->Get_tire_JSON_fnames(0);
             }else if(ntire_file == 2){
                 if(naxle ==0){
                     tire_fname = inp->Get_tire_JSON_fnames(0);
                 }else{
                     tire_fname = inp->Get_tire_JSON_fnames(1);
                 }
             }else{
                 if(veh->GetNumberAxles() > naxle){
                     tire_fname = inp->Get_tire_JSON_fnames(naxle);
                 }
             }
             GetLog() << "naxle = " << naxle << "\t" << tire_fname << "\n";
             
             for (std::shared_ptr< ChWheel > wheel : axle->GetWheels()){
                 std::shared_ptr<ChTire> tire = ReadTireJSON( vehicle::GetDataFile(tire_fname) );
                 veh->InitializeTire(tire,wheel, inp->Get_tire_viz_type());
             }

             //each side has dual tire
             naxle++;
         }
     }

     // ----------------------
     // Create the Bezier path
     // ----------------------

     // From data file
     std::shared_ptr<ChBezierCurve> path = ChBezierCurve::read(vehicle::GetDataFile(inp->Get_path_txt_fname()));

     driver_pos = veh->GetChassis()->GetLocalDriverCoordsys().pos;


     // -------------------------
     // Create the driver systems
     // -------------------------
     driver_follower.reset(new ChPathFollowerDriver (*veh, path, "follow_path", inp->Get_target_speed()) );
     driver_follower->GetSteeringController().SetLookAheadDistance(5);
     driver_follower->GetSteeringController().SetGains(0.8, 0, 0);
     driver_follower->GetSpeedController().SetGains(0.4, 0, 0);
     driver_follower->Initialize();
     
     //initialize irricht
     irricht_initialize(step_size);
     //initialize povray
     initialize_pov();
    }

void Vehicle_model::advance(double adv_step_size, Cfd2Vehicle *cfd2veh_data){
   

    ChVector<> act_fforce(cfd2veh_data->chassis_fforce[0], cfd2veh_data->chassis_fforce[1], cfd2veh_data->chassis_fforce[2]); 
    ChVector<> act_fmoment(cfd2veh_data->chassis_fmoment[0], cfd2veh_data->chassis_fmoment[1], cfd2veh_data->chassis_fmoment[2]); 
    GetLog() << "act_fforce = " << act_fforce.x() << " " << act_fforce.y() << " " << act_fforce.z() << "\n";
    GetLog() << "act_fmoment = " << act_fmoment.x() << " " << act_fmoment.y() << " " << act_fmoment.z() << "\n\n";
    // Extract system state
    double time = veh->GetSystem()->GetChTime();
    ChVector<> acc_CG = veh->GetChassisBody()->GetPos_dtdt();
    ChVector<> acc_driver = veh->GetVehicleAcceleration(driver_pos);
    // Driver inputs
    ChDriver::Inputs driver_inputs = driver_follower->GetInputs();

    // Update modules (process inputs from other modules)
    driver_follower->Synchronize(time);
    terrain->Synchronize(time);
    veh->Synchronize(time, driver_inputs, *terrain, act_fforce, act_fmoment);

    // Advance simulation for one timestep for all modules
    driver_follower->Advance(adv_step_size);
    terrain->Advance(adv_step_size);
    veh->Advance(adv_step_size);

    //visualization
    irricht_advance(adv_step_size, driver_inputs);
    if(current_step%inp->Get_itvl_povray() == 0)
        output_pov(current_step/inp->Get_itvl_povray());

    current_time += adv_step_size;
    current_step ++;
    
}


//use realtime rendering (Irrlicht)
//Must be called once before real-time visualization to initialize the Irrlicht system
void Vehicle_model::irricht_initialize(double step_size){
    if(!inp->Get_use_irricht())
        return;

     app.reset(new ChWheeledVehicleIrrApp(veh.get(), L"Steering PID Controller Demo", irr::core::dimension2d<irr::u32>(800, 640)) );
     
     app->SetHUDLocation(500, 20);
     app->SetSkyBox();
     app->AddTypicalLogo();
     app->AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
     app->AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
     //app->EnableGrid(false);
     app->SetChaseCamera(inp->Get_cam_trackPoint(), 6.0, 0.5);

     app->SetTimestep(step_size);

     // Visualization of controller points (sentinel & target)
     ballS = app->GetSceneManager()->addSphereSceneNode(0.1f);
     ballT = app->GetSceneManager()->addSphereSceneNode(0.1f);
     ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
     ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);
         
     // Finalize construction of visualization assets
     app->AssetBindAll();
     app->AssetUpdateAll();
}


//Calling this function once will update the visualization of Irricht by one step 
void Vehicle_model::irricht_advance(double step_size, ChDriver::Inputs driver_inputs){
    if(!inp->Get_use_irricht())
        return;

    //irricht advance
    // Update sentinel and target location markers for the path-follower controller.
    const ChVector<>& pS = driver_follower->GetSteeringController().GetSentinelLocation();
    const ChVector<>& pT = driver_follower->GetSteeringController().GetTargetLocation();
    ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
    ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));
    app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
    app->DrawAll();
    app->EndScene();
    std::string msg = "Follower driver";
    app->Synchronize(msg, driver_inputs);
    app->Advance(step_size);
}


//use Povray rendering(PovRay)
void Vehicle_model::initialize_pov(){
    if(!inp->Get_status_povray())
        return;

    pov_dir =  GetChronoOutputPath() + "/POVRAY";
    
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return;
    }
    GetLog() << pov_dir << "\n";
    terrain->ExportMeshPovray(pov_dir);
    driver_follower->ExportPathPovray(GetChronoOutputPath());

}

void Vehicle_model::output_pov(int render_frame){
    if(!inp->Get_status_povray())
        return;

    char filename[100];
    sprintf(filename, "data_%03d.dat",render_frame+1);
    utils::WriteShapesPovray(veh->GetSystem(), pov_dir + "/" + filename);
    GetLog() << pov_dir + "/" + filename << "\n";
    //exit(1);
}



//Display time and other current data as texts
void Vehicle_model::disp_current_status(){

    GetLog() << "time = " << current_time << "[s]" << "  vel = " << veh->GetVehicleSpeedCOM() << "\n";

}


void  Vehicle_model::conv_axis(double array[6]){
    if(inp->Get_direc_Xaxis() == false)
        array[0] *=-1.0;
    if(inp->Get_rot_Xaxis() == false)
        array[1] *= -1.0;
    if(inp->Get_direc_Yaxis() == false)
        array[2] *= -1.0;
    if(inp->Get_rot_Yaxis() == false)
        array[3] *= -1.0;
    if(inp->Get_direc_Zaxis() == false)
        array[4] *= -1.0;
    if(inp->Get_rot_Zaxis() == false)
        array[5] *= -1.0;
}

//public
//coupling
void Vehicle_model::vehicle_initialize(){
    culc_mode = coupling;
    culc_sec = preparation;
    setup_system();
    GetLog() << "system setup completed\n";

    initialize();
    GetLog() << "Initialization of vehicle system completed\n";
    out.reset(new Output(*inp, *veh));
    restart.reset(new Restart() );
    current_time = 0.0;
}

void Vehicle_model::vehicle_advance(Cfd2Vehicle *cfd2veh_data, Vehicle2Cfd *veh2cfd_data ){
    Exchange_data exc_data(*inp);

    exc_data.data_unpacking(cfd2veh_data);
    double avd_step_size = this->step_size;
    advance(avd_step_size, cfd2veh_data);
    disp_current_status();
    out->write(current_time, *veh, *driver_follower, *terrain);  
    exc_data.data_packing(*veh, veh2cfd_data);

}

//stand-alone
void Vehicle_model::vehicle_initialize_stand_alone(){
    culc_mode = stand_alone;
    culc_sec = preparation;
    setup_system();
    GetLog() << "system setup completed\n";

    initialize();       //initialize vehicle system
    fmap.reset(new FForce_map(*inp) ); //initialize flow force sytem from aero-coef map

    GetLog() << "Initialization of vehicle system and aerodynamic-coef map completed\n";
    out.reset(new Output(*inp, *veh));
    restart.reset(new Restart() );
    current_time = 0.0;
}

void Vehicle_model::vehicle_advance_stand_alone(){
    Cfd2Vehicle fmap2veh_data;
    fmap->Get_fforce_from_map(*veh, current_time, &fmap2veh_data);
    double avd_step_size = this->step_size;
    advance(avd_step_size, &fmap2veh_data);
    disp_current_status();
    out->write(current_time, *veh, *driver_follower, *terrain);  

}

