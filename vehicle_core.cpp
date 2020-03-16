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

std::shared_ptr<Input_data> inp;
std::shared_ptr<WheeledVehicle> veh;
std::shared_ptr<RigidTerrain> terrain;


int main(int argc, char* argv[]) {
    inp.reset(new Input_data("vehicle_params.inp") );
    filesystem::path current_dir(filesystem::path().getcwd());
    const std::string current_dir_path = current_dir.str();
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);
    chrono::vehicle::SetDataPath(current_dir_path + "/" + inp->Get_inp_dir_name() + "/");

    //==========================================
    //setup params
    double step_size = inp->Get_coupling_dt();
    double tire_step_size = inp->Get_tire_step_size();
    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------
    // Create the vehicle system
    veh.reset(new WheeledVehicle (vehicle::GetDataFile(inp->Get_vehicle_JSON_fname()), ChMaterialSurface::NSC));
    //WheeledVehicle vehicle(vehicle::GetDataFile(inp->Get_vehicle_JSON_fname()), ChMaterialSurface::NSC);
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
            std::shared_ptr<ChTire> tireL = ReadTireJSON( vehicle::GetDataFile(tire_fname) );
            std::shared_ptr<ChTire> tireR = ReadTireJSON( vehicle::GetDataFile(tire_fname) );
            veh->InitializeTire(tireL, axle->m_wheels[0], inp->Get_tire_viz_type());
            veh->InitializeTire(tireR, axle->m_wheels[1], inp->Get_tire_viz_type());
            naxle++;
        }
    }

    // ----------------------
    // Create the Bezier path
    // ----------------------

    // From data file
    std::shared_ptr<ChBezierCurve> path = ChBezierCurve::read(vehicle::GetDataFile(inp->Get_path_txt_fname()));


    ChWheeledVehicleIrrApp app(veh.get(), L"Steering PID Controller Demo", irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.EnableGrid(false);
    app.SetChaseCamera(inp->Get_cam_trackPoint(), 6.0, 0.5);

    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // -------------------------
    // Create the driver systems
    // -------------------------
    ChPathFollowerDriver driver_follower(*veh, path, "my_path", inp->Get_target_speed());
    driver_follower.GetSteeringController().SetLookAheadDistance(5);
    driver_follower.GetSteeringController().SetGains(0.8, 0, 0);
    driver_follower.GetSpeedController().SetGains(0.4, 0, 0);
    driver_follower.Initialize();

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();


    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = veh->GetChassis()->GetLocalDriverCoordsys().pos;


    // Initialize simulation frame counter and simulation time
    int sim_frame = 0;

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        // Extract system state
        double time = veh->GetSystem()->GetChTime();
        ChVector<> acc_CG = veh->GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = veh->GetVehicleAcceleration(driver_pos);

        // End simulation
        if (time >= inp->Get_calc_t_end())
            break;

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver_follower.GetInputs();


        // Update sentinel and target location markers for the path-follower controller.
        const ChVector<>& pS = driver_follower.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver_follower.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();


        // Update modules (process inputs from other modules)
        driver_follower.Synchronize(time);
        terrain->Synchronize(time);
        veh->Synchronize(time, driver_inputs, *terrain);
        std::string msg = "Follower driver";
        app.Synchronize(msg, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver_follower.Advance(step_size);
        terrain->Advance(step_size);
        veh->Advance(step_size);
        app.Advance(step_size);

        // Increment simulation frame number
        sim_frame++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
