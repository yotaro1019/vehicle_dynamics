#include "Veh_Visualization.h"


Veh_Visualization::Veh_Visualization(enum Calculation_mode calc_mode, Input_data &inp, WheeledVehicle &veh, RigidTerrain &terrain, ChPathFollowerDriver &driver_follower){
    if(calc_mode == stand_alone && inp.Get_use_irricht() == true){
        irricht_switch = true;
    }else{
        irricht_switch = false;
    }    
    
    if(inp.Get_status_povray() == true){
        pov_switch = true;
    }else{
        pov_switch = false;
    }

    this->step_dt = inp.Get_coupling_dt();
    this->irricht_initialize(inp, veh);
    pov_out_itvl = inp.Get_itvl_povray();
    this->povray_initialize( terrain, driver_follower);


}


void Veh_Visualization::viz_advance(double step_dt, double time, int step, WheeledVehicle &veh,  ChPathFollowerDriver &driver_follower){

    this->irricht_advance(step_dt, driver_follower);
    this->output_pov(step, veh);
}


void Veh_Visualization::irricht_initialize(Input_data &inp, WheeledVehicle &veh){
    if(!irricht_switch)
        return;

    app.reset(new ChWheeledVehicleIrrApp(&veh, L"Steering PID Controller Demo", irr::core::dimension2d<irr::u32>(800, 640)) );
    
    app->SetHUDLocation(500, 20);
    app->SetSkyBox();
    app->AddTypicalLogo();
    app->AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                        100);
    app->AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                        100);
    //app->EnableGrid(false);
    app->SetChaseCamera(inp.Get_cam_trackPoint(), inp.Get_chase_distance(), inp.Get_chase_height());

    app->SetTimestep(step_dt);

    // Visualization of controller points (sentinel & target)
    ballS = app->GetSceneManager()->addSphereSceneNode(0.1f);
    ballT = app->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);
        
    // Finalize construction of visualization assets
    app->AssetBindAll();
    app->AssetUpdateAll();   

}


void Veh_Visualization::irricht_advance(double step_dt, ChPathFollowerDriver &driver_follower){
    if(!irricht_switch)
        return;

    //irricht advance
    // Update sentinel and target location markers for the path-follower controller.
    const ChVector<>& pS = driver_follower.GetSteeringController().GetSentinelLocation();
    const ChVector<>& pT = driver_follower.GetSteeringController().GetTargetLocation();
    ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
    ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));
    app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
    app->DrawAll();
    app->EndScene();
    std::string msg = "Follower driver";
    app->Synchronize(msg, driver_follower.GetInputs());
    app->Advance(step_dt);

}

void Veh_Visualization::povray_initialize(RigidTerrain &terrain, ChPathFollowerDriver &driver_follower){

    if(!pov_switch)
        return;

    pov_dir =  GetChronoOutputPath() + "/POVRAY";   
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return;
    }

    terrain.ExportMeshPovray(pov_dir);
    driver_follower.ExportPathPovray(pov_dir);

    pov_dir += "/data";   
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return;
    }     
}

void Veh_Visualization::output_pov(int step, WheeledVehicle &veh){
    if(!pov_switch)
        return;
    
    if(step%pov_out_itvl == 0){
        char filename[100];
        sprintf(filename, "data_%03d.dat", step/pov_out_itvl);
        utils::WriteShapesPovray(veh.GetSystem(), pov_dir + "/" + filename);
        GetLog() << pov_dir + "/" + filename << "\n";
    }
}