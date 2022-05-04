#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"
#include"outputlist.h"
#define def_ntire 30

//vehicle status
Chassis_vel_fout chassis_log;
Driveline_fout driveline_log;
Driver_fout dvr_log;
Powertrain_fout ptr_log;
Tire_fout tire_log[def_ntire];
std::vector<int> ntire_list; //number of tire (record each axis)
int ntire_total;    //number of total tires

//1way-info
Vehicle2CFD_info mesh_vel_info;
Vehicle2CFD_info chassis_vel_info;
Vehicle2CFD_info str_vel_info[def_ntire];
Vehicle2CFD_info wheel_vel_info[def_ntire];

//ffoece_info
FForce_info cfd_fforce_info;

Output::Output(Input_data &inp, WheeledVehicle &veh){
    this->initialize_veh_status(inp, veh);
    this->initialize_1way_info(inp, veh);
    this->initialize_fforce_info(inp);
}

void Output::write(int step, double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr, RigidTerrain &ter, Cfd2Vehicle &c2v, Vehicle2Cfd &v2c){
    this->write_veh_status(step, time, veh, dvr, ter);
    this->write_1way_info(step, time, veh, v2c);
    this->write_fforce(step, time, c2v);
}



void Output::initialize_veh_status(Input_data &inp, WheeledVehicle &veh){

    chassis_log.initialize(inp.Get_chassis_com_bool(), GetChronoOutputPath() + inp.Get_chassis_COM_fname());
    driveline_log.initialize(inp.Get_driveline_status_bool(), GetChronoOutputPath() + inp.Get_driveline_status_fname(), veh);
    dvr_log.initialize(inp.Get_driver_input_bool(), GetChronoOutputPath() + inp.Get_driver_input_fname());
    ptr_log.initialize(inp.Get_powertrain_status_bool(), GetChronoOutputPath() + inp.Get_powertrain_status_fname());

    ntire_total = 0;
    
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
    int ntire =  axle->GetWheels().size();
    GetLog() << ntire << "\t";
    ntire_list.push_back(ntire);
    ntire_total += ntire;
    }

    
    GetLog() << ntire_list.size() << "\n";
    int ntire = 0;
    int naxle = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
       std::string fname_base = "tire_axle-" + std::to_string(naxle);
       std::string fname;


          
       if(ntire_list[naxle] == 2){
           //LEFT
            {
               fname = fname_base + "_LEFT.out"; 
               tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
               ntire++;  
            }
           
 
        //RIGHT
            {
               fname = fname_base + "_RIGHT.out";
               tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
               ntire++; 
            }

       }else if(ntire_list[naxle] == 4){
           //LEFT inside  
            {
                fname = fname_base + "_LEFT_inside.out";
                tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);    
                ntire++;
            }

               //LEFT outside
            {
               fname = fname_base + "_LEFT_outside.out";
               tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
               ntire++;
            }
                //RIGHT inside
            {
               fname = fname_base + "_RIGHT_inside.out";
               tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
               ntire++;
            }

               //RIGHT outside
            {
               fname = fname_base + "_RIGHT_outside.out";
               tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
               ntire++;
            }
        }        
        naxle++;    
    }
    GetLog() << "ntire = " << ntire << "\tnaxle = " << naxle << "\n";
}




void Output::write_veh_status(int step, double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr, RigidTerrain &ter){
    chassis_log.write(step, time, veh);
    driveline_log.write(step, time, veh);
    dvr_log.write(step, time, dvr);
    ptr_log.write(step, time, *veh.GetPowertrain() );

    
    int tire_id = 0;
    int naxle = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
        

        if(ntire_list[naxle] == 2){
            //LEFT
            std::shared_ptr<ChWheel> wheel_L = axle->GetWheel(LEFT, SINGLE);    //ChWheel
            tire_log[tire_id].write(step, time, *wheel_L , ter);
            tire_id++;

            //RIGHT
            std::shared_ptr<ChWheel> wheel_R = axle->GetWheel(RIGHT, SINGLE);    //ChWheel
            tire_log[tire_id].write(step, time, *wheel_R , ter);   //ChWheel
            tire_id++;

        }else if(ntire_list[naxle] == 4){
            //LEFT inside
            std::shared_ptr<ChWheel> wheel_LIN = axle->GetWheel(LEFT, INNER);    //ChWheel
            tire_log[tire_id].write(step, time, *wheel_LIN , ter);           
            tire_id++;

            //LEFT outside
            std::shared_ptr<ChWheel> wheel_LOUT = axle->GetWheel(LEFT, OUTER);    //ChWheel
            tire_log[tire_id].write(step, time, *wheel_LOUT , ter);           
            tire_id++;

//
//
            //RIGHT inside
            std::shared_ptr<ChWheel> wheel_RIN = axle->GetWheel(RIGHT, INNER);    //ChWheel
            tire_log[tire_id].write(step, time, *wheel_RIN , ter);   //ChWheel
            tire_id++;

//
            //RIGHT outside
            std::shared_ptr<ChWheel> wheel_ROUT = axle->GetWheel(RIGHT, OUTER);    //ChWheel
            tire_log[tire_id].write(step, time, *wheel_ROUT , ter);   //ChWheel
            tire_id++;

        }
        naxle++;
    }
}


//1WAY info
void Output::initialize_1way_info(Input_data &inp, WheeledVehicle &veh){
    int init_step = inp.Get_restart_step();
    this->info_1way_bool = inp.Get_coupling_info_bool();
    if(!this->info_1way_bool)
        return;

    std::string out_dir =  GetChronoOutputPath() + "/1way";   
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    mesh_vel_info.initialize(inp.Get_coupling_info_bool(), out_dir+"/mesh_vel.txt");
    chassis_vel_info.initialize(inp.Get_coupling_info_bool(), out_dir+"/chassis_vel.txt");

    int axle_id = 0;
    int wheel_id = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
        if(ntire_list[axle_id] == 2){
            {

                str_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/str_vel_axle-"+std::to_string(axle_id)+"_L.txt");
                wheel_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/wheel_vel_axle-"+std::to_string(axle_id)+"_L.txt");                

                wheel_id++;
            }

            {
                str_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/str_vel_axle-"+std::to_string(axle_id)+"_R.txt");
                wheel_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/wheel_vel_axle-"+std::to_string(axle_id)+"_R.txt");                

                wheel_id++;
            }


        }else if(ntire_list[axle_id] == 4){
            {
                str_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/str_vel_axle-"+std::to_string(axle_id)+"_Lout.txt");
                wheel_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/wheel_vel_axle-"+std::to_string(axle_id)+"_Lout.txt");                

                wheel_id++;
            }

            {
                str_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/str_vel_axle-"+std::to_string(axle_id)+"_Lin.txt");
                wheel_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/wheel_vel_axle-"+std::to_string(axle_id)+"_Lin.txt");                
                wheel_id++;
            }
    
            {
                str_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/str_vel_axle-"+std::to_string(axle_id)+"_Rin.txt");
                wheel_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/wheel_vel_axle-"+std::to_string(axle_id)+"_Rin.txt");                
                wheel_id++;
            }

            {
                str_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/str_vel_axle-"+std::to_string(axle_id)+"_Rout.txt");
                wheel_vel_info[wheel_id].initialize(inp.Get_coupling_info_bool(), out_dir+"/wheel_vel_axle-"+std::to_string(axle_id)+"_Rout.txt");             
                wheel_id++;
            }

        }
        axle_id++;
    }

}

void Output::write_1way_info(int step, double time, WheeledVehicle &veh, Vehicle2Cfd &v2c){
    if(!this->info_1way_bool)
        return;

    
    mesh_vel_info.write(step, time, v2c.mesh_vel.translation, v2c.mesh_acc.translation);
    chassis_vel_info.write(step, time, v2c.chassis_vel.translation, v2c.chassis_vel.rotation);
    
    int axle_id = 0;
    int wheel_id = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
        if(ntire_list[axle_id] == 2){
            for(int j = 0; j<2; j++){
                //steering vel
                str_vel_info[wheel_id].write(step, time, v2c.str_vel[wheel_id].translation, v2c.str_vel[wheel_id].rotation);
                //wheel vel
                wheel_vel_info[wheel_id].write(step, time, v2c.wheel_vel[wheel_id].translation, v2c.wheel_vel[wheel_id].rotation);
                wheel_id++;
            }
        }else if(ntire_list[axle_id] == 4){
            for(int j = 0; j<4; j++){
                //steering vel
                str_vel_info[wheel_id].write(step, time, v2c.str_vel[wheel_id].translation, v2c.str_vel[wheel_id].rotation);
                //wheel vel
                wheel_vel_info[wheel_id].write(step, time, v2c.wheel_vel[wheel_id].translation, v2c.wheel_vel[wheel_id].rotation);
                wheel_id++;
            }
        }
        axle_id++;
    }
}

void Output::initialize_fforce_info(Input_data &inp){
    int init_step = inp.Get_restart_step();
    cfd_fforce_info.initialize(inp.Get_coupling_info_bool(), GetChronoOutputPath() + "/inp_fforce.out");
}

void Output::write_fforce(int step, double time, Cfd2Vehicle &c2v){
    cfd_fforce_info.write(step, time, c2v.fforce.translation, c2v.fforce.rotation);
}


void Output::restart(int restart_step){
    
//vehicle status
    chassis_log.restart(restart_step);
    driveline_log.restart(restart_step);
    dvr_log.restart(restart_step);
    ptr_log.restart(restart_step);

    for(int i = 0; i<def_ntire; i++){
        tire_log[i].restart(restart_step);


    }


//1way-info
    mesh_vel_info.restart(restart_step);
    chassis_vel_info.restart(restart_step);

    for(int i = 0; i<def_ntire; i++){
        str_vel_info[i].restart(restart_step);
        wheel_vel_info[i].restart(restart_step);
        

    }


    //ffoece_info
    cfd_fforce_info.restart(restart_step);
}