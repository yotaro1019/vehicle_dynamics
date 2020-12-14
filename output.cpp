#include<iostream>
#include<string>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include"output.h"
#include"outputlist.h"

Chassis_vel_fout chassis_log;
Driver_fout dvr_log;
Powertrain_fout ptr_log;
std::vector<Tire_fout> tire_log;
std::vector<int> ntire_list; //number of tire (record each axis)
int ntire_total;    //number of total tires

//1way-info
Vehicle2CFD_info mesh_vel_info;

Output::Output(Input_data &inp, WheeledVehicle &veh){
    this->initialize_veh_status(inp, veh);
    this->initialize_1way_info(inp, veh);
}

void Output::write(double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr, RigidTerrain &ter, Vehicle2Cfd &v2c){
    this->write_veh_status(time, veh, dvr, ter);
    this->write_1way_info(time, v2c);
}



void Output::initialize_veh_status(Input_data &inp, WheeledVehicle &veh){
    chassis_log.initialize(inp.Get_chassis_com_bool(), GetChronoOutputPath() + inp.Get_chassis_COM_fname());
    dvr_log.initialize(inp.Get_driver_input_bool(), GetChronoOutputPath() + inp.Get_driver_input_fname());
    ptr_log.initialize(inp.Get_powertrain_status_bool(), GetChronoOutputPath() + inp.Get_powertrain_status_fname());

    ntire_total = 0;
    
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
    int ntire =  axle->GetWheels().size();
    GetLog() << ntire << "\t";
    ntire_list.push_back(ntire);
    ntire_total += ntire;
    }

    tire_log.resize(ntire_total);
    GetLog() << ntire_list.size() << "\n";
    int ntire = 0;
    int naxle = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
       std::string fname_base = "tire_axle-" + std::to_string(naxle);
       std::string fname;

       Tire_fout tire_log_base;

       if(ntire_list[naxle] == 2){
           //LEFT

           fname = fname_base + "_LEFT.txt";
           tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
           GetLog() << ntire << "\n";
           ntire++;
 
           //RIGHT
           fname = fname_base + "_RIGHT.txt";
           GetLog() << fname << "\n";
           tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
           GetLog() << ntire << "\n";
           ntire++;
        
       }else if(ntire_list[naxle] == 4){
           //LEFT inside          
            fname = fname_base + "_LEFT_inside.txt";
            tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
            GetLog() << ntire << "\n";
            ntire++;
           //LEFT outside
            fname = fname_base + "_LEFT_outside.txt";
            tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
            GetLog() << ntire << "\n";
            ntire++;
            //RIGHT inside          
            fname = fname_base + "_RIGHT_inside.txt";
            tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
            GetLog() << ntire << "\n";
            ntire++;

           //RIGHT outside
            fname = fname_base + "_RIGHT_outside.txt";
            tire_log[ntire].initialize(inp.Get_tire_force_bool(), GetChronoOutputPath() + fname);
            GetLog() << ntire << "\n";
            ntire++;
        }        
        naxle++;    
    }
    GetLog() << "ntire = " << ntire << "\tnaxle = " << naxle << "\n";
    GetLog() << tire_log.size() << "\n";
}



void Output::write_veh_status(double time, WheeledVehicle &veh, ChPathFollowerDriver &dvr, RigidTerrain &ter){
    chassis_log.write(time, veh);
    dvr_log.write(time, dvr);
    ptr_log.write(time, *veh.GetPowertrain() );
    
    int tire_id = 0;
    int naxle = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
        

        if(ntire_list[naxle] == 2){
            //LEFT
            std::shared_ptr<ChWheel> wheel_L = axle->GetWheel(LEFT, SINGLE);    //ChWheel
            tire_log[tire_id].write(time, *wheel_L , ter);
            tire_id++;

            //RIGHT
            std::shared_ptr<ChWheel> wheel_R = axle->GetWheel(RIGHT, SINGLE);    //ChWheel
            tire_log[tire_id].write(time, *wheel_R , ter);   //ChWheel
            tire_id++;

        }else if(ntire_list[naxle] == 4){
            //LEFT inside
            std::shared_ptr<ChWheel> wheel_LIN = axle->GetWheel(LEFT, INNER);    //ChWheel
            tire_log[tire_id].write(time, *wheel_LIN , ter);           
            tire_id++;

            //LEFT outside
            std::shared_ptr<ChWheel> wheel_LOUT = axle->GetWheel(LEFT, OUTER);    //ChWheel
            tire_log[tire_id].write(time, *wheel_LOUT , ter);           
            tire_id++;

//
//
            //RIGHT inside
            std::shared_ptr<ChWheel> wheel_RIN = axle->GetWheel(RIGHT, INNER);    //ChWheel
            tire_log[tire_id].write(time, *wheel_RIN , ter);   //ChWheel
            tire_id++;

//
            //RIGHT outside
            std::shared_ptr<ChWheel> wheel_ROUT = axle->GetWheel(RIGHT, OUTER);    //ChWheel
            tire_log[tire_id].write(time, *wheel_ROUT , ter);   //ChWheel
            tire_id++;

        }
        naxle++;
    }
}


//1WAY info
void Output::initialize_1way_info(Input_data &inp, WheeledVehicle &veh){
    this->info_1way_bool = inp.Get_coupling_info_bool();
    if(!this->info_1way_bool)
        return;

    std::string out_dir =  GetChronoOutputPath() + "/1way";   
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    mesh_vel_info.initialize(inp.Get_coupling_info_bool(), out_dir+"/mesh_vel.txt");

}

void Output::write_1way_info(double time, Vehicle2Cfd &v2c){
    if(!this->info_1way_bool)
        return;

    GetLog() << v2c.mesh_vel.translation << "\n";
    mesh_vel_info.write(time, v2c.mesh_vel.translation, v2c.mesh_acc.translation);

}