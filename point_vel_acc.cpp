#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<algorithm>
#include "point_vel_acc.h"

Point_vel_acc::Point_vel_acc(bool c_switch, std::string fname, WheeledVehicle &veh){
    this->c_switch = c_switch;
    
    if(!c_switch)
        return;

    this->read_param(fname);
    this->initialize_point(veh);

}

void Point_vel_acc::read_param(std::string fname){
    if(!this->c_switch)
        return;
    
    GetLog() << "read point vel and acc  in cabin\n";
    GetLog() << "input file : " << fname << "\n";

    std::ifstream inp_param_file(fname);
    if(inp_param_file.fail()){
        std::cout << "ERROR unable to open " << fname << "\n";
    }  

    std::string str;
    int counter = 0;
    while(getline(inp_param_file,str)){
        if(counter >= def_point_nlist){
            GetLog() << "The upper limit of recording point is  " << def_point_nlist << "\n";
            GetLog() << "The upper limit has been exceeded\n";
        }

        if(str[0] == '#')
            continue;

        if(str[0] == '!')
            continue;
        
        if(str.empty())
            continue;

        std::stringstream ss;
        std::string name;           

        std::replace(str.begin(), str.end(), '=', ' ');
        std::replace(str.begin(), str.end(), ',', ' ');

        ss << str;
        ss >> name;

        if(name == "begin_point"){
            point_lists[counter].id = Set_int_value(ss);

            //initialize
            point_lists[counter].name = std::to_string(counter+1);
            point_lists[counter].pos.Set(123456789,123456789,123456789);
            point_lists[counter].rot.SetNull();
            point_lists[counter].qrot.Set(1, 0, 0, 0);

            while(getline(inp_param_file,str)){
                if(str[0] == '#')
                    continue;

                if(str[0] == '!')
                    continue;
        
                if(str.empty())
                    continue;

                std::stringstream ss;
                std::string name;           

                std::replace(str.begin(), str.end(), '=', ' ');
                std::replace(str.begin(), str.end(), ',', ' ');

                ss << str;
                ss >> name;

                if(name == "pos_name" ){
                    point_lists[counter].name = Set_str_value(ss);
                }
                
                if(name == "pos" ){
                    point_lists[counter].pos = Set_ChVector(ss);
                }

                if(name == "rot" ){
                    point_lists[counter].rot = Set_ChVector(ss);
                    point_lists[counter].qrot.Q_from_Euler123( point_lists[counter].rot); 
                }

                if(name == "end_point" ){
                    int check_id = Set_int_value(ss);
                    //if(pt_tmp.pos.IsNull() == false){
                    //    GetLog() << "point id : " << pt_tmp.id << "\n";
                    //    GetLog() << "pos is not defined\n";
                    //    exit(1);
                    //}

                    if(point_lists[counter].id == check_id){
                        point_lists[counter].activate = true;
                        counter++;
                        break;
                    }
                }

            }   
            
        }


    }
    GetLog() << "log point(velocity & accereration) : " << counter << "/" << def_point_nlist << "\n";


    
}


void Point_vel_acc::initialize_point(WheeledVehicle &veh){

    for(int i = 0; i < def_point_nlist; i++){
        if(point_lists[i].activate == false){
            break;
        }
        ChCoordsys<> cord(point_lists[i].pos, point_lists[i].qrot);
        veh.GetChassis()->AddMarker(point_lists[i].name, cord);
 	    point_lists[i].marker_id = veh.GetChassis()->GetMarkers().size()-1; 

        //output file
        char fname[500];
        sprintf(fname, "record_point_%02d_%s.txt",  point_lists[i].id, point_lists[i].name.c_str());
        point_lists[i].pout_file.initialize(point_lists[i].activate, GetChronoOutputPath() + fname);
    } 
   
}


void Point_vel_acc::record_points_vel_acc(int step, double time, WheeledVehicle &veh){

    for(int i = 0; i < def_point_nlist; i++){
        if(point_lists[i].activate == false){
            break;
        }
        ChMarker marker = *veh.GetChassis()->GetMarkers()[point_lists[i].marker_id].get();
 	    point_lists[i].pout_file.write(step, time, marker.GetPos(), marker.GetPos_dt(), marker.GetPos_dtdt(), marker.GetWvel_loc(), marker.GetWacc_loc());
    } 

}

void Point_vel_acc::restart(int restart_step){
    for(int i = 0; i < def_point_nlist; i++){
        if(point_lists[i].activate == false){
            break;
        }
 	    point_lists[i].pout_file.restart(restart_step);
    }     
}