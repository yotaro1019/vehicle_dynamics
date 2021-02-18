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
    exit(1);
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

        if(name == "begin_point"){
            Point_data pt_tmp;
            pt_tmp.id = Set_int_value(ss);

            //initialize
            pt_tmp.name = std::to_string(point_lists.size()+1);
            pt_tmp.pos.Set(123456789,123456789,123456789);
            pt_tmp.rot.SetNull();
            pt_tmp.qrot.Set(1, 0, 0, 0);

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
                    pt_tmp.name = Set_str_value(ss);
                }
                
                if(name == "pos" ){
                    pt_tmp.pos = Set_ChVector(ss);
                }

                if(name == "rot" ){
                    pt_tmp.rot = Set_ChVector(ss);
                    pt_tmp.qrot.Q_from_Euler123(pt_tmp.rot); 
                }

                if(name == "end_point" ){
                    int check_id = Set_int_value(ss);
                    //if(pt_tmp.pos.IsNull() == false){
                    //    GetLog() << "point id : " << pt_tmp.id << "\n";
                    //    GetLog() << "pos is not defined\n";
                    //    exit(1);
                    //}

                    if(pt_tmp.id == check_id){
                        point_lists.push_back(pt_tmp);
                        break;
                    }
                }

            }   
            
        }


    }
    GetLog() << "log point(velocity & accereration) : " << point_lists.size() << "\n";


    
}


void Point_vel_acc::initialize_point(WheeledVehicle &veh){
    GetLog() << veh.GetName() << "\n";
    for(int i = 0; i <point_lists.size(); i++){
        GetLog() << "i = " << i << "\n";
        ChCoordsys<> cord(point_lists[i].pos, point_lists[i].qrot);
        veh.GetChassis()->AddMarker(point_lists[i].name, cord);
    } 


}


void Point_vel_acc::log_points_vel_acc(WheeledVehicle &veh){

    //for(int i = 0; i<log_pos_list.size(); i++){
        GetLog() << "1" << "\n";
        //veh.GetChassis()->AddMarker("point", ChCoordsys(log_pos_list[i]) );
    //}


}