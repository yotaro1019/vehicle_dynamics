#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<algorithm>
#include"inp_init_data.h"

namespace chrono{
namespace vehicle{
Input_data::Input_data(const std::string input_fname){

        this->default_param();  //デフォルトのパラメータをよみこみ
        this->read_param(input_fname); //パラメーターファイルからの読み込み
        this->refine_tire_fname();
        this->check_param(); //パラメータミスの確認

    }


//use constructor
void Input_data::default_param(){
    //directory
    inp_dir_name = "inp_vehicle_models";
    out_dir_name = "output_chrono";

    //vehicle system (must check)
    vehicle_JSON_fname = "NaN";
    powertrain_JSON_fname = "NaN";
    terrain_JSON_fname = "NaN";
    vehicle_init_loc.Set(0.0, 0.0, 0.0);
    vehicle_init_rot.Set(1.0, 0.0, 0.0, 0.0);    

    //trailer system 
    use_trailer_model = false;
    trailer_JSON_fname = "NaN";
    trailer_offset.Set(0.0, 0.0, 0.0);
    trailer_joint_pos.Set(0.0, 0.0, 0.0);

    //driver path (must check)
    path_txt_fname = "NaN";
    driver_lookah = 5.0;

    //target speed (must check)
    target_speed = -123456789.0;

    //visualization 
    chassis_viz_type = VisualizationType::PRIMITIVES;
    wheel_viz_type = VisualizationType::PRIMITIVES;
    tire_viz_type = VisualizationType::PRIMITIVES;
    parts_vis_type = VisualizationType::PRIMITIVES;
    //povray
    povray_output = false;
    out_pov_itvl = 0;
    export_pov_mesh = false;
    //irricht
    use_irricht = false;


    //coupling simulation param
    stabi_dt = 0.002;
    coupling_dt = 0.002;
    tire_step_size = 0.002;
    stabi_step = 0;

    
    //restart system (if restart bool = true, must check fname)
    restart_step = -1000000;
    restart_initialization = false;
    restart_output_itvl = 100;


    //output
    chassis_com_bool = false;
    chassis_COM_fname = "chssiss_COM.out";
    driver_inp_bool = false;
    driver_inp_fname = "driver_input.out";
    powertrain_status_bool = false;
    powertrain_status_fname = "powertrain.out";
    tire_force_bool = false;
    tire_fl_force_fname = "tire_FL_force.out";
    tire_fr_force_fname = "tire_FR_force.out";
    tire_rl_force_fname = "tire_RL_force.out";
    tire_rr_force_fname = "tire_RR_force.out";
    wheel_st_angle_fname = "wheel_status.out";
    fforce_output_bool = false;
    fforce_chassis_fname = "chassis_fforce.out";
    init_loc_pos_bool = true;
    coupling_info_bool = false;

    //Get Point vel and acc in cabin
    cabin_pdata_fname = "NaN";
    cabin_pdata_bool = false;

    //parameters for vehicle motion analysis alone
    end_step = 0;
    //irrlicht
    cam_trackPoint = 0.0, 0.0, 1.75;
    chase_distance = 6.0;
    chase_height = 0.5;

    //fforce_map
    fforce_map_fname = "NaN";
    fforce_map_bool = false;
    rho = -10000;
    vpja = -10000;
    wb = -10000;
    airspeed_inp_fname = "NaN";
    airspeed_bool = false;



    //coupling 
    ///data exchange
    direc_Xaxis = false;
    rot_Xaxis = false;
    direc_Yaxis = false;
    rot_Yaxis = false;
    direc_Zaxis = false;
    rot_Zaxis = false;

    flow_stabi_time = 0.0;

}

void Input_data::check_param(){
    //vehicle system (must check)
    if(vehicle_JSON_fname == "NaN"){ 
        GetLog() << "Not input vehicle JSON file name\n";
    }
    if(powertrain_JSON_fname == "NaN"){
        GetLog() << "Not input powertran JSON file name\n";
    }
    if(terrain_JSON_fname == "NaN" ){
        GetLog() << "Not input terrain JSON fname \n";
    }
    
    //tire_system (must check)
    if(tire_dir_name == "NaN"){
        GetLog() << "Not input tire dir name\n";
    }

    if(this->Get_ntire_JSON() == 0){ 
        GetLog() << "Not input tire JSON file name\n"; 
    }
    //driver path (must check)
    if(path_txt_fname == "NaN"){
        GetLog() << "Not input driver follow path file name\n";
    }

    //target speed (must check)
    if(target_speed == -123456789.0){
        GetLog() << "Not input target speed\n";
    }

    if(fforce_map_bool){
       if(fforce_map_fname == "NaN"){
           GetLog() << "Not input fmap file\n";
       }
       if(rho == -10000){
           GetLog() << "Not input rho\n";
       }
       if(vpja == -10000){
           GetLog() << "Not input vehicle projected area\n";
       }
       if(wb == -10000){
           GetLog() << "Not input Wheel base\n";
       }
    }
    
    if(airspeed_bool){
        if(airspeed_inp_fname == "NaN"){
            GetLog() << "Not input airspeed file\n";
        }
    }

    
    
}

void Input_data::read_param(std::string input_fname){
        std::ifstream inp_param_file(input_fname);
        if(inp_param_file.fail()){
            std::cout << "ERROR unable to open " << input_fname << "\n";
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

            
            if(name == "inp_dir_name"){
              this->inp_dir_name = Set_str_value(ss);
            }

            if(name == "out_dir_name"){
              this->out_dir_name = Set_str_value(ss);
            }

        //vehicle model
            if(name == "vehicle_JSON"){
               this->vehicle_JSON_fname = Set_str_value(ss);
            }

            if(name == "terrain_JSON"){
                this->terrain_JSON_fname = Set_str_value(ss);
            }

            if(name == "powertrain_JSON"){
                this->powertrain_JSON_fname = Set_str_value(ss);
            }

            if(name == "tire_dir_name"){
                this->tire_dir_name = Set_str_value(ss);
            }
            if(name == "tire_JSON_fnames"){
                this->tire_JSON_fnames = Set_str_vec(ss);
            }


        //trailer model
            if(name == "trailer_JSON_fname"){
               this->trailer_JSON_fname = Set_str_value(ss);
               this->use_trailer_model = true;
            }
            if(name == "trailer_offset"){
               this->trailer_offset = Set_ChVector(ss);
            }
            if(name == "trailer_joint_pos"){
               this->trailer_joint_pos = Set_ChVector(ss);
            }            

        //driver model
            if(name == "path_txt"){
                this->path_txt_fname =  Set_str_value(ss);
            }
            if(name == "driver_LookAhead_Distance"){
                this->driver_lookah = Set_double_value(ss);
            }

            if(name == "vehicle_init_loc"){
                this->vehicle_init_loc = Set_ChVector(ss);
            }

            if(name == "vehicle_init_rot"){
                this->vehicle_init_rot = Set_ChQuaternion(ss);
            }

            if(name == "target_speed"){
                this->target_speed = Set_double_value(ss);
            }

            if(name == "stabilize_step_size"){
                this->stabi_dt = Set_double_value(ss);
            }
            if(name == "coupling_step_size"){
                this->coupling_dt = Set_double_value(ss);
            }

            if(name == "tire_step_size"){
                this->tire_step_size = Set_double_value(ss);
            }


            if(name == "stabilization_step"){
                this->stabi_step = Set_double_value(ss);
            }

            if(name == "restart_step"){
                this->restart_step = Set_int_value(ss);
            }


            if(name == "restart_output_itvl"){
                this->restart_output_itvl = Set_int_value(ss);
                
            }

            if(name == "restart_initialization"){
                this->restart_initialization = Set_bool_value(ss);
                
            }           


            if(name == "parts_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->parts_vis_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->parts_vis_type= VisualizationType::PRIMITIVES;
                }else{
                    this->parts_vis_type= VisualizationType::NONE;
                }               
            }

            if(name == "chassis_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->chassis_viz_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->chassis_viz_type= VisualizationType::PRIMITIVES;
                }else{
                    this->chassis_viz_type= VisualizationType::NONE;
                }               
            }

            if(name == "wheel_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->wheel_viz_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->wheel_viz_type= VisualizationType::PRIMITIVES;
                }else{
                    this->wheel_viz_type= VisualizationType::NONE;
                }               
            }

            if(name == "tire_viz_type"){
                std::string vis = Set_str_value(ss);

                if(vis == "MESH"){
                    this->tire_viz_type= VisualizationType::MESH;
                }else if(vis == "PRIMITIVES"){
                    this->tire_viz_type= VisualizationType::PRIMITIVES;
                }else{
                    this->tire_viz_type= VisualizationType::NONE;
                }               
            }
            
            //povray
            if(name == "output_POV-Ray"){
                this->povray_output = Set_bool_value(ss);
            }
            if(name == "export_pov_mesh"){
                this->export_pov_mesh = Set_bool_value(ss);
            }
            if(name == "POV-Ray_output_itvl"){
                this->out_pov_itvl = Set_int_value(ss);
            }

            //irricht
            if(name == "use_irrlicht"){
                this->use_irricht = Set_bool_value(ss);
            }
            if(name == "cam_trackPoint"){
                this->cam_trackPoint = Set_ChVector(ss);
            }
            if(name == "chase_distance"){
                this->chase_distance = Set_double_value(ss);
            }
            if(name == "chase_height"){
                this->chase_height = Set_double_value(ss);
            }
            

//output data

            if(name == "chassis_COM_data"){
                this->chassis_com_bool = Set_bool_value(ss);
            }

            if(name == "chassis_COM_fname"){
                this->chassis_COM_fname = Set_str_value(ss);
            }
            
            if(name == "driver_input_data"){
                this->driver_inp_bool = Set_bool_value(ss);
            }

            if(name == "driver_input_fname"){
                this->driver_inp_fname = Set_str_value(ss);
            }

            if(name == "powertrain_status_data"){
                this->powertrain_status_bool = Set_bool_value(ss);
            }

            if(name == "powertrain_status_fname"){
                this->powertrain_status_fname = Set_str_value(ss);
            }

            if(name == "tire_force_data"){
                this->tire_force_bool = Set_bool_value(ss);
            }

            if(name == "tire_fl_force_fname"){
                this->tire_fl_force_fname = Set_str_value(ss);
            }
            if(name == "tire_fr_force_fname"){
                this->tire_fr_force_fname = Set_str_value(ss);
            }
            if(name == "tire_rl_force_fname"){
                this->tire_rl_force_fname = Set_str_value(ss);
            }
            if(name == "tire_rr_force_fname"){
                this->tire_rr_force_fname = Set_str_value(ss);
            }
            if(name == "wheel_steering_angle_fname"){
                this->wheel_st_angle_fname = Set_str_value(ss);
            }

            if(name == "fforce_output"){
                this->fforce_output_bool = Set_bool_value(ss);
            }
            if(name == "fforce_chassis_fname"){
                this->fforce_chassis_fname = Set_str_value(ss);
            }
            if(name == "init_loc_pos_data"){
                this->init_loc_pos_bool = Set_bool_value(ss);;
            }

            if(name == "coupling_info"){
                this->coupling_info_bool = Set_bool_value(ss);;
            }



            //Get Point vel and acc in cabin
            if(name == "cabin_pdata_fname"){
                this->cabin_pdata_fname = Set_str_value(ss);
                this->cabin_pdata_bool = true;
            }          

            //parameters for vehicle motion analysis alone
            if(name == "end_step"){
                this->end_step = Set_int_value(ss);
            }


            //fforce map
            if(name == "ffoce_map_inp"){
                this->fforce_map_fname = Set_str_value(ss);
                this->fforce_map_bool = true;
            }
            if(name == "rho"){
                this->rho = Set_double_value(ss);
            }
            if(name == "vehicle_projected_area"){
                this->vpja = Set_double_value(ss);
            }
            if(name == "wheel_base"){
                this->wb = Set_double_value(ss);
            }
            if(name == "airspeed_inp"){
                this->airspeed_inp_fname = Set_str_value(ss);
                this->airspeed_bool = true;
            }
            //CUBE coupling----------------------------------------
            //data exchange
            if(name == "direction_x_axis"){
                this->direc_Xaxis = Set_bool_value(ss);
            }
            if(name == "rot_x_axis"){
                this->rot_Xaxis = Set_bool_value(ss);
            }
            if(name == "direction_y_axis"){
                this->direc_Yaxis = Set_bool_value(ss);
            }
            if(name == "rot_y_axis"){
                this->rot_Yaxis = Set_bool_value(ss);
            }
            if(name == "direction_z_axis"){
                this->direc_Zaxis = Set_bool_value(ss);
            }
            if(name == "rot_z_axis"){
                this->rot_Zaxis = Set_bool_value(ss);
            }

            if(name == "flow_stabilize_time"){
                this->flow_stabi_time = Set_double_value(ss);
            }

        }

}

void Input_data::refine_tire_fname(){
    GetLog() << tire_JSON_fnames.size() << "\n";
    for(int i=0; i<tire_JSON_fnames.size(); i++){
        std::string tmp;
        tmp = this->tire_dir_name + "/" + tire_JSON_fnames[i];
        tire_JSON_fnames[i] = tmp;
    }
    for(int i=0; i<tire_JSON_fnames.size(); i++){
        GetLog() << tire_JSON_fnames[i] << "\n";
    }    
}

//output data

bool Input_data::Get_tire_force_bool(){
    return this->tire_force_bool;
}

std::string Input_data::Get_tire_fl_force_fname(){
    return this->tire_fl_force_fname;
}
std::string Input_data::Get_tire_fr_force_fname(){
    return this->tire_fr_force_fname;
}
std::string Input_data::Get_tire_rl_force_fname(){
    return this->tire_rl_force_fname;
}
std::string Input_data::Get_tire_rr_force_fname(){
    return this->tire_rr_force_fname;
}
std::string Input_data::Get_wheel_st_angle_fname(){
    return this->wheel_st_angle_fname;
}

bool Input_data::Get_fforce_output_bool(){
    return this->fforce_output_bool;
}
std::string Input_data::Get_fforce_chassis_fname(){
    return this->fforce_chassis_fname;
}


//data exchange
//data exchange
 bool Input_data::Get_direc_Xaxis(){
     return this->direc_Xaxis;
 }
 bool Input_data::Get_rot_Xaxis(){
     return this->rot_Xaxis;
 }
 bool Input_data::Get_direc_Yaxis(){
     return this->direc_Yaxis;
 }
 bool Input_data::Get_rot_Yaxis(){
     return this->rot_Yaxis;
 }
 bool Input_data::Get_direc_Zaxis(){
     return this->direc_Zaxis;
 }
 bool Input_data::Get_rot_Zaxis(){
     return this->rot_Zaxis;
 }

}   //end namespace vehicle
}   //end namespace chrono