#ifndef _read_init_data_
#define _read_init_data_
#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/ChSubsysDefs.h"


namespace chrono{
namespace vehicle{

class Input_data{
private:
    //constructor
    void default_param();
    void check_param();
    void read_param(std::string input_fname);
    void refine_tire_fname();



    //dir_path
    std::string inp_dir_name, out_dir_name;
   
    //-------------------------------------------
    //vehicle or tractor
    //vehicle model
    std::string vehicle_JSON_fname;

    //powertrain model
    std::string powertrain_JSON_fname;

    //terrain model
    std::string terrain_JSON_fname;
    
    //tire model
    std::string  tire_dir_name;
    std::vector<std::string> tire_JSON_fnames;
    //driver model
    std::string path_txt_fname;
    double driver_lookah;

    //-------------------------------------------
    //trailer
    bool use_trailer_model;
    std::string trailer_JSON_fname;
    ChVector<> trailer_offset;
    ChVector<> trailer_joint_pos;
    

    //vehicle situation    
    double target_speed, stabi_dt, coupling_dt, tire_step_size, begin_step;
    int stabi_step, restart_step, restart_cube_step;
    bool restart_bool;
    std::string restart_inp_fname;
    ChVector<> vehicle_init_loc;
    ChQuaternion<> vehicle_init_rot;
    

    //visualization
    std::string visualization_type;
    VisualizationType chassis_viz_type, wheel_viz_type, tire_viz_type, parts_vis_type;
    //povray
    bool povray_output;
    bool export_pov_mesh;
    int out_pov_itvl;
    //irricht
    bool use_irricht;


    //output data
    bool chassis_com_bool;    
    std::string chassis_COM_fname;

    bool driver_inp_bool;
    std::string driver_inp_fname;

    bool powertrain_status_bool;
    std::string powertrain_status_fname;

    bool tire_force_bool;
    std::string tire_fl_force_fname, tire_fr_force_fname, tire_rl_force_fname, tire_rr_force_fname, wheel_st_angle_fname;

    bool fforce_output_bool;
    std::string fforce_chassis_fname;

    bool init_loc_pos_bool;

    std::string restart_output_fname;

    bool coupling_info_bool;

    //Get Point vel and acc in cabin
    std::string cabin_pdata_fname;
    bool cabin_pdata_bool;

    //parameters for stand-alone
    double calc_t_begin, calc_t_end;
    int calc_begin_step;

    //irricht
    ChVector<> cam_trackPoint;
    double chase_distance;
    double chase_height;
    //fforce map
    std::string fforce_map_fname;
    bool fforce_map_bool;
    double rho, vpja, wb;   //rho, vehicle_projected_area
    std::string airspeed_inp_fname;
    bool airspeed_bool;

    //data exchange
    bool direc_Xaxis, rot_Xaxis;
    bool direc_Yaxis, rot_Yaxis;
    bool direc_Zaxis, rot_Zaxis;

    //flow stabilize time
    double flow_stabi_time;


public:
    Input_data(const std::string input_fname);

    //dir
    std::string Get_inp_dir_name(){ return inp_dir_name; }
    std::string Get_out_dir_name(){ return out_dir_name; }
    
    //vehicle model
    std::string Get_vehicle_JSON_fname(){ return vehicle_JSON_fname; }
    std::string Get_powertrain_JSON_fname(){ return powertrain_JSON_fname; }
    std::string Get_tire_JSON_fnames(int i) { return tire_JSON_fnames[i]; }
    int Get_ntire_JSON(){ return tire_JSON_fnames.size(); }

    //trailer model
    std::string Get_trailer_JSON_fname(){ return this->trailer_JSON_fname; }
    bool Get_use_trailer_model(){ return this->use_trailer_model; }
    ChVector<> Get_trailer_offset(){ return this->trailer_offset; }
    ChVector<> Get_trailer_joint_pos(){ return this->trailer_joint_pos; }

    //driver model
    std::string Get_path_txt_fname(){ return path_txt_fname; }
    double Get_driver_lookah(){ return this->driver_lookah; }
    //terrain model
    std::string Get_terrain_JSON_fname(){ return terrain_JSON_fname; }  

    //vehicle situation
    ChVector<> Get_vehicle_init_loc(){ return this->vehicle_init_loc; }
    ChQuaternion<> Get_vehicle_init_rot(){ return this->vehicle_init_rot; }
    double Get_target_speed(){ return this->target_speed; }
    double Get_stabi_dt(){ return this->stabi_dt; };
    double Get_coupling_dt(){ return this->coupling_dt; }
    double Get_tire_step_size() { return tire_step_size; }
    int Get_begin_step();
    int Get_stabi_step(){  return this->stabi_step; }
    int Get_restart_step(){ return this->restart_step; }
    int Get_restart_cube_step(){ return this->restart_cube_step; }
    bool Get_restart_bool(){ return this->restart_bool; }
    std::string Get_restart_inp_fname(){  return this->restart_inp_fname; }

    //visualization  
    VisualizationType Get_chassis_viz_type(){ return this->chassis_viz_type; }
    VisualizationType Get_wheel_viz_type(){ return this->wheel_viz_type; }
    VisualizationType Get_tire_viz_type(){ return this->tire_viz_type; }
    VisualizationType Get_parts_viz_type(){ return this->parts_vis_type; }
    //povray
    bool Get_status_povray(){ return this->povray_output; }
    bool Get_export_pov_mesh(){ return this->export_pov_mesh; }
    int Get_itvl_povray(){ return this->out_pov_itvl; }

    //irricht
    bool Get_use_irricht(){ return this->use_irricht; }
    ChVector<> Get_cam_trackPoint(){ return this->cam_trackPoint; }
    double Get_chase_distance(){ return this->chase_distance; }
    double Get_chase_height(){ return this->chase_height; }

    //parameters for vehicle motion analysis alone
    double Get_calc_t_begin(){ return this->calc_t_begin; }
    double Get_calc_t_end(){ return this->calc_t_end; }
    int Get_calc_begin_step(){ return this->calc_begin_step;}
    
    //output data
    bool Get_chassis_ref_bool();
    std::string Get_chassis_ref_fname();

    bool Get_chassis_com_bool(){ return this->chassis_com_bool; }
    std::string Get_chassis_COM_fname(){ return this->chassis_COM_fname; }

    bool Get_driver_input_bool(){ return this->driver_inp_bool; }
    std::string Get_driver_input_fname(){ return this->driver_inp_fname; }

    bool Get_powertrain_status_bool(){ return this->powertrain_status_bool; }
    std::string Get_powertrain_status_fname(){ return this->powertrain_status_fname; }

    bool Get_tire_force_bool();
    std::string Get_tire_fl_force_fname(); 
    std::string Get_tire_fr_force_fname(); 
    std::string Get_tire_rl_force_fname(); 
    std::string Get_tire_rr_force_fname(); 
    std::string Get_wheel_st_angle_fname();

    bool Get_fforce_output_bool();
    std::string Get_fforce_chassis_fname();

    bool Get_init_loc_pos_bool(){ return init_loc_pos_bool; };

    std::string Get_restart_output_fname(){ return this->restart_output_fname; }
    bool Get_coupling_info_bool() { return this->coupling_info_bool; }

    //Get Point vel and acc in cabin
    std::string Get_cabin_pdata_fname(){ return this->cabin_pdata_fname; }
    bool Get_cabin_pdata_bool() { return this->cabin_pdata_bool; }

    //fforce map
    std::string Get_fforce_map_fname(){ return this->fforce_map_fname; };
    bool Get_fforce_map_bool(){ return this->fforce_map_bool; };
    double Get_rho(){ return this->rho; }
    double Get_vpja(){ return this->vpja; }
    double Get_wb(){ return this->wb; }
    std::string Get_airspeed_fname(){ return this->airspeed_inp_fname; };
    bool Get_airspeed_bool(){ return this->airspeed_bool; };

    //data exchange
    bool Get_direc_Xaxis();
    bool Get_rot_Xaxis();
    bool Get_direc_Yaxis();
    bool Get_rot_Yaxis();
    bool Get_direc_Zaxis();
    bool Get_rot_Zaxis();

    //flow stabilize time
    double Get_flowstabi_time(){ return flow_stabi_time; }
   

    //set function
    std::string Set_str_value(std::stringstream &ss);
    int Set_int_value(std::stringstream &ss);
    double Set_double_value(std::stringstream &ss);
    bool Set_bool_value(std::stringstream &ss);
    ChVector<> Set_ChVector(std::stringstream &ss);
    ChQuaternion<> Set_ChQuaternion(std::stringstream &ss);
    std::vector<std::string> Set_str_vec(std::stringstream &ss);

};

}
}
#endif