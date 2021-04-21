#include"exchange_data.h"
#include<iostream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChFrameMoving.h"

Exchange_data::Exchange_data(Input_data &inp, WheeledVehicle &veh){
    if(inp.Get_direc_Xaxis() == false)
        direction_axis[0] = -1.0;
    if(inp.Get_direc_Yaxis() == false)
        direction_axis[1] = -1.0;
    if(inp.Get_direc_Zaxis() == false)
        direction_axis[2] = -1.0;

    if(inp.Get_rot_Xaxis() == false)
        direction_rot[0] = -1.0;
    if(inp.Get_rot_Yaxis() == false)
        direction_rot[1] = -1.0;
    if(inp.Get_rot_Zaxis() == false)
        direction_rot[2] = -1.0;  
    
    this->steer_angles = Get_str_angles(veh);
}

void Exchange_data::conv_direction(Components &cmp){
    conv_translation(cmp.translation);
    conv_rotation(cmp.rotation);   
}

void Exchange_data::conv_translation(double data[3]){
    for(int i=0; i<3; i++){
        data[i] *= direction_axis[i];
    }
}

void Exchange_data::conv_rotation(double data[3]){
    for(int i=0; i<3; i++){
        data[i] *= direction_rot[i];
    }
}

std::vector<double> Exchange_data::Get_str_angles(WheeledVehicle &veh){
    int id = 0;
    std::vector<double> steer_angles;

    //cucl chassis yaw angle
    ChQuaternion<> chassis_rot;
    ChVector<> chassis_Yaxis;
    double chassis_glb_angle;
    
    chassis_rot = veh.GetVehicleRot();
    chassis_Yaxis = chassis_rot.GetYaxis();
    chassis_glb_angle = -atan(chassis_Yaxis.x() / chassis_Yaxis.y());

    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()) {
        for (std::shared_ptr< ChWheel > wheel : axle->GetWheels()){

            //culc wheel yaw angle
            ChQuaternion<> wheel_rot;
            ChVector<> wheel_Yaxis;
            double wheel_glb_angle_glb;

            wheel_rot = wheel->GetState().rot;
            wheel_Yaxis = wheel_rot.GetYaxis();
            wheel_glb_angle_glb = -atan(wheel_Yaxis.x() / wheel_Yaxis.y());  //wheel angle express by global cordniate

            double str_angle = wheel_glb_angle_glb - chassis_glb_angle;

            steer_angles.push_back(str_angle);
        }
    }

    return steer_angles;

}




void Exchange_data::data_unpacking(Cfd2Vehicle *input_data){
    //convert direction
    conv_direction(input_data->fforce);
}


//public
void Exchange_data::comp_zeros(Components &cmp){
    for(int i=0; i<3; i++){
        cmp.translation[i] = 0.0;
        cmp.rotation[i]    = 0.0;
    }

}
void Exchange_data::comp_set6array(Components &cmp, double array[6]){
    for(int i=0; i<3; i++){
        cmp.translation[i] = array[i];
        cmp.rotation[i]  = array[i+3];
    }
}

void Exchange_data::comp_get6array(Components &cmp, double array[6]){
    for(int i=0; i<3; i++){
        array[i] = cmp.translation[i];
        array[i+3] = cmp.rotation[i];
    }    
}

void Exchange_data::comp_get3transarray(Components &cmp, double array[3]){
    for(int i = 0; i<3; i++){
        array[i] = cmp.translation[i];
    }
} 

void Exchange_data::comp_get3rotarray(Components &cmp, double array[3]){
    for(int i = 0; i<3; i++){
        array[i] = cmp.rotation[i];
    }
}


//packing datas from vehicle instance to Vehicle2Cfd
void Exchange_data::data_packing(WheeledVehicle &veh,  Vehicle2Cfd *output_data, double adv_step_size){

    ChVector<> com_loc_poc = veh.GetChassis()->GetLocalPosCOM();
    ChVector<> vel_axis = veh.GetVehiclePointVelocity(com_loc_poc);
    ChVector<> acc_axis = veh.GetVehiclePointAcceleration(com_loc_poc);


    //mesh velocity
    output_data->mesh_vel.translation[0] = vel_axis.x();
    output_data->mesh_vel.translation[1] = -1.0 * vel_axis.y();
    output_data->mesh_vel.translation[2] = 0.0;
    conv_direction(output_data->mesh_vel);

    //mesh acceleration
    output_data->mesh_acc.translation[0] = acc_axis.x();
    output_data->mesh_acc.translation[1] = -1.0 * acc_axis.y();
    output_data->mesh_acc.translation[2] = 0.0;   
    conv_direction(output_data->mesh_acc);
    


//--------------chassis data------------------------------------------------------

    //Translation speed of chassis
    ChVector<> chassis_loc_vel = veh.GetChassisBody()->GetPos_dt();
    output_data->chassis_vel.translation[0] = 0.0;
    output_data->chassis_vel.translation[1] = 0.0;
    output_data->chassis_vel.translation[2] = chassis_loc_vel.z();

    //Rotational speed of chassis
    ChVector<> chassis_rot_vel_Euler = veh.GetChassisBody()->GetWvel_loc();
    output_data->chassis_vel.rotation[0] = -1.0 * chassis_rot_vel_Euler.x();
    output_data->chassis_vel.rotation[1] = -1.0 * chassis_rot_vel_Euler.y();
    output_data->chassis_vel.rotation[2] = -1.0 * chassis_rot_vel_Euler.z(); 
    conv_direction(output_data->chassis_vel);

//--------------culc steering angle velocity-----------------------------------------------    
    std::vector<double> steer_angles_current = Get_str_angles(veh);
    std::vector<double> steer_angle_vel;
    
    for(int i=0; i<steer_angles_current.size(); i++){
         double str_ang_vel_tmp = (steer_angles_current[i] - steer_angles[i]) / adv_step_size;
         steer_angle_vel.push_back(str_ang_vel_tmp);
    }
    
    this->steer_angles = steer_angles_current;

    //-------------------------------------------------

    int id = 0;

    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()) {
        for (std::shared_ptr< ChWheel > wheel : axle->GetWheels()){

            //ステアリング角の回転中心はchassisに固定
            output_data->str_vel[id].translation[0] = 0.0;
            output_data->str_vel[id].translation[1] = 0.0;
            output_data->str_vel[id].translation[2] = 0.0;

            output_data->str_vel[id].rotation[0] = 0.0;
            output_data->str_vel[id].rotation[1] = 0.0;
            output_data->str_vel[id].rotation[2] = -1.0 * steer_angle_vel[id];
            conv_direction(output_data->str_vel[id]);

            //step4　タイヤの回転角速度を取得
            output_data->wheel_vel[id].translation[0] = 0.0;
            output_data->wheel_vel[id].translation[1] = 0.0;
            output_data->wheel_vel[id].translation[2] = 0.0;
            output_data->wheel_vel[id].rotation[0] = 0.0;
            output_data->wheel_vel[id].rotation[1] = wheel->GetState().omega;
            output_data->wheel_vel[id].rotation[2] = 0.0;
            conv_direction(output_data->wheel_vel[id]);

            id++;
        }
    }

}