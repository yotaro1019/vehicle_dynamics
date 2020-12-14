#include"exchange_data.h"
#include<iostream>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChFrameMoving.h"

Exchange_data::Exchange_data(Input_data &inp){
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

void Exchange_data::conv_direction(Components &cmp){
    conv_translation(cmp.translation);
    conv_rotation(cmp.rotation);   
}

void Exchange_data::data_unpacking(Cfd2Vehicle *input_data){
    //convert direction
    conv_direction(input_data->fforce);
}


//packing datas from vehicle instance to Vehicle2Cfd
void Exchange_data::data_packing(WheeledVehicle &veh,  Vehicle2Cfd *output_data){

    ChVector<> veh_vel = veh.GetChassisBody()->GetFrame_REF_to_abs().GetPos_dt();

    ChVector<> com_pos = veh.GetVehicleCOMPos();
    ChVector<> vel_axis = veh.GetVehiclePointVelocity(com_pos);
    ChVector<> acc_axis = veh.GetVehiclePointAcceleration(com_pos);


    //mesh velocity
    output_data->mesh_vel.translation[0] = vel_axis.x();
    output_data->mesh_vel.translation[1] = vel_axis.y();
    output_data->mesh_vel.translation[2] = 0.0;;
    conv_direction(output_data->mesh_vel);

    //mesh acceleration
    output_data->mesh_acc.translation[0] = acc_axis.x();
    output_data->mesh_acc.translation[1] = acc_axis.y();
    output_data->mesh_acc.translation[2] = 0.0;   
    conv_direction(output_data->mesh_acc);
    


//--------------chassis data------------------------------------------------------
    //Translation speed of chassis
    output_data->chassis_vel.translation[0] = 0.0;
    output_data->chassis_vel.translation[1] = 0.0;
    output_data->chassis_vel.translation[2] = veh_vel.z();

    //Rotational speed of chassis
    ChVector<> rot_Euler_vel = veh.GetChassisBody()->GetWvel_loc();
    output_data->chassis_vel.rotation[0] = 0.0; //rot_Euler_vel.x();
    output_data->chassis_vel.rotation[1] = 0.0; //rot_Euler_vel.y();
    output_data->chassis_vel.rotation[2] = 0.0; //rot_Euler_vel.z();   
    conv_direction(output_data->chassis_vel);
 	

//--------------culc steering angle-----------------------------------------------    
    ChVector<> chassis_rot_vel_gl = veh.GetChassisBody()->GetFrame_COG_to_abs().GetWvel_loc();
    GetLog() <<"chassis om_z = " << chassis_rot_vel_gl.z() << "\n";
    //-------------------------------------------------
    int id = 0;
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()) {
        for (std::shared_ptr< ChWheel > wheel : axle->GetWheels()){
            ChVector<> str_rot_vel_gl = wheel->GetSpindle()->GetFrame_COG_to_abs().GetWvel_loc();
            GetLog() <<"str_id = " << id << "\tom_z = " << str_rot_vel_gl.z() << "\n";
            //ステアリング角の回転中心はchassisに固定
            output_data->str_vel[id].translation[0] = 0.0;
            output_data->str_vel[id].translation[1] = 0.0;
            output_data->str_vel[id].translation[2] = 0.0;
 
            //step3
            //ボデーから見たwheelの相対的な運動を計算
            output_data->str_vel[id].rotation[0] = 0.0;
            output_data->str_vel[id].rotation[1] = 0.0;
            output_data->str_vel[id].rotation[2] = 0.0; // str_rot_vel_gl.z() - chassis_rot_vel_gl.z();
            conv_direction(output_data->str_vel[id]);

            //step4　タイヤの回転角速度を取得
            output_data->wheel_vel[id].translation[0] = 0.0;
            output_data->wheel_vel[id].translation[1] = 0.0;
            output_data->wheel_vel[id].translation[2] = 0.0;
            output_data->wheel_vel[id].rotation[0] = 0.0;
            output_data->wheel_vel[id].rotation[1] = 0.0; // wheel->GetState().omega;
            output_data->wheel_vel[id].rotation[2] = 0.0;
            conv_direction(output_data->wheel_vel[id]);

            id++;
        }
    }

}