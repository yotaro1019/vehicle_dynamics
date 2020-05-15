#include"exchange_data.h"
#include<iostream>

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



void Exchange_data::conv_dir(double data[3]){
    for(int i=0; i<3; i++){
        data[i] *= direction_axis[i];
    }
}

void Exchange_data::conv_rot(double data[3]){
    for(int i=0; i<3; i++){
        data[i] *= direction_rot[i];
    }
}

void Exchange_data::data_unpacking(Cfd2Vehicle *input_data){
    //convert direction
    conv_dir(input_data->chassis_fforce);
    conv_rot(input_data->chassis_fmoment);

}


//packing datas from vehicle instance to Vehicle2Cfd
void Exchange_data::data_packing(WheeledVehicle &veh,  Vehicle2Cfd *output_data){
    
    ChVector<> com_pos = veh.GetVehicleCOMPos();
    ChVector<> vel_axis = veh.GetVehiclePointVelocity(com_pos);
    ChVector<> acc_axis = veh.GetVehicleAcceleration(com_pos);
    //mesh velocity
    output_data->mesh_vel[0] = vel_axis.x();
    output_data->mesh_vel[1] = vel_axis.y();
    output_data->mesh_vel[2] = 0.0;
    conv_dir(output_data->mesh_vel);

    //mesh acceleration
    output_data->mesh_acc[0] = acc_axis.x();
    output_data->mesh_acc[1] = acc_axis.y();
    output_data->mesh_acc[2] = 0.0;   
    conv_dir(output_data->mesh_acc);

    //for (std::shared_ptr< ChAxle > axle : veh->GetAxles()) {
//
    //         //each side has single tire
    //         if(axle->GetWheels().size() == 2){
    //             //task of 2 tires on the axle
//
//
    //         }else if(axle->GetWheels().size() == 4){
    //             //task of 4 tires on the axle
//
    //         }
    //         //each side has dual tire
    //         naxle++;
    //     }

    //-------------------------------------------------
    //step1　グローバル座標系でのボデーのヨー角を計算(x-y)
    //output_list.cppの36行目を参考にしてボデーのクオータニオンを取得
    //クオータニオンから，(x-y)平面上でのヨー角を取得

    //-------------------------------------------------
    for (std::shared_ptr< ChAxle > axle : veh->GetAxles()) {
        for (std::shared_ptr< Chwheel > wheel : axle->GetWheels()){
            //step2　各wheelのグローバル座標系でのヨー角(x-y)
            //output_list.cppの125行目を参考にしてwheelのクオータニオンを取得
            //クオータニオンから，(x-y)平面上でのヨー角を取得



            //step3
            //ボデーから見たwheelの相対的な運動を計算
        }
    }

}