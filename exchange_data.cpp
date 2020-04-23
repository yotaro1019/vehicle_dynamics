#include"exchange_data.h"
#include<iostream>

Exchange_data::Exchange_data(Input_data &inp){
    if(inp.Get_direc_Xaxis() == false)
        direction_axis[0] =-1.0;
    if(inp.Get_rot_Xaxis() == false)
        direction_axis[1] = -1.0;
    if(inp.Get_direc_Yaxis() == false)
        direction_axis[2] = -1.0;

    if(inp.Get_rot_Yaxis() == false)
        direction_rot[0] = -1.0;
    if(inp.Get_direc_Zaxis() == false)
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
//packing datas from vehicle instance to Vehicle2Cfd
void Exchange_data::data_packing(WheeledVehicle &veh,  Vehicle2Cfd output_data){
    
    ChVector<> com_pos = veh.GetVehicleCOMPos();
    ChVector<> vel_axis = veh.GetVehiclePointVelocity(com_pos);
    ChVector<> acc_axis = veh.	GetVehicleAcceleration(com_pos);
    //mesh velocity
    veh2cfd.mesh_vel[0] = vel_axis.x();
    veh2cfd.mesh_vel[1] = vel_axis.y();
    veh2cfd.mesh_vel[2] = 0.0;
    conv_dir(veh2cfd.mesh_vel);

    //mesh acceleration
    veh2cfd.mesh_acc[0] = acc_axis.x();
    veh2cfd.mesh_acc[1] = acc_axis.y();
    veh2cfd.mesh_acc[2] = 0.0;   
    conv_dir(veh2cfd.mesh_acc);



    GetLog() << "data_packing\n";
    output_data = this->veh2cfd;

}