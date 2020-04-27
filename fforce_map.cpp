#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include <math.h>
#include<algorithm>
#include<iomanip>
#include"fforce_map.h"
#include"Eigen/Core"
#include"Eigen/LU"


namespace chrono{
namespace vehicle{

FForce_map::FForce_map(Input_data inp){
    this->map_switch = inp.Get_fforce_map_bool();
    this->airspeed_switch = inp.Get_airspeed_bool();
    if(!this->map_switch)
        return;

    this->rho = inp.Get_rho();
    this->vpja = inp.Get_vpja();
    this->wb = inp.Get_wb();
    x_direc_Xaxis = inp.Get_direc_Xaxis();
    y_direc_Xaxis = inp.Get_direc_Yaxis();
    z_direc_Xaxis = inp.Get_direc_Zaxis();
    x_rot_Xaxis = inp.Get_rot_Xaxis();
    y_rot_Xaxis = inp.Get_rot_Yaxis();
    z_rot_Xaxis = inp.Get_rot_Zaxis();

    read_map_inp(inp.Get_fforce_map_fname()); 
    output_map_data(); 
    GetLog() << "complete created fmap system";
    GetLog() << this->airspeed_switch;
    if(!this->airspeed_switch){
        GetLog() << "\n";
        return;
    }
    read_airspeed_inp(inp.Get_airspeed_fname());
    aspd_step = 0;
    GetLog() << "with Airspeed\n";


}

void FForce_map::read_map_inp(std::string map_fname){

    std::ifstream inp_fmap(map_fname);

    if(inp_fmap.fail()){
        std::cout << "ERROR unable to open " << map_fname << "\n";
    }
        
    GetLog() << "fforce map input \t" << map_fname << "\n";
    std::string str;
            
    while(getline(inp_fmap,str)){   
        std::stringstream ss;
        std::string stang, stcd, stcs, stcl, stcrm, stcpm, stcym;
        ss << str;
        ss >> stang;
        ss >> stcd;
        ss >> stcs;
        ss >> stcl;
        ss >> stcrm;
        ss >> stcpm;
        ss >> stcym;

        this->ang.push_back(std::stod(stang));
        this->cd.push_back(std::stod(stcd));
        this->cs.push_back(std::stod(stcs));
        this->cl.push_back(std::stod(stcl));
        this->crm.push_back(std::stod(stcrm));
        this->cpm.push_back(std::stod(stcpm));
        this->cym.push_back(std::stod(stcym));


    }
    this->nval = this->ang.size();
    calc_coeff(cd, cd_coef);    //cd
    calc_coeff(cs, cs_coef);    //cs
    calc_coeff(cl, cl_coef);    //cl
    calc_coeff(crm, crm_coef);    //crm
    calc_coeff(cpm, cpm_coef);    //cpm
    calc_coeff(cym, cym_coef);    //cym

}

void FForce_map::calc_coeff(std::vector<double> vlist, std::vector<double>& coef){
    //Eigen::MatrixXd amat(this->nval, this->nval);
    Eigen::MatrixXd amat(this->nval, this->nval);
    
    for(int i=0; i<this->nval; i++){
        for(int j=0; j<this->nval; j++){
            amat(i,j)= pow(ang[i], j);
        }
    }

    Eigen::VectorXd b(this->nval);
    for(int i=0; i<this->nval; i++){
        b(i) = vlist[i];
    }

    Eigen::VectorXd x = amat.fullPivLu().solve(b);

    for(int i=0; i<this->nval; i++){
        coef.push_back(x(i));
    }

}

double FForce_map::linear_interpolation(double x0, double x1, double y0, double y1, double x){
    Eigen::MatrixXd amat(2,2);
    for(int i=0; i<this->nval; i++){
        amat(0,0)= x0;
        amat(1,0)= x1;
        amat(0,1)= 1.0;
        amat(1,1)= 1.0;
    }
    Eigen::VectorXd b(2);
    b[0] = y0;
    b[1] = y1;
    Eigen::VectorXd coef = amat.fullPivLu().solve(b);
    double Fx = coef[0]*x + coef[1];
    return Fx;
}

double FForce_map::polynomial_approximation(std::vector<double> coef, double x){
    double Fx = 0.0;
    for(int i = 0; i < coef.size(); i++){
        Fx += coef[i] * pow(x, i);
    }
    return Fx;
}

void FForce_map::Get_fforce_from_map(WheeledVehicle &veh, double time, Cfd2Vehicle *input_data){
    if(!this->map_switch)
            return;

    ChVector<> aspd_vel(0.0, 0.0, 0.0);
    get_airspeed(time, aspd_vel);

    ChVector<> chassis_COM_pos = veh.GetChassis()->GetCOMPos();
    ChVector<> chassis_COM_vel = veh.GetChassis()->GetPointVelocity(veh.GetChassis()->GetLocalPosCOM() ) - aspd_vel;
    ChQuaternion<> chassis_COM_rot = veh.GetChassis()->GetCOMRot();
    ChVector<> chassis_COM_Xaxis = chassis_COM_rot.GetXaxis();
    double chassis_grobal_yaw = atan( chassis_COM_Xaxis.y() / chassis_COM_Xaxis.x() );
     if(std::isnan(chassis_grobal_yaw))
        chassis_grobal_yaw = 0.0;   
    double chassis_vel_yaw = atan( chassis_COM_vel.y() / chassis_COM_vel.x());
    double slip = (chassis_vel_yaw - chassis_grobal_yaw) * 180/M_PI;
    if(std::isnan(slip))
        slip = 0.00001;
    //double vel = veh.GetChassis()->GetSpeed();
    double vel =  chassis_COM_vel.Length(); 
    double fx = Get_Cdmap(slip)*(0.5*rho*pow(vel,2)*vpja);       //Fx
    double fy = Get_Csmap(slip)*(0.5*rho*pow(vel,2)*vpja);       //Fy
    double fz = Get_Clmap(slip)*(0.5*rho*pow(vel,2)*vpja);       //Fx
    double mx = Get_Crmmap(slip)*(0.5*rho*pow(vel,2)*vpja);     //Mx
    double my = Get_Cpmmap(slip)*(0.5*rho*pow(vel,2)*vpja*wb);  //My
    double mz = Get_Cymmap(slip)*(0.5*rho*pow(vel,2)*vpja);     //Mz
    if(!x_direc_Xaxis)
        fx *= -1.0;

    if(!y_direc_Xaxis)
        fy *= -1.0;

    if(!z_direc_Xaxis)
        fz *= -1.0;

    if(!x_rot_Xaxis)
        mx *= -1.0;

    if(!y_rot_Xaxis)
        my *= -1.0;

    if(!z_rot_Xaxis)
        mz *= -1.0;
    //convert L-RF => G-RF
    input_data->chassis_fforce[0] = fx*cos(chassis_grobal_yaw)-fy*sin(chassis_grobal_yaw);
    input_data->chassis_fforce[1] = fx*sin(chassis_grobal_yaw)+fy*cos(chassis_grobal_yaw);
    input_data->chassis_fforce[2] = fz;
    input_data->chassis_fmoment[0] = mx*cos(chassis_grobal_yaw)-my*sin(chassis_grobal_yaw);
    input_data->chassis_fmoment[1] = mx*sin(chassis_grobal_yaw)+my*cos(chassis_grobal_yaw);
    input_data->chassis_fmoment[2] = mz;
}

double FForce_map::Get_Cdmap(double deg){
    double Cd = 0.0;
    if(fabs(deg) > this->ang[1] and fabs(deg) < this->ang[this->ang.size()-2] ){
        Cd = this->polynomial_approximation(this->cd_coef, fabs(deg));
    }else if(fabs(deg) > this->ang[0] and fabs(deg) < this->ang[1]){
        Cd = this->linear_interpolation(this->ang[0], this->ang[1], this->cd[0], this->cd[1], fabs(deg) );
    }else if(this->ang[this->ang.size()-2] <  fabs(deg)){
        Cd = this->linear_interpolation(this->ang[this->ang.size()-2], this->ang[this->ang.size()-1], this->cd[this->ang.size()-2], this->cd[this->ang.size()-1], fabs(deg) );
    }else{
        GetLog() << "input deg = " << deg << "\ncannot calculate Cd from fmap\n";
    }
    return Cd;
}

double FForce_map::Get_Csmap(double deg){
    double Cs = 0.0;

    if(fabs(deg) > this->ang[1] and fabs(deg) < this->ang[this->ang.size()-2] ){
        Cs = this->polynomial_approximation(this->cs_coef, fabs(deg));
    }else if(fabs(deg) > this->ang[0] and fabs(deg) < this->ang[1]){
        Cs = this->linear_interpolation(this->ang[0], this->ang[1], this->cs[0], this->cs[1], fabs(deg) );
    }else if(this->ang[this->ang.size()-2] <  fabs(deg)){
        Cs = this->linear_interpolation(this->ang[this->ang.size()-2], this->ang[this->ang.size()-1], this->cs[this->ang.size()-2], this->cs[this->ang.size()-1], fabs(deg) );
    }else{
        GetLog() << "input deg = " << deg << "\ncannot calculate Cs from fmap\n";
    }
    
    if(deg < 0.0){
        Cs *= -1.0;
    }
    return Cs;
}

double FForce_map::Get_Clmap(double deg){
    double Cl = 0.0;
    if(fabs(deg) > this->ang[1] and fabs(deg) < this->ang[this->ang.size()-2] ){
        Cl = this->polynomial_approximation(this->cl_coef, fabs(deg));
    }else if(fabs(deg) > this->ang[0] and fabs(deg) < this->ang[1]){
        Cl = this->linear_interpolation(this->ang[0], this->ang[1], this->cl[0], this->cl[1], fabs(deg) );
    }else if(this->ang[this->ang.size()-2] <  fabs(deg)){
        Cl = this->linear_interpolation(this->ang[this->ang.size()-2], this->ang[this->ang.size()-1], this->cl[this->ang.size()-2], this->cl[this->ang.size()-1], fabs(deg) );
    }else{
        GetLog() << "input deg = " << deg << "\ncannot calculate Cl from fmap\n";
    }
    return Cl;
}

double FForce_map::Get_Crmmap(double deg){
    double Crm = 0.0;

    if(fabs(deg) > this->ang[1] and fabs(deg) < this->ang[this->ang.size()-2] ){
        Crm = this->polynomial_approximation(this->crm_coef, fabs(deg));
    }else if(fabs(deg) > this->ang[0] and fabs(deg) < this->ang[1]){
        Crm = this->linear_interpolation(this->ang[0], this->ang[1], this->crm[0], this->crm[1], fabs(deg) );
    }else if(this->ang[this->ang.size()-2] <  fabs(deg)){
        Crm = this->linear_interpolation(this->ang[this->ang.size()-2], this->ang[this->ang.size()-1], this->crm[this->ang.size()-2], this->crm[this->ang.size()-1], fabs(deg) );
    }else{
        GetLog() << "input deg = " << deg << "\ncannot calculate Crm from fmap\n";
    }
    
    if(deg < 0.0){
        Crm *= -1.0;
    }
    return Crm;
}

double FForce_map::Get_Cpmmap(double deg){
    double Cpm = 0.0;

    if(fabs(deg) > this->ang[1] and fabs(deg) < this->ang[this->ang.size()-2] ){
        Cpm = this->polynomial_approximation(this->cpm_coef, fabs(deg));
    }else if(fabs(deg) > this->ang[0] and fabs(deg) < this->ang[1]){
        Cpm = this->linear_interpolation(this->ang[0], this->ang[1], this->cpm[0], this->cpm[1], fabs(deg) );
    }else if(this->ang[this->ang.size()-2] <  fabs(deg)){
        Cpm = this->linear_interpolation(this->ang[this->ang.size()-2], this->ang[this->ang.size()-1], this->cpm[this->ang.size()-2], this->cpm[this->ang.size()-1], fabs(deg) );
    }else{
        GetLog() << "input deg = " << deg << "\ncannot calculate Cpm from fmap\n";
    }

    return Cpm;
}

double FForce_map::Get_Cymmap(double deg){
    double Cym = 0.0;

    if(fabs(deg) > this->ang[1] and fabs(deg) < this->ang[this->ang.size()-2] ){
        Cym = this->polynomial_approximation(this->cym_coef, fabs(deg));
    }else if(fabs(deg) > this->ang[0] and fabs(deg) < this->ang[1]){
        Cym = this->linear_interpolation(this->ang[0], this->ang[1], this->cym[0], this->cym[1], fabs(deg) );
    }else if(this->ang[this->ang.size()-2] <  fabs(deg)){
        Cym = this->linear_interpolation(this->ang[this->ang.size()-2], this->ang[this->ang.size()-1], this->cym[this->ang.size()-2], this->cym[this->ang.size()-1], fabs(deg) );
    }else{
        GetLog() << "input deg = " << deg << "\ncannot calculate Cym from fmap\n";
    }

    if(deg < 0.0){
        Cym *= -1.0;
    }
    return Cym;
}

void FForce_map::output_map_data(){
    GetLog() << ang[0] << "deg ~ " << ang[nval-1] << "deg\n";
    double dx = (ang[nval-1] - ang[0])/1000;

    std::ofstream outputfile( vehicle::GetoutputDirDataFile("ForceMap.out") );
    char header[500];
    sprintf(header, "%15s%15s%15s%15s%15s%15s%15s", "yaw","cd","cs","cl","crm","cpm","cym");
    outputfile << header << "\n"; 
    double deg = -50.0;
    for(int i=0; i<=1000; i++){
        char val[500];
        sprintf(val,"%15.7f%15.5f%15.7f%15.7f%15.7f%15.7f%15.7f",  deg , this->Get_Cdmap(deg), this->Get_Csmap(deg), this->Get_Clmap(deg), this->Get_Crmmap(deg), this->Get_Cpmmap(deg), this->Get_Cymmap(deg) );     
        outputfile << val << "\n";
        deg+=0.1;
    }
    outputfile.close();

}

void FForce_map::read_airspeed_inp(std::string fname){
    std::ifstream inp_aspd(fname);

    if(inp_aspd.fail()){
        std::cout << "\nERROR unable to open " << fname << "\n";
    }
        
    GetLog() << "airspeed input \t" << fname << "\n";
    std::string str;
            
    while(getline(inp_aspd,str)){   
        std::stringstream ss;
        std::string time, u, v, w;
        ss << str;
        ss >> time;
        ss >> u;
        ss >> v;
        ss >> w;

        this->aspd_time.push_back(std::stod(time));
        this->aspd_uu.push_back(std::stod(u));
        this->aspd_vv.push_back(std::stod(v));
        this->aspd_ww.push_back(std::stod(w));
    }
}

void FForce_map::get_airspeed(double time, ChVector<> aspd_vel){
    if(!this->airspeed_switch){
        GetLog() << "\n";
        return;
    }
    while (time < aspd_time[aspd_step]){
        aspd_step++;
    }
    
    aspd_vel[0] = linear_interpolation(aspd_time[aspd_step], aspd_time[aspd_step+1], aspd_uu[aspd_step], aspd_uu[aspd_step+1], time);
    aspd_vel[1] = linear_interpolation(aspd_time[aspd_step], aspd_time[aspd_step+1], aspd_vv[aspd_step], aspd_vv[aspd_step+1], time);
    aspd_vel[2] = linear_interpolation(aspd_time[aspd_step], aspd_time[aspd_step+1], aspd_ww[aspd_step], aspd_ww[aspd_step+1], time);

}


}// end namespace vhicle
}// end namespace chrono