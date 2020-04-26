#ifndef _fforce_map_
#define _fforce_map_
#include<iostream>
#include<string>
#include<fstream>
#include"inp_init_data.h"
#include"exchange_data.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
namespace chrono{
namespace vehicle{

class  CH_VEHICLE_API  FForce_map{
public:
FForce_map(Input_data inp);
void Get_fforce_from_map(WheeledVehicle &veh, double time, Cfd2Vehicle *input_data);
void get_airspeed(double time, ChVector<> aspd_vel);

private:
std::vector<double> ang, cd, cs, cl, crm, cpm, cym;
std::vector<double> cd_coef, cs_coef, cl_coef, crm_coef, cpm_coef, cym_coef;
int nval;

double rho, vpja, wb; //rho, vehicle_project_area, wheel_base

bool map_switch;
void read_map_inp(std::string map_fname);
void calc_coeff(std::vector<double> vlist, std::vector<double>& coef);
void output_map_data();
double linear_interpolation(double x0, double x1, double y0, double y1, double x);
double polynomial_approximation(std::vector<double> coef, double x);

double Get_Cdmap(double deg);
double Get_Csmap(double deg);
double Get_Clmap(double deg);
double Get_Crmmap(double deg);
double Get_Cpmmap(double deg);
double Get_Cymmap(double deg);

bool x_direc_Xaxis, y_direc_Xaxis, z_direc_Xaxis;
bool x_rot_Xaxis, y_rot_Xaxis, z_rot_Xaxis;


bool airspeed_switch;
void read_airspeed_inp(std::string fname);
int aspd_step;
std::vector<double> aspd_time, aspd_uu, aspd_vv, aspd_ww; 


};

}// end namespace vhicle
}// end namespace chrono


#endif