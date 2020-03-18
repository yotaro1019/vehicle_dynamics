#include "vehicle_core.h"

int main(){
    Vehicle_model vehicle;
    vehicle.vehicle_initialize();

    double time = 0.0;
    double dt = 0.001;
    double t_end = 3.0;

    while(time <= t_end){
        vehicle.vehicle_advance();
        time += dt;
    }

}