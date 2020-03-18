#include "vehicle_core.h"
#include<math.h>


int main(){
    Vehicle_model vehicle;
    vehicle.vehicle_initialize();

    double time = 0.0;
    double dt = 0.001;
    double t_end = 3.0;
    double fforce[6];
    int step= 0;
    while(time <= t_end){
        double seed = 100*sin(M_PI * step/15000);
        for(int i = 0; i<6;i++){
            fforce[i] = (i+1) * seed;
        }
        vehicle.vehicle_advance(fforce);
        time += dt;
        step++;
    }

}