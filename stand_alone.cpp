#include<iostream>
#include"c_vehicle_core.h"


int main(){
    int begin_step;
    int end_step;



    vehicle_initialize_stand_alone(begin_step, end_step);


    std::cout << "START\n";
    for(int i=begin_step; i<end_step; i++){
        std::cout << " \n";
        vehicle_advance_stand_alone();
    }

}