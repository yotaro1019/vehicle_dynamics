#include<iostream>
#include"c_vehicle_core.h"


int main(){
    vehicle_initialize_stand_alone();

    std::cout << "START\n";
    for(int i=0; i<10000; i++){
        std::cout << " \n";
        vehicle_advance_stand_alone();
    }

}