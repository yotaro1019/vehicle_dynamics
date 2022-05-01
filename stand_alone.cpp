#include<iostream>
#include"c_vehicle_core.h"


int main(int argc, char* argv[]){
    int end_step = atoi(argv[1]);
    
    std::cout << " end_step:" << end_step << "\n";
    vehicle_initialize_stand_alone();
    
    
    
    std::cout << "START\n";
    for(int i=0; i<end_step; i++){
        std::cout << " \n";
        vehicle_advance_stand_alone();
        
    }

}