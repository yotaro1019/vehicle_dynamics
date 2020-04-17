//restart system
#include "restart.h"

void Restart::Restart(){
    Baseout::Baseout(); 

}

void Restart::output_initialize(){
    std::string fname = "restart.out";
    fout.reset(new std::ofstream( fname.c_str() ) );
    check_file_status(fout, fname);
}


void Restart::output_write(double act_force[6]){

}