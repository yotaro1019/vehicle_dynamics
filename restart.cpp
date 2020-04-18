//restart system
#include "restart.h"



Restart::Restart() : Baseout(){
     

}

void Restart::output_initialize(){
    this->c_switch = true;
    std::string fname = "restart.out";
    fout.reset(new std::ofstream( fname.c_str() ) );
    check_file_status(fout, fname);
}


void Restart::output_record_force_momennt(ChVector<> fforce, ChVector<> fmoment){

}