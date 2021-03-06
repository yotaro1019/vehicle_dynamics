#include<iostream>
#include<string>
#include<fstream>
#include"baseout.h"

Baseout::Baseout(){
    this->c_switch = false;
}

Baseout::~Baseout(){
    if(!this->c_switch)
        return;
        
    fout->close(); 
}

void Baseout::check_file_status(std::shared_ptr<std::fstream> fout, std::string fname){
    if(fout->fail()){
        std::cout << "cannot open " << fname << "\n";
        this->c_switch = false;
        exit(1);
    }  
}