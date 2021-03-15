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

void Baseout::check_file_status(std::string fname, char header[]){

    if(this->checkFileExistence(fname) == false){
        std::ofstream outputfile(fname);
    }

    fout.reset(new std::fstream(fname.c_str()) );

    if(fout->fail()){
        std::cout << "cannot open " << fname << "\n";
        this->c_switch = false;
        exit(1);
    }  
    *fout << header << "\n";

}



///protected
void Baseout::write_data(char data[]){
    *fout << data << "\n";
}

bool Baseout::checkFileExistence(const std::string& str)
{
    std::ifstream ifs(str);
    return ifs.is_open();
}

void Baseout::skip_line(int step){
    std::cout << step << "\n";

}
