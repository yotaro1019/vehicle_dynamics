#include<iostream>
#include<string>
#include<fstream>
#include"baseout.h"
#include<sstream>
#include <unistd.h>
Baseout::Baseout(){
    this->c_switch = false;
}

Baseout::~Baseout(){
    if(!this->c_switch)
        return;
        
    fout.close(); 
}

void Baseout::check_file_status(std::string fname, char header[]){
    if(!this->c_switch)
        return;

    if(this->checkFileExistence(fname) == false){
        std::ofstream outputfile(fname);
    }

    fout.open(fname.c_str());

    if(fout.fail()){
        std::cout << "cannot open " << fname << "\n";
        this->c_switch = false;
        exit(1);
    }  
    this->fname  = fname;
    this->write_data(header);

}



///protected
void Baseout::write_data(char data[]){
    if(!this->c_switch)
        return;

    fout << data << "\n";
}

bool Baseout::checkFileExistence(const std::string& str){


    std::ifstream ifs(str);
    return ifs.is_open();
}

void Baseout::restart(int step){
    if(!this->c_switch)
        return;

    std::string str;


    while(getline(this->fout,str)){
        std::stringstream ss;
        std::string name;  
        ss << str;
        ss >> name;
        if(atoi(name.c_str()) ==step){
            break;
        }
    }    
    
}

