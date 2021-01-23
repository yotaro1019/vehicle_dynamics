#ifndef _BASEOUT_
#define _BASEOUT_
#include<iostream>
#include<string>
#include<fstream>
#include<memory>

class Baseout{
  protected:
    bool c_switch;
    std::shared_ptr<std::ofstream> fout;

  public:
    Baseout();
    virtual ~Baseout();    
    void check_file_status(std::shared_ptr<std::ofstream> fout, std::string fname);
};
#endif