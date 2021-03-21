#ifndef _BASEOUT_
#define _BASEOUT_
#include<iostream>
#include<string>
#include<fstream>
#include<memory>


class Baseout{
  protected:
    bool c_switch;
    std::fstream fout;
    std::string  fname;
    void write_data(char data[]);
    bool checkFileExistence(const std::string& str);
 
  public:
    Baseout();
    virtual ~Baseout();    
    void check_file_status(std::string fname, char header[]);
    void restart(int step);
};

#endif