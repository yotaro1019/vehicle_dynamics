#include<iostream>
#include<string>
#include<fstream>
#include<memory>

class Baseout{
  protected:
    bool c_switch;
    std::shared_ptr<std::fstream> fout;

  public:
    Baseout();
    virtual ~Baseout();    
    void check_file_status(std::shared_ptr<std::fstream> fout, std::string fname);
};