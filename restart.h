#include"baseout.h"

class Restart : public Baseout{
  public:
    Restart();
    void output_initialize();
    void output_write(double act_force[6]);

};