#ifndef _vehicle_restart_
#define __vehicle_restart_

#include"baseout.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"


using namespace chrono;


class Restart : public Baseout{
  public:
    Restart();
    void output_initialize();
    void output_record_force_momennt(ChVector<> fforce, ChVector<> fmoment);

};

#endif