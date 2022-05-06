#ifndef _vehicle_restart_
#define _vehicle_restart_

#include "calculation_mode.h"
#include"inp_init_data.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "output.h"
#include "point_vel_acc.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;

class Restart : public Set_value{
  private:
  int output_itvl;
  std::string out_dir;
  bool restart_switch;
  bool restart_initialization;
  int restart_step;
  char restart_fname[500];

  struct Powertrain_info{
    std::string TemplateName;
    ChPowertrain::DriveMode DriveMode;
  };
  
  private:
  void read_from_file(ChState &state_pos, ChStateDelta &state_vel, ChStateDelta &state_acc, ChVectorDynamic<> &state_reactions, ChDriver::Inputs &driver_inputs, double &T, Powertrain_info &pt_info);
  void rebuild_powertrain(Powertrain_info &pt_info, WheeledVehicle &veh);
  void output_powertrain(std::ofstream &out, WheeledVehicle &veh);

  public:
    Restart(Input_data &inp, int &step);
    void rebuild_system(double &time, WheeledVehicle &veh, ChPathFollowerDriver &driver, RigidTerrain &terrain, Output &out, Point_vel_acc &point_vel_acc);
    void output(WheeledVehicle &veh, ChDriver::Inputs driver_inputs, int step, double time);

};

#endif