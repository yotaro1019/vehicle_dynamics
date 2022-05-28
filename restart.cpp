//restart system
#include "restart.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include<iomanip>



Restart::Restart(Input_data &inp, int &step){
    this->output_itvl = inp.Get_restart_output_itvl();
    this->out_dir = GetChronoOutputPath() + "restart_files/";
        
    if (!filesystem::create_directory(filesystem::path(this->out_dir ))) {
        std::cout << "Error creating directory " << this->out_dir  << std::endl;
        return;
    }

    this->restart_step = inp.Get_restart_step();
    this->restart_initialization = inp.Get_restart_initialization();
    if(this->restart_step < 0){
        restart_switch = false;
    }else{
        restart_switch = true;
        step = this->restart_step;
        sprintf(restart_fname, "restart_veh_%05d.txt",  this->restart_step);
    }
    
    if(this->restart_initialization == true){
        step = 0;
    }

}


void Restart::rebuild_system(double &time, WheeledVehicle &veh, Driver_model_controller &driver, RigidTerrain &terrain, Output &out, Point_vel_acc &point_vel_acc){
    double tmp_step = 1e-20;
    
    if(!restart_switch)
        return;
    
    GetLog() << "\n\n=======================================================================\n";
    GetLog() << "start restart system\n";
    ChDriver::Inputs driver_inputs = driver.GetInputs();
    veh.Synchronize(time, driver_inputs, terrain);
    veh.Advance(tmp_step);

    ChState state_pos;
    ChStateDelta state_vel, state_acc; 
    ChVectorDynamic<> state_reactions;   
    Powertrain_info pt_info;
    double T;
    this->read_from_file(state_pos, state_vel, state_acc, state_reactions, driver_inputs, T, pt_info);
    
    if (this->restart_initialization == true){
        T =0.0;
    }
    time = T;

    veh.GetSystem()->StateScatter(state_pos, state_vel, T, true);
    veh.GetSystem()->StateScatterAcceleration(state_acc);
    veh.GetSystem()->StateScatterReactions(state_reactions);
    veh.GetSystem()->Update();

    //reset tergrt culclation points
    driver.reset(veh);
    driver.SetInputs(driver_inputs.m_steering, driver_inputs.m_throttle, driver_inputs.m_braking);

    //rebuild tire system
    for (std::shared_ptr< ChAxle > axle : veh.GetAxles()){
        int ntire =  axle->GetWheels().size();
        if(ntire == 2){
            //LEFT
            std::shared_ptr<ChTire> tire_L = axle->GetWheel(LEFT, SINGLE)->GetTire();    //ChWheel
            tire_L->Synchronize(time, terrain);
            tire_L->Advance(tmp_step);
            
            //RIGHT
            std::shared_ptr<ChTire> tire_R = axle->GetWheel(RIGHT, SINGLE)->GetTire();    //ChWheel
            tire_R->Synchronize(time, terrain);
            tire_R->Advance(tmp_step);

        }else if(ntire == 4){
            //LEFT inside
            std::shared_ptr<ChTire> tire_LIN = axle->GetWheel(LEFT, INNER)->GetTire();    //ChWheel
            tire_LIN->Synchronize(time, terrain);
            tire_LIN->Advance(tmp_step);

            //LEFT outside
            std::shared_ptr<ChTire> tire_LOUT = axle->GetWheel(LEFT, OUTER)->GetTire();    //ChWheel
            tire_LOUT->Synchronize(time, terrain);
            tire_LOUT->Advance(tmp_step);

            //RIGHT inside
            std::shared_ptr<ChTire> tire_RIN = axle->GetWheel(RIGHT, INNER)->GetTire();    //ChWheel
            tire_RIN->Synchronize(time, terrain);
            tire_RIN->Advance(tmp_step);

            //RIGHT outside
            std::shared_ptr<ChTire> tire_ROUT = axle->GetWheel(RIGHT, OUTER)->GetTire();    //ChWheel
            tire_ROUT->Synchronize(time, terrain);
            tire_ROUT->Advance(tmp_step);
        }
    }
    GetLog() << "\n\n=======================================================================\n";
    

    //prepare restart @ output files
    if (this->restart_initialization == false){
        out.restart(this->restart_step);
        point_vel_acc.restart(this->restart_step);
    }


}



void Restart::output(WheeledVehicle &veh, ChDriver::Inputs driver,  int current_step, double time){
    if(current_step%output_itvl == 0){
        double T;
        ChState state_pos;
        ChStateDelta state_vel, state_acc, dydt;
        ChVectorDynamic<> state_reactions(veh.GetSystem()->GetNconstr());

        veh.GetSystem()->StateSetup(state_pos, state_vel, state_acc);
        veh.GetSystem()->StateGather(state_pos, state_vel, T);
        veh.GetSystem()->StateGatherAcceleration(state_acc);
        veh.GetSystem()->StateGatherReactions(state_reactions);

        char rest_fout[500];
        sprintf(rest_fout, "restart_veh_%05d.txt",  current_step);
        std::ofstream out(this->out_dir  +  rest_fout);
        if(out.fail()){
            GetLog() << "cannot open " << rest_fout << "\n";
            exit(1);
        } 

  
        int output_precision = 10;
        //write in output file
        out << "begin_informations\n";
        out << "T\t" << T << "\n";
        out << "step\t" << current_step << "\n";
        out << "time \t " << time << "\n";
        out << "acc n_coord\t" <<  veh.GetSystem()->GetNcoords_a() << "\n";
        out << "n_coord\t" <<  veh.GetSystem()->GetNcoords_y() << "\n";
        out << "vel n_coord\t" <<  veh.GetSystem()->GetNcoords_dy() << "\n";
        out << "end_informations\n\n\n"; 

        out << "begin_state_pos\n";
        for(int i = 0; i<state_pos.size(); i++){
            out << std::setprecision(output_precision) <<  state_pos[i] << "\n";
        }
        out << "end_state_pos\n";
        out << "\n\n";

        GetLog() << std::setprecision(output_precision) << "state_pos\t"<< state_pos[1] << "\ttype_id : " << typeid(state_pos[1]) << "\n";
        out << "begin_state_vel\n";
        for(int i = 0; i<state_vel.size(); i++){
            out << std::setprecision(output_precision) <<  state_vel[i] << "\n";
        }        
        out << "end_state_vel\n";
        out << "\n\n";

        out << "begin_state_acc\n";
        for(int i = 0; i<state_acc.size(); i++){
            out << std::setprecision(output_precision) <<  state_acc[i] << "\n";
        }             
        out << "end_state_acc\n";
        out << "\n\n";


        out << "begin_state_reactions\n";
        for(int i = 0; i<state_reactions.size(); i++){
            out << std::setprecision(output_precision) <<  state_reactions[i] << "\n";
        }       
        out << "end_state_reactions\n";
        out << "\n\n"; 

        out << "begin_driver_input\n";
        out <<  std::setprecision(output_precision) << "throttle\t" << driver.m_throttle << "\n"; 
        out << std::setprecision(output_precision) <<  "steering\t" << driver.m_steering << "\n";
        out << std::setprecision(output_precision) <<  "braking \t" << driver.m_braking << "\n";
        out << std::setprecision(output_precision) <<  "end_driver_input\n\n";

        this->output_powertrain(out, veh);

        out.close();

    }

}


void Restart::output_powertrain(std::ofstream &out, WheeledVehicle &veh){
    std::string pt_TemplateName = veh.GetPowertrain()->GetTemplateName();
    
    out << "begin_powertrain\n";
    out << "TemplateName\t" << pt_TemplateName << "\n"; 

    if(pt_TemplateName == "SimplePowertrain"){
        out << "DriveMode\t" << veh.GetPowertrain()->GetDriveMode() << "\n";
    }
    else{
        GetLog() << "Unimplementation output powertrain\n";
    }


    out << "end_powertrain\n\n";

}


void Restart::read_from_file(ChState &state_pos, ChStateDelta &state_vel, ChStateDelta &state_acc, ChVectorDynamic<> &state_reactions, ChDriver::Inputs &driver_inputs, double &T, Powertrain_info &pt_info){
    
    std::vector<double> pos_vec, vel_vec, acc_vec, reaction_vec;
            
    GetLog() << "restart file \t" << this->out_dir  +  restart_fname << "\n";
    std::ifstream inp_param_file(this->out_dir  +  restart_fname);
    
    if(inp_param_file.fail()){
        std::cout << "ERROR unable to open " << restart_fname << "\n";
        exit(1);
    }
        std::string str;

    while(getline(inp_param_file,str)){
        

        if(str[0] == '#')
            continue;
        if(str[0] == '!')
            continue;
        if(str.empty())
            continue;

        std::stringstream ss;
        std::string val;

        ss << str;
        ss >> val;

        if(val == "begin_informations"){
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_informations" ){
                    break;
                }

                if(val == "T"){
                    ss>>val;
                    T = std::stod(val);                   
                }
            }
        }


        if(val == "begin_state_pos"){
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_state_pos" ){
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    pos_vec.push_back(dbl);
                }
            }
        }

        if(val == "begin_state_vel"){
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                   
                if(val == "end_state_vel" ){
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    vel_vec.push_back(dbl);
                }
            }
        }

        if(val == "begin_state_acc"){
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_state_acc" ){
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    acc_vec.push_back(dbl);
                }
            }
        }

        if(val == "begin_state_reactions"){
            GetLog() << "\n===========\nbegin_state_reactions\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_state_reactions" ){
                    GetLog() << "end_state_reactions\n===========\n\n";
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    reaction_vec.push_back(dbl);
                }
            }
        }

        if(val == "begin_driver_input"){
            GetLog() << "\n===========\nbegin_driver_input\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_driver_input" ){
                    GetLog() << "end_driver_input\n===========\n\n";
                    break;
                }else{
                    if(val == "throttle"){
                        driver_inputs.m_throttle = Set_double_value(ss);
                    }
                    if(val == "steering"){
                        driver_inputs.m_steering = Set_double_value(ss);
                    }
                    if(val == "braking"){
                        driver_inputs.m_braking = Set_double_value(ss);
                    }
                }
            }

        }

        if(val == "begin_powertrain"){
            GetLog() << "\n===========\nbegin_powertrain\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_powertrain" ){
                    GetLog() << "end_powertrain\n===========\n\n";
                    break;
                }else{
                    if(val == "TemplateName"){
                        pt_info.TemplateName = Set_str_value(ss);
                    }
                    if(val == "DriveMode"){
                        int dvm_int = Set_int_value(ss);
                        pt_info.DriveMode = static_cast<ChPowertrain::DriveMode>(dvm_int);
                    }
                }
            }

        }


    }


    state_pos.conservativeResize(pos_vec.size());
    state_vel.conservativeResize(vel_vec.size());
    state_acc.conservativeResize(acc_vec.size());
    state_reactions.conservativeResize(reaction_vec.size());

    for(int i=0; i<pos_vec.size(); i++){
        state_pos[i] = pos_vec[i];
    }
    for(int i=0; i<vel_vec.size(); i++){
        state_vel[i] = vel_vec[i];
    }
    for(int i=0; i<acc_vec.size(); i++){
        state_acc[i] = acc_vec[i];
    }

    for(int i=0; i<reaction_vec.size(); i++){
        state_reactions[i] = reaction_vec[i];
    }
}
