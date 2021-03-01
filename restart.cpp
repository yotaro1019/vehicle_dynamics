//restart system
#include "restart.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"



Restart::Restart(Input_data &inp, int &step){
    this->output_itvl = inp.Get_restart_output_itvl();
    this->out_dir = GetChronoOutputPath() + "restart_files/";
        
    if (!filesystem::create_directory(filesystem::path(this->out_dir ))) {
        std::cout << "Error creating directory " << this->out_dir  << std::endl;
        return;
    }

    if(inp.Get_restart_step() == 0){
        restart_switch = false;
    }else{
        restart_switch = true;
        step = inp.Get_restart_step();
        sprintf(restart_fname, "restart_veh_%05d.txt",  inp.Get_restart_step());
    }

}


void Restart::rebuild_system(WheeledVehicle &veh, double &time){
    if(!restart_switch)
        return;
        
    GetLog() << "start restart system\n";
        
    ChState state_pos;
    ChStateDelta state_vel, state_acc;    
    double T;
    this->read_from_file(state_pos, state_vel, state_acc, T);
    time = T;
    veh.GetSystem()->StateScatter(state_pos, state_vel, T, true);
    veh.GetSystem()->StateScatterAcceleration(state_acc);

}

void Restart::output(WheeledVehicle &veh,  int current_step, double time){
    if(current_step%output_itvl == 0){
        double T;
        ChState state_pos;
        ChStateDelta state_vel, state_acc;
        ChVector<> irr_cam_pos;

        veh.GetSystem()->StateSetup(state_pos, state_vel, state_acc);
        veh.GetSystem()->StateGather(state_pos, state_vel, T);
        veh.GetSystem()->StateGatherAcceleration(state_acc);

        char rest_fout[500];
        sprintf(rest_fout, "restart_veh_%05d.txt",  current_step);
        std::ofstream out(this->out_dir  +  rest_fout);
        if(out.fail()){
            GetLog() << "cannot open " << rest_fout << "\n";
            exit(1);
        } 

  

        //write in output file
        out << "begin_informations\n";
        out << "T\t" << T << "\n";
        out << "step\t" << current_step << "\n";
        out << "time \t " << time << "\n";
        out << "end_informations\n\n\n"; 

        out << "begin_state_pos\n";
        out << state_pos << "\n";
        out << "end_state_pos\n\n\n";

        out << "begin_state_vel\n";
        out << state_vel << "\n";
        out << "end_state_vel\n\n\n";

        out << "begin_state_acc\n";
        out << state_acc << "\n";
        out << "end_state_acc\n\n\n";

        out.close();
    }

}


void Restart::read_from_file(ChState &state_pos, ChStateDelta &state_vel, ChStateDelta &state_acc, double &T){
    
    std::vector<double> pos_vec, vel_vec, acc_vec;
            
    GetLog() << "restart file \t" << this->out_dir  +  restart_fname << "\n";
    std::ifstream inp_param_file(this->out_dir  +  restart_fname);
    
    if(inp_param_file.fail()){
        std::cout << "ERROR unable to open " << restart_fname << "\n";
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
            GetLog() << "\n===========\nbegin_informations\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_informations" ){
                    GetLog() << "end_informations\n===========\n\n";
                    break;
                }

                if(val == "T"){
                    ss>>val;
                    T = std::stod(val);                   
                }
            }
        }


        if(val == "begin_state_pos"){
            GetLog() << "\n===========\nbegin_state_pos\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_state_pos" ){
                    GetLog() << "end_state_pos\n===========\n\n";
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    pos_vec.push_back(dbl);
                }
            }
        }

        if(val == "begin_state_vel"){
            GetLog() << "\n===========\nbegin_state_vel\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                   
                if(val == "end_state_vel" ){
                    GetLog() << "end_state_vel\n===========\n\n";
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    vel_vec.push_back(dbl);
                }
            }
        }

        if(val == "begin_state_acc"){
            GetLog() << "\n===========\nbegin_state_acc\n";
            while(getline(inp_param_file,str)){
                std::stringstream ss;
                std::string val;
                ss << str;
                ss >> val;                    
                if(val == "end_state_acc" ){
                    GetLog() << "end_state_acc\n===========\n\n";
                    break;
                }else{
                    double dbl;
                    dbl = std::stod(val);
                    acc_vec.push_back(dbl);
                }
            }
        }


    }


    state_pos.conservativeResize(pos_vec.size());
    state_vel.conservativeResize(vel_vec.size());
    state_acc.conservativeResize(acc_vec.size());

    for(int i=0; i<pos_vec.size(); i++){
        state_pos[i] = pos_vec[i];
    }
    for(int i=0; i<vel_vec.size(); i++){
        state_vel[i] = vel_vec[i];
    }
    for(int i=0; i<acc_vec.size(); i++){
        state_acc[i] = acc_vec[i];
    }



}
