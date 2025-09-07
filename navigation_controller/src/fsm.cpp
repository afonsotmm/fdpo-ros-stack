#include "fsm.h"

// ---------------------------------------------------------------------------------------
//                                 Fsm Class 
// ---------------------------------------------------------------------------------------
Fsm::Fsm(int initial_state) : state(initial_state), new_state(initial_state), prev_state(initial_state), tes(ros::Time::now()), tis(ros::Duration(0)) {

    ROS_INFO("Fsm instance created.");

}

void Fsm::reset_times() {

    tes = ros::Time::now();
    tis = ros::Duration(0);

}

void Fsm::set_state() {

    if (state != new_state) {

        prev_state = state;
        state = new_state;
        reset_times();
        
    }
    
}

void Fsm::update_tis() {
    tis = ros::Time::now() - tes;
}

bool Fsm::check_timeout(double timeout) {
    return tis.toSec() > timeout; // toSec() method converts: ros::Duration -> double
}

