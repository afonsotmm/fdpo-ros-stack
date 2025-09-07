#pragma once

#include <ros/ros.h>

class Fsm {
    
    public:

        int state, new_state, prev_state;
        ros::Time tes;
        ros::Duration tis; 
       
        Fsm(int initial_state);
        void reset_times();
        void set_state();
        void update_tis();
        bool check_timeout(double timeout);

};

