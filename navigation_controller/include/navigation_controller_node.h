//
//  Created by afonso on 07/09/2025
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h> 
#include <cmath>
#include <algorithm>
#include <deque> 
#include <XmlRpcValue.h>

#include "fsm.h"


struct Pose {

    double x, y, theta;

};

struct WayPoint {

    int id;
    Pose pose;
    bool align;

};

class NavigationController {

    public:
        NavigationController(ros::NodeHandle& nh_);

    private:
        ros::NodeHandle& nh;

        Fsm navigationFsm;
        Pose poseCurr, poseDesired;
        double v_d, w_d;
        
        struct Parameters {

            double v_nom, w_nom;
            double k_p;          
            double arrive_radius, yaw_tol;
            int loop_rate_hz;

        };

        Parameters param;
        void loadNavigationParameters();

        double normalizeAngle(double theta);

        bool checkPositionArrived();
        bool checkYawArrived();

        void setTheta();
        void goToXY();

        std::deque<WayPoint> route;
        bool goSignal;
        void updateDesiredPose();
        void loadRouteFromParameters();
        
        ros::Subscriber odomSub;
        void updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg);

        ros::Publisher velPub;
        void publishVel();

        ros::Timer controlTimer;
        void controlLoop(const ros::TimerEvent&);

};


namespace navigation {

    namespace states {

        enum {

            idle = 0,
            driveToGoal,
            turnToFinalYaw,
            done

        }; 
    }

}