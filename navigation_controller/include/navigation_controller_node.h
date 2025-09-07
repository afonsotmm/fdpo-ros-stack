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
#include <geometry_msgs/PoseStamped.h> 
#include <navigation_controller/NavigationControl.h> 
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "fsm.h"


struct Pose {

    double x, y, theta;

};

struct WayPoint {

    int id;
    Pose pose;
    bool align;
    bool backwards;

};

class NavigationController {

    public:
        NavigationController(ros::NodeHandle& nh_);

    private:
        ros::NodeHandle& nh;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        Pose poseDesiredMap;

        std::string mode; // "start" | "pause" | "unpause" | "stop""

        Fsm navigationFsm;
        // both with respect to the odom frame
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

        void hardStop();
        void setTheta();
        void goToXY();

        std::deque<WayPoint> route;
        void updateDesiredPose();
        bool desiredPoseFromMapToOdom();
        void loadRouteFromParameters();
        
        ros::Subscriber odomSub;
        void updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg);

        ros::Publisher velPub;
        void publishVel();

        ros::Timer controlTimer;
        void navigationFsmRunner(const ros::TimerEvent&);

        bool rvizGoalAppend;
        ros::Subscriber rvizGoalSub;
        void rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // Services        
        ros::ServiceServer controlSrv; 
        bool controlSrvCb(navigation_controller::NavigationControl::Request& req, navigation_controller::NavigationControl::Response& res);

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
