#include "navigation_controller_node.h"

NavigationController::NavigationController(ros::NodeHandle& nh_) : nh(nh_), v_d(0.0), w_d(0.0), navigationFsm(navigation::states::idle) {
    
    goSignal = false;

    // load parameters
    nh.param("v_nom", param.v_nom, 0.4);
    nh.param("w_nom", param.w_nom, 1.2);
    nh.param("k_p",   param.k_p,   5.0);
    nh.param("arrive_radius",  param.arrive_radius, 0.05);
    nh.param("yaw_tol",  param.yaw_tol, 0.08);
    nh.param("loop_rate_hz",   param.loop_rate_hz,  30);

    // ros init
    odomSub = nh.subscribe("/odom", 10, &NavigationController::updateCurrPose, this);
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    controlTimer = nh.createTimer(ros::Duration(1.0 / std::max(1, param.loop_rate_hz)), &NavigationController::controlLoop, this);

    // load route
    loadRouteFromParameters();

    ROS_INFO("NavigationController instace created");
}

void NavigationController::updateDesiredPose() {

    goSignal = false;

    if (!route.empty()) {
        
        goSignal = true;
        poseDesired = route.front().pose;

        ROS_INFO("New waypoint: x=%.2f y=%.2f yaw=%.2f", poseDesired.x, poseDesired.y, poseDesired.theta);
    }

}

void NavigationController::loadRouteFromParameters(){

    XmlRpc::XmlRpcValue waypoints;
    if(!nh.getParam("waypoints", waypoints)) return;

    route.clear();

    for(int i = 0; i < static_cast<int>(waypoints.size()); ++i){
        WayPoint waypoint_temp;

        waypoint_temp.pose.x = static_cast<double>(waypoints[i]["x"]);
        waypoint_temp.pose.y = static_cast<double>(waypoints[i]["y"]);
        waypoint_temp.pose.theta = static_cast<double>(waypoints[i]["yaw"]);
        waypoint_temp.align = static_cast<bool>(waypoints[i]["align"]);
        ROS_INFO("Waypoint: x=%.2f y=%.2f yaw=%.2f", waypoint_temp.pose.x, waypoint_temp.pose.y, waypoint_temp.pose.theta);

        route.push_back(waypoint_temp);

    }

    updateDesiredPose();

}

bool NavigationController::checkPositionArrived() {
    
    double position_error = std::hypot(poseDesired.x - poseCurr.x, poseDesired.y - poseCurr.y);

    if(position_error <= param.arrive_radius) return true;
    return false;

}

bool NavigationController::checkYawArrived() {

    double yaw_error = normalizeAngle(poseDesired.theta - poseCurr.theta); 

    if(std::fabs(yaw_error) <= param.yaw_tol) return true;
    return false;

}

void NavigationController::updateCurrPose(const nav_msgs::Odometry::ConstPtr& msg) {

    poseCurr.x = msg->pose.pose.position.x;
    poseCurr.y = msg->pose.pose.position.y;
    poseCurr.theta = tf2::getYaw(msg->pose.pose.orientation);

}

void NavigationController::publishVel() {

    geometry_msgs::Twist cmd;
    cmd.linear.x  = v_d;
    cmd.linear.y  = 0.0;
    cmd.linear.z  = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = w_d;
    velPub.publish(cmd);

}

double NavigationController::normalizeAngle(double theta) {

    while(theta > M_PI) theta -= 2.0 * M_PI;
    while (theta <= -M_PI) theta += 2.0*M_PI;

    return theta;
}

void NavigationController::setTheta() {

    v_d = 0.0;

    double yaw_error = normalizeAngle(poseDesired.theta - poseCurr.theta); 
    if(std::fabs(yaw_error) <= param.yaw_tol) {

        w_d = 0.0;

        return;
    }

    w_d = param.w_nom * yaw_error * 2 / M_PI;

    if(w_d > param.w_nom) w_d = param.w_nom;
    else if(w_d < -param.w_nom) w_d = -param.w_nom;

}

void NavigationController::goToXY() {

    double position_error = std::hypot(poseDesired.x - poseCurr.x, poseDesired.y - poseCurr.y);
    double theta_d = std::atan2(poseDesired.y - poseCurr.y, poseDesired.x - poseCurr.x);
    double yaw_error = normalizeAngle(theta_d - poseCurr.theta); 

    if(position_error <= param.arrive_radius) {

        v_d = 0.0;
        w_d = 0.0;
        
        return;
    }

    // angular
    w_d = param.w_nom * yaw_error * 8.0 / M_PI;

    if(w_d > param.w_nom) w_d = param.w_nom;
    else if(w_d < -param.w_nom) w_d = -param.w_nom;

    // linear
    if(std::fabs(yaw_error) > M_PI/8.0) v_d = 0.0;
    else if(yaw_error <= M_PI/8.0) v_d = param.v_nom * std::cos(yaw_error) * std::min<double>(1.0, param.k_p * position_error);


}

void NavigationController::controlLoop(const ros::TimerEvent&) {

    navigationFsm.update_tis();

    // Compute Transitions
    if(navigationFsm.state == navigation::states::idle && goSignal) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && checkPositionArrived() && route.front().align) {

        navigationFsm.new_state = navigation::states::turnToFinalYaw;

    }

    else if(navigationFsm.state == navigation::states::driveToGoal && checkPositionArrived() && !route.front().align) {

        route.pop_front();
        updateDesiredPose();

        navigationFsm.new_state = navigation::states::done;

    }

    else if(navigationFsm.state == navigation::states::turnToFinalYaw && checkYawArrived()) {

        route.pop_front();
        updateDesiredPose();
        
        navigationFsm.new_state = navigation::states::done;

    }

    else if(navigationFsm.state == navigation::states::done && goSignal) {

        navigationFsm.new_state = navigation::states::driveToGoal;

    }

    else if(navigationFsm.state == navigation::states::done && !goSignal) {

        navigationFsm.new_state = navigation::states::idle;

    }

    // Set states
    navigationFsm.set_state();

    // Compute Actions
    if(navigationFsm.state == navigation::states::driveToGoal) goToXY();
    else if(navigationFsm.state == navigation::states::turnToFinalYaw) setTheta();
    else v_d = w_d = 0.0;

    // Affect outputs
    publishVel();

}