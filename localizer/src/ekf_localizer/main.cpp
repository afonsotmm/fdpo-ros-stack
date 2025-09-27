#include "ekf_localizer/localizer_node.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "localizer_node");
    ros::NodeHandle nh;
    LocalizerNode node(nh);
    ros::spin();

    return 0;
}