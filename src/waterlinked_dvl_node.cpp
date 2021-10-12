#include "waterlinked_dvl/waterlinked_dvl.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "waterlinked_dvl");

    WaterlinkedDvl w;

    ros::spin();

    return 0;
}
