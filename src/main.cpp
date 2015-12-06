#include <src/ardrone.h>
int main(int argc, char **argv){
    ros::init(argc,argv,"opt_drone");
    cv::namedWindow(WINDOW);
    my_ardrone_node man;


    ros::spin();
    return 0;
    }
