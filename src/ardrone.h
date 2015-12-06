#ifndef DRONETEST_H_
#define DRONETEST_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

#define SPACE 1048608
#define ENTER 1048586
#define UP 1113938
#define DOWN 1113940
#define LEFT 1113937
#define RIGHT 1113939
#define Q_KEY 1048689
#define W_KEY 1048695
#define E_KEY 1048677
#define A_KEY 1048673
#define S_KEY 1048691
#define D_KEY 1048676
#define R_KEY 1048690

using namespace std;
using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="color Image";
static const char WINDOW3[]="opt Image";
static const std_msgs::Empty e;

class my_ardrone_node{

    VideoWriter vm_cflow;
    VideoWriter vm_range_flow;
    VideoWriter vm_rgb;
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    image_transport::Subscriber image_sub;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist fresh_vel;
    ros::Publisher vel_pub;
    Mat last_image;//上一张图片
    Mat current_image;//下一张图片
    bool vm_flag;
    void takeoff(){
        takeoff_pub.publish(e);
        vel_pub.publish(fresh_vel);

    }
    void land(){
        land_pub.publish(e);
    }
    void forward(float i){
        cmd_vel = fresh_vel;
        cmd_vel.linear.x = i;
        vel_pub.publish(cmd_vel);
    }
    void back(float i){
        cmd_vel = fresh_vel;

        cmd_vel.linear.x = -i;
        vel_pub.publish(cmd_vel);

    }
    void turn_left(float i){
        cmd_vel = fresh_vel;

        cmd_vel.angular.z = i;
        vel_pub.publish(cmd_vel);

    }
    void turn_right(float i){
        cmd_vel = fresh_vel;

        cmd_vel.angular.z = -i;
        vel_pub.publish(cmd_vel);

    }

    void hover(){
        cmd_vel = fresh_vel;
        vel_pub.publish(cmd_vel);
    }

    void left(float i){
        cmd_vel = fresh_vel;

        cmd_vel.linear.y = i;
        vel_pub.publish(cmd_vel);

    }

    void right(float i){
        cmd_vel = fresh_vel;
        cmd_vel.linear.y = -i;
        vel_pub.publish(cmd_vel);

    }

    void fly(float x, float y, float z, float rx, float ry, float rz){
        cmd_vel = fresh_vel;
        cmd_vel.linear.x = x;
        cmd_vel.linear.y = y;
        cmd_vel.linear.z = z;
        cmd_vel.angular.x = rx;
        cmd_vel.angular.y = ry;
        cmd_vel.angular.z = rz;
        vel_pub.publish(cmd_vel);
    }
    void process(const sensor_msgs::ImageConstPtr& cam_image);
public:
    /*
     *init the subscribers and publishers
     */
    my_ardrone_node(){

        ros::NodeHandle n;
        image_transport::ImageTransport it(n);
        image_sub = it.subscribe("/ardrone/image_raw",1,&my_ardrone_node::process,this);
        takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
        land_pub = n.advertise<std_msgs::Empty>("/ardrone/land",1);
        vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        vm_cflow.open("cflow.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10.0,Size(640,360),false);

        vm_range_flow.open("range_flow.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10.0,Size(640,360),false);
        vm_rgb.open("rgb.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10.0,Size(640,360));
        vm_flag = false;
    }





};

#endif
