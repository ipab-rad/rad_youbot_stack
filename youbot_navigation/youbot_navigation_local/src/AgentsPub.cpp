#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <fstream>
#include <sstream>
#include <unistd.h>

using std::string;

#define DEBUG 1
int MATRIX_WIDTH = 200;
int MATRIX_HEIGHT = 200;

int main(int argc, char **argv) {

  ros::init(argc, argv, "arrayPublisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/agents_layer", 20);

  while (ros::ok())
  {
    std_msgs::Float32MultiArray v;
    v.data.clear();
    v.data.push_back(1.0);
    v.data.push_back(1.0);
    v.data.push_back(3.0);
    v.data.push_back(3.0);
    v.data.push_back(5.0);
    v.data.push_back(5.0);
    v.data.push_back(7.0);
    v.data.push_back(7.0);
    v.data.push_back(11.0);
    v.data.push_back(11.0);
    v.data.push_back(13.0);
    v.data.push_back(13.0);
    v.data.push_back(15.0);
    v.data.push_back(15.0);
    pub.publish(v);
    ROS_INFO("TEST: Agents layer has been published");
    ros::spinOnce();
    sleep(5);
  }
}
