#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "arrayPublisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Int8MultiArray>("/some_publisher", 100);

  while (ros::ok())
  {
    // some vectors of vectors here
    std::vector < std::vector<int> > matrix;

    std_msgs::Int8MultiArray matrix_ma;
    matrix_ma.data.clear();
    int WIDTH_MATRIX = 200;
    int HEIGHT_MATRIX = 200;
    // linearise matrix
    for (int i = 0; i < WIDTH_MATRIX*HEIGHT_MATRIX; i++) {
      matrix_ma.data.push_back(matrix[i / WIDTH_MATRIX][i % WIDTH_MATRIX]);
    }

    pub.publish(matrix_ma);
    ROS_INFO("TEST: Outer layer has been published");
    ros::spinOnce();
    sleep(1);
  }
}
