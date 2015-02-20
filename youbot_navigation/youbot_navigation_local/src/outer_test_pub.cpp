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

  ros::Publisher pub = n.advertise<std_msgs::Int8MultiArray>("/outer_layer", 400);

  while (ros::ok())
  {
    // some vectors of vectors here
    // TODO: ALEX PUT YOUR VECTOR HERE!
    std::vector < std::vector<int> > matrix;

    std_msgs::Int8MultiArray matrix_ma;
    matrix_ma.data.clear();
    // linearise matrix
    std::vector< std::vector<int> >::iterator r;
    std::vector<int>::iterator c;
    for (r = matrix.begin(); r != matrix.end(); r++) {
      for (c = r->begin(); c != r->end(); c++) {
        matrix_ma.data.push_back(*c);
      }
    }

    pub.publish(matrix_ma);
    ROS_INFO("TEST: Outer layer has been published");
    ros::spinOnce();
    sleep(1);
  }
}
