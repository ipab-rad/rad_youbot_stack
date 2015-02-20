#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "arrayPublisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("/some_publisher", 100);

  while (ros::ok())
  {
    vector < vector <Int> > V (
        std_msgs::Int8MultiArray matrix_ma;
        matrix.data.clear();
        matrix.data.push_back();

        for (int i = 0; i < w*h; i++) {
          oneDReversed[i] = twoD[(i / w)][(i%w)];
        }
        //for loop, pushing data in the size of the array
        for (int i = 0; i < 200; i++)
        {
          for (int j = 0; j < 200; j++) {
            //assign array a random number between 0 and 255.
            array.data.push_back(rand() % 255);
          }
        }
        //Publish array
        pub.publish(array);
        //Let the world know
        ROS_INFO("TEST: Outer layer has been published");
        //Do this.
        ros::spinOnce();
        //Added a delay so not to spam
        sleep(2);
        }

  }
