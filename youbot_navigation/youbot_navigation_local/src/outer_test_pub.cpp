#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8MultiArray.h"
#include <fstream>
#include <sstream>
#include <unistd.h>

using std::string;
typedef std::vector < std::vector <int> > Matrix;

#define DEBUG 1
int MATRIX_WIDTH = 200;
int MATRIX_HEIGHT = 200;

Matrix createVector(string f_s)
{
  Matrix m;
  m.resize(MATRIX_HEIGHT, std::vector<int>(MATRIX_WIDTH, 0));
  std::ifstream infile(f_s.c_str());
  if (!infile.good()) {
     throw std::invalid_argument( "File does not exist!" );
  }
  std::string line;
  while (std::getline(infile, line))
  {
    std::istringstream iss(line);
    int a, b;
    float c;
    if (!(iss >> a >> b >> c)) {continue;} // error
    m[a][b] = (int) ((c*100)+100)*1.27;
    if (DEBUG) {
      std::cout << "LINE: " << a << ", "
                << b << ", "
                << c << " -> "
                << m[a][b] << std::endl;
    }

  }

  return m;
}





int main(int argc, char **argv) {

  ros::init(argc, argv, "arrayPublisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Int8MultiArray>("/outer_layer", 400);

  char cwd[1024];
  getcwd(cwd, sizeof(cwd));
  string path(cwd);
  while (ros::ok())
  {
    // some vectors of vectors here
    // TODO: ALEX PUT YOUR VECTOR HERE!
    Matrix matrix = createVector(path + "/share/Model-A.txt");

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
