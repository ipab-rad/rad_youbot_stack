#ifndef OUTER_LAYER_H
#define OUTER_LAYER_H_

#include <iostream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32MultiArray.h>

namespace outer_layer {
class OuterLayer : public costmap_2d::Layer {
 public:
  OuterLayer();
  void receive_layer(const std_msgs::Int32MultiArray::ConstPtr& msg);
  virtual void onInitialize();
  virtual void updateBounds(double origin_x,
                            double origin_y,
                            double origin_yaw,
                            double* min_x,
                            double* min_y,
                            double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i,
                           int min_j,
                           int max_i,
                           int max_j);

 private:
  int MATRIX_WIDTH;
  int MATRIX_HEIGHT;
  bool is_matrix_initialised_;
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;

  ros::NodeHandle node_;
  ros::Subscriber sub_;
  double mark_x_, mark_y_;
  std::vector< std::vector<int> > data_matrix;
};
}

#endif
