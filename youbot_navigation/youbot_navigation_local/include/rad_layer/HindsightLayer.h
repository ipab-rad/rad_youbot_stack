#ifndef HINDSIGHT_LAYER_H_
#define HINDSIGHT_LAYER_H_
#endif

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32MultiArray.h>

namespace rad_layer
{

class HindsightLayer : public costmap_2d::Layer
{
 public:
  HindsightLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  void receive_layer(const std_msgs::Float32MultiArray::ConstPtr& msg);
 private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  double agent_x_, agent_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  bool is_matrix_initialised;
  bool DEBUG;
  bool new_data_;
  std::vector< std::pair<float, float> > hindsight_data;
  ros::NodeHandle node_;
  ros::Subscriber sub_;
  int MATRIX_WIDTH;
  int MATRIX_HEIGHT;
  bool is_matrix_initialised_;
  bool updated_pose_;
  double pose_x_;
  double pose_y_;
};
}
