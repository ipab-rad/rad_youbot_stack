#ifndef HINDSIGHT_LAYER_H_
#define HINDSIGHT_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32MultiArray.h>
#include <people_msgs/People.h>
#include <youbot_navigation_global/AgentPrediction.h>
#include <youbot_navigation_global/InteractivePrediction.h>

namespace foresight_layer {

class ForesightLayer : public costmap_2d::Layer {
 public:
  ForesightLayer();

  virtual void onInitialize();

  void predictCB(
    const youbot_navigation_global::InteractivePrediction::ConstPtr& msg);

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
  // Flags
  bool costmap_init_;

  // Constants

  // Variables
  double mark_x_, mark_y_;
  std::vector< std::vector<int> > data_matrix;

  // ROS
  ros::NodeHandle node_;
  ros::Subscriber layer_sub_;
  ros::Subscriber predict_sub_;

  youbot_navigation_global::InteractivePrediction predict_msg_;
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
};
}

#endif
