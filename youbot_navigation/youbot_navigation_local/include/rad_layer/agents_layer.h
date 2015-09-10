#ifndef AGENTS_LAYER_H_
#define AGENTS_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <people_msgs/People.h>

namespace agents_layer {

class AgentsLayer : public costmap_2d::Layer {
 public:
  AgentsLayer();

  virtual void onInitialize();
  void peopleCB(const people_msgs::People::ConstPtr& msg);

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
  bool do_once_;
  int p_radius_;
  int i_radius_;
  float res;

  // Variables
  people_msgs::People people_msg_;

  // ROS
  ros::NodeHandle node_;
  ros::Subscriber agent_sub_;
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
};
}
#endif
