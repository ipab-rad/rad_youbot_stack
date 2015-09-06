#include <simple_map/outer_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(outer_layer::OuterLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace outer_layer {

OuterLayer::OuterLayer() {}

void OuterLayer::onInitialize() {
  costmap_init_ = false;
  // data_matrix.resize(MATRIX_HEIGHT, std::vector<int>(MATRIX_WIDTH, 0));
  ros::NodeHandle node;
  // layer_sub_ = node.subscribe("/outer_layer", 400, &OuterLayer::layerCB, this);
  ROS_INFO("Interactive Costmap setup");
  // agent_sub_ = node.subscribe("/people", 400, &OuterLayer::peopleCB, this);
  predict_sub_ = node.subscribe("/interactive_prediction", 400,
                                &OuterLayer::predictCB, this);

  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
    boost::bind(
      &OuterLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void OuterLayer::predictCB(
  const youbot_navigation_global::InteractivePrediction::ConstPtr& msg) {
  costmap_init_ = true;
  predict_msg_ = *msg;
}

void OuterLayer::updateBounds(double origin_x, double origin_y,
                              double origin_yaw, double* min_x,
                              double* min_y, double* max_x, double* max_y) {
  if (!enabled_)
    return;
  double MW = 1000.0;
  double MH = 1000.0;
  *min_x = std::min(*min_x, (double) - MW);
  *min_y = std::min(*min_y, (double) - MH);
  *max_x = std::max(*max_x, (double) MW);
  *max_y = std::max(*max_y, (double) MH);
}

void OuterLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                             int min_j, int max_i,
                             int max_j) {
  if (!costmap_init_) { return; }
  if (!enabled_) {return;}

  size_t mx;
  size_t my;

  // Agent prediction cost layer
  for (size_t agent = 0; agent < predict_msg_.agent.size(); ++agent) {
    for (size_t i = 0; i < predict_msg_.foresight; ++i) {
      double wx = predict_msg_.agent[agent].pose[i].x;
      double wy = predict_msg_.agent[agent].pose[i].y;
      if (master_grid.worldToMap(wx, wy, mx, my)) {
        // ROS_INFO("wx: %f, wy: %f, mx: %u, my: %u", wx, wy, mx, my);
        master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void OuterLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                               uint32_t level) {
  enabled_ = config.enabled;
}

} // end namespace
