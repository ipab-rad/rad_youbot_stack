#include <rad_layer/foresight_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(foresight_layer::ForesightLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace foresight_layer {

ForesightLayer::ForesightLayer() {}

void ForesightLayer::onInitialize() {
  costmap_init_ = false;
  do_once_ = false;
  def_value_ = 63;
  g_radius_ = 20;
  std_dev2_ = 50.0;
  min_cost_ = 1.0;
  max_cost_ = 127.0;
  save_map_ = false;
  ros::NodeHandle node;
  ROS_INFO("Interactive Costmap setup");
  predict_sub_ = node.subscribe("/interactive_prediction", 400,
                                &ForesightLayer::predictCB, this);

  ros::NodeHandle nh("~/" + name_);
  current_ = true; // DO NOT REMOVE!
  // global_pose.x = 0.0; // DO NOT REMOVE!
  // pose_y_ = 0.0; // DO NOT REMOVE!
  // updated_pose_ = false; // DO NOT REMOVE!
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
    boost::bind(
      &ForesightLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void ForesightLayer::predictCB(
  const youbot_navigation_global::InteractivePrediction::ConstPtr& msg) {
  costmap_init_ = true;
  predict_msg_ = *msg;
}

void ForesightLayer::updateBounds(double origin_x, double origin_y,
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

void ForesightLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j,
                                 int max_i, int max_j) {
  if (!costmap_init_) { return; }
  if (!enabled_) {return;}
  if (!do_once_) {
    // master_grid.saveMap("test_map.pgm");
    master_grid.setDefaultValue((unsigned char)def_value_);
    // ROS_INFO_STREAM("DefVal: " << (int)master_grid.getDefaultValue());
    do_once_ = true;
  }

  // Agent prediction cost layer
  for (size_t agent = 0; agent < predict_msg_.agent.size(); ++agent) {
    for (size_t i = 0; i < predict_msg_.foresight; ++i) {
      double wx = predict_msg_.agent[agent].pose[i].x;
      double wy = predict_msg_.agent[agent].pose[i].y;
      size_t mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {

        for (int i = -g_radius_; i < g_radius_; ++i) {
          for (int j = -g_radius_; j < g_radius_; ++j) {
            unsigned char old_cost = master_grid.getCost(i + mx, j + my);
            if (old_cost == costmap_2d::NO_INFORMATION) {continue;}

            double a = this->gaussian(i, j, 0, 0, def_value_);
            if (agent == 0) {
              double cvalue = def_value_ - a;
              double new_cost = std::min(cvalue, (double) old_cost);
              master_grid.setCost(i + mx, j + my,
                                  (unsigned char) std::max(new_cost, min_cost_));
            } else {
              double cvalue = def_value_ + a;
              double new_cost = std::max(cvalue, (double) old_cost);
              master_grid.setCost(i + mx, j + my,
                                  (unsigned char) std::min(new_cost, max_cost_));
            }
            // ROS_INFO_STREAM("CV: " << (int)cvalue << " OC: " << (int)old_cost
            //                 << " MX: " << (int)std::max(cvalue, old_cost));
          }
        }

      }
    }
  }
  if (save_map_) {master_grid.saveMap("test_map.pgm");}
}

double ForesightLayer::gaussian(double x, double y, double x0, double y0,
                                double A) {
  double t1 = pow(x - x0, 2.0f) / std_dev2_;
  double t2 = pow(y - y0, 2.0f) / std_dev2_;
  double e = exp(-0.5 * (t1 + t2));
  double cost = A * e;
  return cost;
}

void ForesightLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                                   uint32_t level) {
  enabled_ = config.enabled;
}
} // end namespace
