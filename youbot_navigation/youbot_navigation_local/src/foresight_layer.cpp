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
  updated_ = false;
  def_value_ = 63;
  g_radius_ = 16;
  std_dev2_ = 50.0;
  min_cost_ = 1.0;
  max_cost_ = 127.0;
  save_map_ = false;
  count_ = 0;
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
  updated_ = true;
}

void ForesightLayer::updateBounds(double origin_x, double origin_y,
                                  double origin_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y) {
  if (!enabled_)
    return;
  double MW = 2000.0;
  double MH = 2000.0;
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
    master_grid.setDefaultValue((unsigned char)def_value_);
    do_once_ = true;
  }

  // Agent interactive cost layer
  for (size_t agent = 0; agent < predict_msg_.agent.size(); ++agent) {
    for (size_t p = 0; p < predict_msg_.foresight; ++p) {
      double wx = predict_msg_.agent[agent].pose[p].x;
      double wy = predict_msg_.agent[agent].pose[p].y;
      size_t mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {

        for (int x = -g_radius_; x < g_radius_; ++x) {
          for (int y = -g_radius_; y < g_radius_; ++y) {
            unsigned char old_cost = master_grid.getCost(x + mx, y + my);
            if (old_cost == costmap_2d::NO_INFORMATION) {continue;}

            double a = this->gaussian(x, y, 0, 0, def_value_);
            double new_cost;
            if ((double)old_cost <= def_value_) {
              new_cost = (double)old_cost + a;
            } else {
              new_cost = std::max(def_value_ + a, (double) old_cost);
              //new_cost = def_value_ + a;
            }

            master_grid.setCost(x + mx, y + my,
                                (unsigned char) std::min(new_cost, max_cost_));
          }
        }
      }
    }
  }

  // Planner interactive reward layer
  // if (updated_) {
  for (size_t p = 0; p < predict_msg_.foresight; ++p) {
    double wx = predict_msg_.planner_pose[p].x;
    double wy = predict_msg_.planner_pose[p].y;
    size_t mx, my;
    if (master_grid.worldToMap(wx, wy, mx, my)) {

      for (int x = -g_radius_; x < g_radius_; ++x) {
        for (int y = -g_radius_; y < g_radius_; ++y) {
          unsigned char old_cost = master_grid.getCost(x + mx, y + my);
          if (old_cost == costmap_2d::NO_INFORMATION) {continue;}

          double a = this->gaussian(x, y, 0, 0, def_value_);
          // double cvalue = def_value_ - a;
          // double new_cost = std::min(cvalue, (double) old_cost);
          double new_cost;
          if ((double)old_cost >= def_value_) {
            new_cost = (double)old_cost - a;
          } else {
            new_cost = std::min(def_value_ - a, (double) old_cost);
          }

          master_grid.setCost(x + mx, y + my,
                              (unsigned char) std::max(new_cost, min_cost_));
        }
      }
    }
  }
  // }
  updated_ = false;
  if (save_map_) {
    count_++;
    master_grid.saveMap("test_map_" + boost::to_string(count_) + ".pgm");
  }
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
