#include <rad_layer/foresight_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(foresight_layer::ForesightLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace foresight_layer {

ForesightLayer::ForesightLayer() {}

void ForesightLayer::onInitialize() {
  costmap_init_ = false; // DO NOT CHANGE!
  do_once_ = false; // DO NOT CHANGE!
  updated_ = false; // DO NOT CHANGE!
  def_value_ = 63; // DO NOT CHANGE!
  g_radius_ = 16;
  std_dev2_ = 50.0;
  save_map_ = false;
  min_cost_ = 1.0; // DO NOT CHANGE!
  max_cost_ = 127.0; // DO NOT CHANGE!
  count_ = 0; // DO NOT CHANGE!
  ros::NodeHandle node;
  ROS_INFO("Interactive Costmap setup");
  predict_sub_ = node.subscribe("model/interactive_prediction", 400,
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
  // ROS_ERROR("UPDATED!");
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
  ROS_INFO("BeginCostmap");

  float min_wx = 20.0f;
  float min_wy = 20.0f;
  float max_wx = -20.0f;
  float max_wy = -20.0f;

  youbot_navigation_global::InteractivePrediction predict_msg = predict_msg_;
  // Get min/max wx, wy values;
  // ROS_INFO_STREAM("ASize1: " << predict_msg.agent.size());
  for (size_t agent = 0; agent < predict_msg.agent.size(); ++agent) {
    for (size_t p = 0; p < predict_msg.foresight; ++p) {
      // ROS_INFO("Here-1");
      min_wx = std::min((float)predict_msg.agent[agent].pose[p].x, min_wx);
      max_wx = std::max((float)predict_msg.agent[agent].pose[p].x, max_wx);
      min_wy = std::min((float)predict_msg.agent[agent].pose[p].y, min_wy);
      max_wy = std::max((float)predict_msg.agent[agent].pose[p].y, max_wy);
      // ROS_INFO_STREAM("AX: " << (float)predict_msg.agent[agent].pose[p].x);
      // ROS_INFO_STREAM("AY: " << (float)predict_msg.agent[agent].pose[p].y);
    }
  }

  for (size_t p = 0; p < predict_msg.foresight; ++p) {
    min_wx = std::min((float)predict_msg.planner_pose[p].x, min_wx);
    max_wx = std::max((float)predict_msg.planner_pose[p].x, max_wx);
    min_wy = std::min((float)predict_msg.planner_pose[p].y, min_wy);
    max_wy = std::max((float)predict_msg.planner_pose[p].y, max_wy);
  }

  ROS_INFO_STREAM("MinWX: " << min_wx << " MinWY: " << min_wy);
  ROS_INFO_STREAM("MaxWX: " << max_wx << " MaxWY: " << max_wy);

  if ((max_wx > min_wx) || (max_wy > min_wy)) {
    size_t min_mx, min_my, max_mx, max_my;
    master_grid.worldToMap(min_wx, min_wy, min_mx, min_my);
    master_grid.worldToMap(max_wx, max_wy, max_mx, max_my);

    ROS_INFO_STREAM("MaxMX: " << max_mx << " MaxMY: " << max_my);
    ROS_INFO_STREAM("MinMX: " << min_mx << " MinMY: " << min_my);

    size_t layer_size_x = (max_mx + g_radius_) - (min_mx - g_radius_) + 1;
    size_t layer_size_y = (max_my + g_radius_) - (min_my - g_radius_) + 1;
    int off_mx = max_mx + g_radius_;
    int off_my = max_my + g_radius_;

    ROS_INFO_STREAM("LSizeX: " << layer_size_x << " LSizeY: " << layer_size_y);

    double cost_layer[layer_size_x][layer_size_y];
    double reward_layer[layer_size_x][layer_size_y];

    for (size_t ly = 0; ly < layer_size_y; ++ly) {
      for (size_t lx = 0; lx < layer_size_x; ++lx) {
        cost_layer[lx][ly] = 0.0;
        reward_layer[lx][ly] = 0.0;
      }
    }
    // Agent interactive cost layer
    // ROS_INFO_STREAM("ASize2: " << predict_msg.agent.size());
    for (size_t agent = 0; agent < predict_msg.agent.size(); ++agent) {
      for (size_t p = 0; p < predict_msg.foresight; ++p) {
        // ROS_INFO_STREAM("Here-2 a: " << agent << " p: " << p);
        double wx = (double)predict_msg.agent[agent].pose[p].x;
        double wy = (double)predict_msg.agent[agent].pose[p].y;
        // ROS_INFO_STREAM("Here-2 x: " << wx << " y: " << wy);
        size_t mx, my;
        if (master_grid.worldToMap(wx, wy, mx, my)) {

          for (int y = 0; y <= g_radius_ * 2; ++y) {
            for (int x = 0; x <= g_radius_ * 2; ++x) {
              int x_val = x - g_radius_;
              int y_val = y - g_radius_;
              // ROS_INFO_STREAM("LSizeX: " << layer_size_x << " LSizeY: " << layer_size_y);
              // ROS_INFO_STREAM("OffX: " << off_mx << " OffY: " << off_my <<
              //                 " mx: " << mx << " my: " << my <<
              //                 " x: " << x << " y: " << x <<
              //                 " xv: " << x_val << " yv: " << y_val <<
              //                 " IndX: " << (off_mx - (int)mx + x_val) <<
              //                 " IndY: " << (off_my - (int)my + y_val));
              // min_mx + (layer_size_x - lx) - g_radius_
              double c_cost = cost_layer[off_mx - (int)mx + x_val][off_my - (
                                                                     int)my + y_val];
              // ROS_INFO("c_cost");
              double a = this->gaussian(x_val, y_val, 0, 0,
                                        def_value_) / (((float)p / 3.0) + 1.0);
              // ROS_INFO("a");
              double new_cost = std::max(a, c_cost);
              // ROS_INFO("new_cost");
              cost_layer[off_mx - (int)mx + x_val][off_my - (int)my + y_val] =
                new_cost;
              // ROS_INFO("cost_layer");
            }
          }
        }
      }
    }
    // Planner interactive reward layer
    for (size_t p = 0; p < predict_msg.foresight; ++p) {
      double wx = predict_msg.planner_pose[p].x;
      double wy = predict_msg.planner_pose[p].y;
      size_t mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {

        for (int y = 0; y <= g_radius_ * 2; ++y) {
          for (int x = 0; x <= g_radius_ * 2; ++x) {
            int x_val = x - g_radius_;
            int y_val = y - g_radius_;
            double c_rew = reward_layer[off_mx - (mx + x_val)][off_my - (my + y_val)];

            double a = this->gaussian(x_val, y_val, 0, 0, def_value_);
            double new_reward = std::min(-a, c_rew);
            reward_layer[off_mx - (mx + x_val)][off_my - (my + y_val)] = new_reward;
          }
        }
      }
    }
    // Cost and Reward layer addition
    for (int ly = 0; ly < layer_size_y; ++ly) {
      for (int lx = 0; lx < layer_size_x; ++lx) {
        size_t mx = min_mx + (layer_size_x - lx) - g_radius_;
        size_t my = min_my + (layer_size_y - ly) - g_radius_;
        // double wx, wy;
        // master_grid.mapToWorld(mx, my, wx, wy);
        double f_cost = cost_layer[lx][ly] + reward_layer[lx][ly] + def_value_;
        // ROS_INFO_STREAM("wx: " << wx << " wy: " << wy <<
        //                 " CLayer: " << cost_layer[lx][ly] <<
        //                 " RLayer: " << reward_layer[lx][ly] <<
        //                 " FCost: " << f_cost);
        f_cost = std::max(min_cost_, f_cost);
        f_cost = std::min(max_cost_, f_cost);
        // ROS_INFO_STREAM("lx: " << lx << " ly: " << ly <<
        //                 " mx: " << mx << " my: " << my);
        master_grid.setCost(mx - 1, my - 1, (unsigned char) f_cost);
      }
    }
  }

  // // Agent interactive cost layer
  // for (size_t agent = 0; agent < predict_msg_.agent.size(); ++agent) {
  //   for (size_t p = 0; p < predict_msg_.foresight; ++p) {
  //     double wx = predict_msg_.agent[agent].pose[p].x;
  //     double wy = predict_msg_.agent[agent].pose[p].y;
  //     size_t mx, my;
  //     if (master_grid.worldToMap(wx, wy, mx, my)) {

  //       for (int x = -g_radius_; x < g_radius_; ++x) {
  //         for (int y = -g_radius_; y < g_radius_; ++y) {
  //           unsigned char old_cost = master_grid.getCost(x + mx, y + my);
  //           if (old_cost == costmap_2d::NO_INFORMATION) {continue;}

  //           double a = this->gaussian(x, y, 0, 0, def_value_);
  //           double new_cost;
  //           if ((double)old_cost <= def_value_) {
  //             new_cost = (double)old_cost + a;
  //           } else {
  //             new_cost = std::max(def_value_ + a, (double) old_cost);
  //             //new_cost = def_value_ + a;
  //           }

  //           master_grid.setCost(x + mx, y + my,
  //                               (unsigned char) std::min(new_cost, max_cost_));
  //         }
  //       }
  //     }
  //   }
  // }

  // // Planner interactive reward layer
  // // if (updated_) {
  // for (size_t p = 0; p < predict_msg_.foresight; ++p) {
  //   double wx = predict_msg_.planner_pose[p].x;
  //   double wy = predict_msg_.planner_pose[p].y;
  //   size_t mx, my;
  //   if (master_grid.worldToMap(wx, wy, mx, my)) {

  //     for (int x = -g_radius_; x < g_radius_; ++x) {
  //       for (int y = -g_radius_; y < g_radius_; ++y) {
  //         unsigned char old_cost = master_grid.getCost(x + mx, y + my);
  //         if (old_cost == costmap_2d::NO_INFORMATION) {continue;}

  //         double a = this->gaussian(x, y, 0, 0, def_value_);
  //         // double cvalue = def_value_ - a;
  //         // double new_cost = std::min(cvalue, (double) old_cost);
  //         double new_cost;
  //         if ((double)old_cost >= def_value_) {
  //           new_cost = (double)old_cost - a;
  //         } else {
  //           new_cost = std::min(def_value_ - a, (double) old_cost);
  //         }

  //         master_grid.setCost(x + mx, y + my,
  //                             (unsigned char) std::max(new_cost, min_cost_));
  //       }
  //     }
  //   }
  // }
  // }
  ROS_INFO("EndCostmap");
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
