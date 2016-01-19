#include <rad_layer/agents_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agents_layer::AgentsLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace agents_layer {

AgentsLayer::AgentsLayer() {}

void AgentsLayer::onInitialize() {
  costmap_init_ = false;
  do_once_ = false;
  ros::NodeHandle node;
  ROS_INFO("Agent Costmap setup");
  agent_sub_ = node.subscribe("/people", 400, &AgentsLayer::peopleCB, this);

  p_radius_ = 2;
  i_radius_ = 1;
  ros::NodeHandle nh("~/" + name_);
  current_ = true; // DO NOT REMOVE!
  // pose_x_ = 0.0; // DO NOT REMOVE!
  // pose_y_ = 0.0; // DO NOT REMOVE!
  // updated_pose_ = false; // DO NOT REMOVE!
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
    boost::bind(
      &AgentsLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void AgentsLayer::peopleCB(const people_msgs::People::ConstPtr& msg) {
  costmap_init_ = true;
  people_msg_ = *msg;
}

void AgentsLayer::updateBounds(double origin_x, double origin_y,
                               double origin_yaw, double* min_x,
                               double* min_y, double* max_x,
                               double* max_y) {
  if (!enabled_) {return;}
  double MW = 200.0;
  double MH = 200.0;
  *min_x = std::min(*min_x, (double) - MW);
  *min_y = std::min(*min_y, (double) - MH);
  *max_x = std::max(*max_x, (double) MW);
  *max_y = std::max(*max_y, (double) MH);
}

void AgentsLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                              int min_i, int min_j,
                              int max_i, int max_j) {
  if (!costmap_init_) { return; }
  if (!enabled_) {return;}
  if (!do_once_) {res = master_grid.getResolution();}

  uint mx;
  uint my;

  // People costmap
  size_t n_people = people_msg_.people.size();
  if (n_people != 0) {
    for (size_t i = 0; i < people_msg_.people.size(); ++i) {
      double wx = -people_msg_.people[i].position.x;
      double wy = people_msg_.people[i].position.y;
      for (int i = -p_radius_; i <= p_radius_; ++i) {
        for (int j = -p_radius_; j <= p_radius_; ++j) {
          if ((abs(i) <= i_radius_) && (abs(j) <= i_radius_)) {continue;}
          if (master_grid.worldToMap(wx + (i * res), wy + (j * res), mx, my)) {
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
          }
        }
      }
    }
  }
}

void AgentsLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                                uint32_t level) {
  enabled_ = config.enabled;
}

} // end namespace
