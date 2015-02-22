#include <rad_layer/AgentsLayer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rad_layer::AgentsLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace rad_layer
{

AgentsLayer::AgentsLayer() {}

void AgentsLayer::onInitialize()
{
  DEBUG = 1;
  MATRIX_WIDTH = 200;
  MATRIX_HEIGHT = 200;
  pose_x_ = 0.0;
  pose_y_ = 0.0;
  updated_pose_ = false;
  is_matrix_initialised_ = false;
  agent_data.resize(MATRIX_HEIGHT, std::pair<float, float>(0.0, 0.0));
  ros::NodeHandle node;
  sub_ = node.subscribe("/agents_layer",
                            400,
                            &AgentsLayer::receive_layer,
                            this);
  ROS_INFO("AGENTS_LAYER: Subscribing to agents layer");

  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb
      = boost::bind(&AgentsLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void AgentsLayer::receive_layer(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (!updated_pose_)
    return;

  std::pair<float, float> p;
  for (std::vector<float>::const_iterator it = msg->data.begin();
       it != msg->data.end(); ++it) {
    // get x
    p.first = pose_x_ + *it;
    ++it;
    // get y
    p.second = pose_y_ + *it;
    agent_data.push_back(p);
  }
  is_matrix_initialised_ = true;
  new_data_ = true;
  if (DEBUG) {
    std::cout << "AGENTS_LAYER: MSG RECEIVED!";
  }
}

void AgentsLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config,
                                uint32_t level)
{
  enabled_ = config.enabled;
}

void AgentsLayer::updateBounds(double origin_x, double origin_y,
                               double origin_yaw, double* min_x,
                               double* min_y, double* max_x,
                               double* max_y)
{
  if (!enabled_)
    return;

  pose_x_ = origin_x;
  pose_y_ = origin_y;
  updated_pose_ = true;
  double MW = 200.0;
  double MH = 200.0;
  *min_x = std::min(*min_x, (double) -MW);
  *min_y = std::min(*min_y, (double) -MH);
  *max_x = std::max(*max_x, (double) MW);
  *max_y = std::max(*max_y, (double) MH);
  if (DEBUG) {
    std::cout << "AGENTS: UPDATE  -> max:"
              << *max_x << ", "
              << *max_y << "; min: "
              << *min_x << ", "
              << *min_y << std::endl;
  }
}

void AgentsLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                              int min_i, int min_j,
                              int max_i, int max_j)
{
  if (!is_matrix_initialised_) return;
  if (!new_data_) return;
  if (!enabled_) return;
  unsigned int mx, my;
  for (std::vector< std::pair<float, float> >::const_iterator it = agent_data.begin();
       it != agent_data.end(); ++it) {
    agent_x_ = (double) it->first - 0.3;
    agent_y_ = (double) it->second - 0.3;
    // make a 60X60cm box
    for (int i = 0; i < 3; i++) {
      agent_x_ += 0.1;
      for (int k = 0; k < 3; k++) {
        agent_y_ += 0.1;
        if(master_grid.worldToMap(agent_x_, agent_y_, mx, my)) {
          master_grid.setCost(mx, my, LETHAL_OBSTACLE);
          if (DEBUG) {
            std::cout << "AGENTS LAYER - Update: " << std::endl << "   "
                      << mx << ", "
                      << my << "; " << std::endl << "   "
                      << agent_x_ << ", "
                      << agent_y_ << std::endl;
          }
        }
      }
    }
  }
  new_data_ = false;
}
} // end namespace
