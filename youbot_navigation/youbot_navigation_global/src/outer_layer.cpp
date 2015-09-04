#include <simple_map/outer_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(outer_layer::OuterLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace outer_layer {

OuterLayer::OuterLayer() {}

void OuterLayer::onInitialize() {
  MATRIX_WIDTH = 20;
  MATRIX_HEIGHT = 20;
  is_matrix_initialised_ = false;
  data_matrix.resize(MATRIX_HEIGHT, std::vector<int>(MATRIX_WIDTH, 0));
  ros::NodeHandle node;
  sub_ = node.subscribe("/outer_layer", 400, &OuterLayer::receive_layer, this);
  ROS_INFO("Subscribing to outer layer msg");

  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
    boost::bind(
      &OuterLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void OuterLayer::receive_layer(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  is_matrix_initialised_ = true;
  int count = 0;
  int r;
  int c;
  for (std::vector<int>::const_iterator m_it = msg->data.begin();
       m_it != msg->data.end(); ++m_it) {
    r = (count / MATRIX_WIDTH);
    c = (count % MATRIX_WIDTH);
    data_matrix[r][c] = *m_it; // stil an int
    count++;
  }
}

void OuterLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                               uint32_t level) {
  enabled_ = config.enabled;
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

  std::cout << "OUTER: UPDATE  -> max:"
            << *max_x << ", "
            << *max_y << "; min: "
            << *min_x << ", "
            << *min_y << std::endl;
}

void OuterLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                             int min_j, int max_i,
                             int max_j) {
  if (!is_matrix_initialised_) { return; }
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;

  for (int r = 0; r < MATRIX_HEIGHT; r++) {
    for (int c = 0; c < MATRIX_WIDTH; c++) {
      unsigned char value = data_matrix[r][c];
      if (value == 0) {
        continue;
      }
      if (master_grid.worldToMap(-c, -r, mx, my)) {
        ROS_INFO("wx: %d, wy: %d, mx: %u, my: %u", -c, -r, mx, my);
        master_grid.setCost((mx / 2.0) + 30, (my / 2.0) + 20, value);

      }
    }
  }
}

} // end namespace
