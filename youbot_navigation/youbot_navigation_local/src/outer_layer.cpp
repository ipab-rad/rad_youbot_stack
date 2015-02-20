#include <simple_map/outer_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(outer_layer::OuterLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace outer_layer
{

OuterLayer::OuterLayer() {}

void OuterLayer::onInitialize()
{
  enabled_ = false;
  MATRIX_WIDTH = 200;
  MATRIX_HEIGHT = 200;
  is_matrix_initialised = false;
  // initialising the vector
  // std::vector< std::vector<int> > data_matrix(MATRIX_HEIGHT, std::vector<int>(MATRIX_WIDTH));
  data_matrix.resize(MATRIX_HEIGHT, std::vector<int>(MATRIX_WIDTH, 0));
  ros::NodeHandle node;
  sub_ = node.subscribe("/outer_layer",
                            400,
                            &OuterLayer::receive_layer,
                            this);
  ROS_INFO("Subscribing to outer layer msg");

  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &OuterLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void OuterLayer::receive_layer(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  int count = 0;
  int r;
  int c;
  for(std::vector<int>::const_iterator m_it = msg->data.begin();
      m_it != msg->data.end(); ++m_it) {
    r = (count / MATRIX_WIDTH);
    c = (count % MATRIX_WIDTH);
    data_matrix[r][c] = *m_it; // stil an int
  }
  std::cout << "RANDOM POINT: "
            << data_matrix[rand() % 254][rand() % 254]
            << std::endl;
}

void OuterLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void OuterLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void OuterLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                             double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  /* double mark_x = origin_x + cos(origin_yaw), mark_y = origin_y + sin(origin_yaw); */
  /* unsigned int mx; */
  /* unsigned int my; */
  /* if(worldToMap(mark_x, mark_y, mx, my)){ */
  /*   setCost(mx, my, LETHAL_OBSTACLE); */
  /* } */
  /* *min_x = std::min(*min_x, mark_x); */
  /* *min_y = std::min(*min_y, mark_y); */
  /* *max_x = std::max(*max_x, mark_x); */
  /* *max_y = std::max(*max_y, mark_y); */
  std::cout << "OUTER: UPDATE  -> max:"
            << *max_x << ", "
            << *max_y << "; min: "
            << *min_x << ", "
            << *min_y << std::endl;
}

void OuterLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                            int max_j)
{

  /*
    if (!enabled_)
     return;
  */

  std::cout << "OUTER: COSTS -> max:"
            << max_i << ", "
            << max_j << "; min: "
            << min_i << ", "
            << min_j << std::endl;

  for (int r = 0; r < MATRIX_HEIGHT; r++)
  {
    for (int c = 0; c < MATRIX_WIDTH; c++)
    {
      // TODO: using WORLDTOMAP?
      // int index = getIndex(i, j);
      unsigned char value = data_matrix[r][c];
      if (value == NO_INFORMATION){
        // std::cout << "NO INFORMATION!" << std::endl;
        continue;
      }
      std::cout << "YES INFORMATION!" << std::endl;
      master_grid.setCost(r, c, value);
    }
  }
}

} // end namespace
