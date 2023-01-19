#ifndef PROXEMIC_LAYER_HPP_
#define PROXEMIC_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nav2_proxemic_costmap_plugin
{

class ProxemicLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ProxemicLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~ProxemicLayer();

  virtual void onInitialize();

  virtual void getFrameNames();

  bool getAgentTFs(std::vector<tf2::Transform> & agents) const;

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y);

  void peopleCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void onFootprintChanged();

  virtual void reset();

  virtual bool isClearable();

private:

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  bool update_cost_;

  geometry_msgs::msg::PoseStamped pose_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  bool rolling_window_;

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2D> proxemic_costmap_;
  //std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_{nullptr};

};

}  // namespace nav2_proxemic_costmap_plugin

#endif  // PROXEMIC_LAYER_HPP_