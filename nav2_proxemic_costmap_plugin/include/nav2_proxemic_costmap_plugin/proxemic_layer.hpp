#ifndef PROXEMIC_LAYER_HPP_
#define PROXEMIC_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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

  void peopleCallBack(const geometry_msgs::msg::PoseArray msg);

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y);

  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void setGaussian(nav2_costmap_2d::Costmap2D & master_grid, double pose_x, double pose_y, double ori);


  virtual void onFootprintChanged();
  virtual void reset();
  virtual bool isClearable();

private:

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  
  std::vector<float> posesx, posesy, posesz;

  double global_max_x, global_max_y, global_min_x, global_min_y; // map's size in meters
  int i_max; // number of received poses
  bool nuevo; // any new pose received ?
  bool first_time;
  double sigx_, sigy_; // sigmas for gaussian cost
  double robot_x_, robot_y_;

  bool debug_info_;

  bool need_recalculation_; // chooses proceidure of updateBounds function
  bool update_cost_; // chooses proceidure of updateCosts function
  std::string poses_topic_name_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> proxemic_costmap_;
};

}  // namespace nav2_proxemic_costmap_plugin

#endif  // PROXEMIC_LAYER_HPP_