#include "nav2_proxemic_costmap_plugin/proxemic_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_proxemic_costmap_plugin
{

ProxemicLayer::ProxemicLayer(): last_min_x_(-std::numeric_limits<float>::max()),last_min_y_(-std::numeric_limits<float>::max()),last_max_x_(std::numeric_limits<float>::max()),last_max_y_(std::numeric_limits<float>::max()){
}

void ProxemicLayer::onInitialize()
{
    auto node = node_.lock(); //node_ (weak_ptr), node (shared_ptr)

    RCLCPP_INFO(node->get_logger(),"adios\n");

    sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("/people_topic",1,std::bind(&ProxemicLayer::peopleCallBack, this, std::placeholders::_1));

    declareParameter("enabled", rclcpp::ParameterValue(true));    
    node->get_parameter(name_ + "." + "enabled", enabled_);

    need_recalculation_ = false;
    pose_.pose.position.x = 0.0;
    pose_.pose.position.y = 0.0;
    pose_.pose.position.z = 0.0;
    pose_.pose.orientation.x = 0.0;
    pose_.pose.orientation.y = 0.0;
    pose_.pose.orientation.z = 0.0;
    pose_.pose.orientation.w = 0.0;
    pose_.header.frame_id = "map";
    pose_.header.stamp.sec = 0;
    pose_.header.stamp.nanosec = 0;

}

void ProxemicLayer::getFrameNames()
{
    // auto frames = tf_buffer_->getAllFrameNames();
    // for (auto tf : frames) {    
    //     if (tf.find(tf_prefix_) != std::string::npos) {
    //     agent_ids_.push_back(tf);
    //     }
    // }
    // sort(agent_ids_.begin(), agent_ids_.end());
    // agent_ids_.erase(unique(agent_ids_.begin(), agent_ids_.end()), agent_ids_.end());
}

bool ProxemicLayer::getAgentTFs(std::vector<tf2::Transform> & agents) const
{
    // geometry_msgs::msg::TransformStamped global2agent;
    // auto node = node_.lock(); //node_ (weak_ptr), node (shared_ptr)
    // for (auto id : agent_ids_) {
    //     try {
    //     // Check if the transform is available
    //     global2agent = tf_buffer_->lookupTransform(global_frame_, id, tf2::TimePointZero);
    //     } catch (tf2::TransformException & e) {
    //     RCLCPP_WARN(node->get_logger(), "%s", e.what());
    //     return false;
    //     }
    //     tf2::Transform global2agent_tf2;
    //     tf2::impl::Converter<true, false>::convert(global2agent.transform, global2agent_tf2);
    //     agents.push_back(global2agent_tf2);
    // }
    return true;
}

void ProxemicLayer::peopleCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        
    pose_.pose.position.x = msg->pose.position.x;
    pose_.pose.position.y = msg->pose.position.y;
    pose_.pose.orientation.z = msg->pose.orientation.z;

    auto node = node_.lock();
    RCLCPP_INFO(node->get_logger(),"He recibido la pose: [x:%f, y:%f, o:%f]", pose_.pose.position.x, pose_.pose.position.y, pose_.pose.orientation.z);

    need_recalculation_ = true;
}

void ProxemicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y){ 
    
    // ejemplo sencillo de bounds
    if (need_recalculation_ && !enabled_){
        *min_x = (pose_.pose.position.x)-0.1;
        *min_y = (pose_.pose.position.y)-0.1;
        *max_x = (pose_.pose.position.x)+0.1;
        *max_y = (pose_.pose.position.y)+0.1;
        // *min_x = std::min(*min_x, pose_.pose.position.x - 0.1);
        // *min_y = std::min(*min_y, pose_.pose.position.y - 0.1);
        // *max_x = std::max(*max_x, pose_.pose.position.x + 0.1);
        // *max_y = std::max(*max_y, pose_.pose.position.y + 0.1);
        need_recalculation_ = false;
    }

}

void ProxemicLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
    
    if (!enabled_) {return;}

    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!

    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():

    //DESCOMENTAR:
    if (need_recalculation_){
        unsigned char * master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX();
        unsigned int size_y = master_grid.getSizeInCellsY();

        // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
        // These variables are used to update the costmap only within this window
        // avoiding the updates of whole area.
        //
        // Fixing window coordinates with map size if necessary.

        //min_i = std::max(0, min_i);
        //min_j = std::max(0, min_j);
        //DESCOMENTAR:
        
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);

        auto node = node_.lock();
        RCLCPP_INFO(node->get_logger(),"Bounds after: [max_x:%d, min_x:%d, max_y:%d, min_y:%d]", max_i, min_i, max_j, min_j);

        // Simply computing one-by-one cost per each cell
        for (int j = min_j; j < max_j; j++) {
            for (int i = min_i; i < max_i; i++) {
                int index = master_grid.getIndex(i, j);
                // setting the cost
                unsigned char cost = LETHAL_OBSTACLE;
                master_array[index] = cost;
            }
        }

        //need_recalculation_ = false;
    }

    // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);

}

void ProxemicLayer::matchSize(){

}

void ProxemicLayer::onFootprintChanged(){
//     need_recalculation_ = true;
//     RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "ProxemicLayer::onFootprintChanged(): num footprint points: %lu",layered_costmap_->getFootprint().size());
}

void ProxemicLayer::reset() {}

bool ProxemicLayer::isClearable() {return false;}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_proxemic_costmap_plugin::ProxemicLayer, nav2_costmap_2d::CostmapLayer)
PLUGINLIB_EXPORT_CLASS(nav2_proxemic_costmap_plugin::ProxemicLayer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(nav2_proxemic_costmap_plugin::ProxemicLayer, nav2_costmap_2d::Costmap2D)
