#include "nav2_proxemic_costmap_plugin/proxemic_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_proxemic_costmap_plugin
{

ProxemicLayer::~ProxemicLayer() {}

void ProxemicLayer::onInitialize()
{
    auto node = node_.lock(); //node_ (weak_ptr), node (shared_ptr)

    sub_ = node->create_subscription<std::vector<geometry_msgs::msg::PoseStamped>>("/people_topic",10,std::bind(&ProxemicLayer::peopleCallBack, this, std::placeholders::_1));

    declareParameter("enabled", rclcpp::ParameterValue(true));    
    node->get_parameter(name_ + "." + "enabled", enabled_);

    need_recalculation_ = false;
    current_ = true;
    update_cost_ = false;

    global_frame_ = layered_costmap_->getGlobalFrameID();
    rolling_window_ = false;

    ProxemicLayer::matchSize();

    proxemic_costmap_ = std::make_shared<Costmap2D>(
    layered_costmap_->getCostmap()->getSizeInCellsX(),
    layered_costmap_->getCostmap()->getSizeInCellsY(),
    layered_costmap_->getCostmap()->getResolution(),
    layered_costmap_->getCostmap()->getOriginX(),
    layered_costmap_->getCostmap()->getOriginY());

    proxemic_costmap_->setDefaultValue(nav2_costmap_2d::FREE_SPACE);
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

void ProxemicLayer::peopleCallBack(const std::vector<geometry_msgs::msg::PoseStamped> msg){

    auto node = node_.lock();

    int i_max = msg.size();
    for (int i = 0; i < (i_max+1); i++){
        poses_[i] = msg[i];
        RCLCPP_INFO(node->get_logger(),"He recibido la pose: [x:%f, y:%f, o:%f]", poses_[i].pose.position.x, poses_[i].pose.position.y, poses_[i].pose.orientation.z);
        i++;
    }

    need_recalculation_ = true;
    update_cost_ = false;
}

void ProxemicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y){ 
    
    auto node = node_.lock();

    if(need_recalculation_){
        std::vector<float> posesx;
        std::vector<float> posesy;

        *max_x = 0.5;
        *max_y = 0.5;
        *min_x = - 0.5;
        *min_y = - 0.5;

        int tam = poses_.size();
        for (int i = 0; i < tam; i++){
            posesx[i] = poses_[i].pose.position.x;
            posesy[i] = poses_[i].pose.position.y;

            if(posesx[i] > *max_x){
                *max_x = posesx[i] + 1;
            }
            if(posesx[i] < *min_x){
                *min_x = posesx[i] - 1;
            }
            if(posesy[i] > *max_y){
                *max_y = posesy[i] + 1;
            }
            if(posesy[i] < *min_y){
                *min_y = posesy[i] - 1;
            }
        }

        posesx.clear();
        posesy.clear();

        need_recalculation_ = false;
        update_cost_ = true;
    }
    
}

void ProxemicLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
    
    if (!enabled_) {return;}
    
    auto node = node_.lock();

    if(update_cost_){

        std::vector<float> posesx;
        std::vector<float> posesy;

        std::vector<int> mapx;
        std::vector<int> mapy;

        int tam = poses_.size();

        for (int k = 0; k < tam; k++){
            posesx[k] = poses_[k].pose.position.x;
            posesy[k] = poses_[k].pose.position.y;

            worldToMapEnforceBounds(posesx[k], posesy[k], mapx[k], mapy[k]);

            max_i = mapx[k] + 5;
            max_j = mapy[k] + 5;
            min_i = mapx[k] - 5;
            min_j = mapy[k] - 5;

            for (int j = min_j; j < max_j; j++) {
                for (int i = min_i; i < max_i; i++) {
                    unsigned char cost = LETHAL_OBSTACLE;
                    setCost(i, j, cost);
                }
            }
        }

        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        update_cost_ = false;
        poses_.clear();
    }
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
