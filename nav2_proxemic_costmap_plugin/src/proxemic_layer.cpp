#include "nav2_proxemic_costmap_plugin/proxemic_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_proxemic_costmap_plugin
{

ProxemicLayer::~ProxemicLayer() {}

void ProxemicLayer::onInitialize()
{
    auto node = node_.lock(); //node_ (weak_ptr), node (shared_ptr)

    // PARAMETERS AND INTERNAL VARIABLES:

    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    declareParameter("sigx", rclcpp::ParameterValue(6.0));
    node->get_parameter(name_ + "." + "sigx", sigx_);

    declareParameter("sigy", rclcpp::ParameterValue(6.0));
    node->get_parameter(name_ + "." + "sigy", sigy_);

    declareParameter("debug_info", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "debug_info", debug_info_);

    declareParameter("poses_topic_name", rclcpp::ParameterValue("/poses_topic"));
    node->get_parameter(name_ + "." + "poses_topic_name", poses_topic_name_);

    need_recalculation_ = true;
    current_ = true;
    update_cost_ = true;

    i_max = 0;
    nuevo = false;

    auto qos_var = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default); // quality of service profile, for subscription

    sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(poses_topic_name_,qos_var,std::bind(&ProxemicLayer::peopleCallBack, this, std::placeholders::_1));

    // Copies characteristics of master layer into my layer: size, resolution, origin

    ProxemicLayer::matchSize();

    proxemic_costmap_ = std::make_shared<Costmap2D>(
    layered_costmap_->getCostmap()->getSizeInCellsX(),
    layered_costmap_->getCostmap()->getSizeInCellsY(),
    layered_costmap_->getCostmap()->getResolution(),
    layered_costmap_->getCostmap()->getOriginX(),
    layered_costmap_->getCostmap()->getOriginY());

    proxemic_costmap_->setDefaultValue(nav2_costmap_2d::FREE_SPACE);
}

void ProxemicLayer::peopleCallBack(const geometry_msgs::msg::PoseArray msg){

    auto node = node_.lock();

    i_max = msg.poses.size();

    if (debug_info_){
        RCLCPP_INFO(node->get_logger(),"%d poses received. (callback)",i_max);
    }
    
    posesx.clear();
    posesy.clear();
    posesz.clear();

    for (int i = 0; i < i_max; i++){

        posesx.push_back(msg.poses[i].position.x);
        posesy.push_back(msg.poses[i].position.y);

        // quaternion to euler
        tf2::Quaternion ori_q = tf2::Quaternion(msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w);
        auto ori_eu = ori_q.getAngle();
        if(msg.poses[i].orientation.z < 0){
            ori_eu = ori_eu*(-1);
        }

        posesz.push_back(ori_eu);

        if (debug_info_){
            RCLCPP_INFO(node->get_logger(),"The following pose was received: [x:%f, y:%f, o:%f]", posesx[i], posesy[i], posesz[i]);
        }
    }

    if(i_max > 0){
        nuevo = true;

    }else{
        nuevo = false;
    }

    need_recalculation_ = true;
    update_cost_ = true;
}

void ProxemicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y){

    auto node = node_.lock();

    // we get map's size in meters
    global_min_x = getOriginX();
    global_min_y = getOriginY();

    global_max_x = getSizeInCellsX()*getResolution()+global_min_x;
    global_max_y = getSizeInCellsY()*getResolution()+global_min_y;

    // we save the pose so we can use in gaussian function
    robot_x_ = robot_x;
    robot_y_ = robot_y;


    if(need_recalculation_){    // we set as recalculation bounds the entire map's size (so we are able to upload the map with other layers information)

        *min_x = global_min_x;
        *min_y =  global_min_y;
        *max_x = global_max_x;
        *max_y = global_max_y;

        if(nuevo){              
            need_recalculation_ = false;
            nuevo = false;
            update_cost_ = true;
        }

    }else{                      // we set as recalculation bounds only a certain area around the robot's pose
    
        *min_x = robot_x - 3.0;
        *min_y = robot_y - 3.0;
        *max_x = robot_x + 3.0;
        *max_y = robot_y + 3.0;

        if(*min_x < global_min_x){
            *min_x = global_min_x;
        }
        if(*min_y < global_min_y){
            *min_y = global_min_y;
        }
        if(*max_x > global_max_x){
            *max_x = global_max_x;
        }
        if(*max_y > global_max_y){
            *max_y = global_max_y;
        }

        update_cost_ = false;
    }

}

void ProxemicLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{

    if (!enabled_) {return;}

    auto node = node_.lock();
                                                                
    if(update_cost_){                                           // update --> decreases the cost of the "trail" and erases the weakest part

        int map_min_i = 0;
        int map_min_j = 0;
        int map_max_i = 0;
        int map_max_j = 0;

        double min_x = robot_x_ - 3.0;
        double min_y = robot_y_ - 3.0;
        double max_x = robot_x_ + 3.0;
        double max_y = robot_y_ + 3.0;

        if(min_x < global_min_x){
            min_x = global_min_x;
        }
        if(min_y < global_min_y){
            min_y = global_min_y;
        }
        if(max_x > global_max_x){
            max_x = global_max_x;
        }
        if(max_y > global_max_y){
            max_y = global_max_y;
        }

    worldToMapEnforceBounds(min_x, min_y, map_min_i, map_min_j);
    worldToMapEnforceBounds(max_x, max_y, map_max_i, map_max_j);

        unsigned char * master_array = master_grid.getCharMap();

        for (int j = map_min_j; j < map_max_j; j++) {
            for (int i = map_min_i; i < map_max_i; i++) {
                unsigned int index = master_grid.getIndex(i,j);
                int cost = getCost(index);
                if(cost > 60){
                    setCost(i, j, cost - 30);               // we update the cost in proxemic layer AND in master layer
                    master_array[index] = cost - 30;

                }else{
                    setCost(i, j, nav2_costmap_2d::FREE_SPACE);
                    
                }

            }
        }
        updateWithMax(master_grid, map_min_i, map_min_j, map_max_i, map_max_j);

        if (debug_info_){
            RCLCPP_INFO(node->get_logger(),"Updating proxemic layer");
        }


    }else{                                                      // when not updating, we set the gaussian in the last poses received
        int tam = posesx.size();

        for (int k = 0; k < tam; k++){

            setGaussian(master_grid, posesx[k], posesy[k], posesz[k]);  // calling the function for each pose

        }

        if(i_max == 0){
            update_cost_ = true;
        }

        need_recalculation_ = false;

        if (debug_info_){
            RCLCPP_INFO(node->get_logger(),"Setting gaussian of last poses received");
        }
    }
}

void ProxemicLayer::setGaussian(nav2_costmap_2d::Costmap2D & master_grid, double pose_x, double pose_y, double ori){
    
    // SETTING GAUSSIAN COST
    
    auto node = node_.lock();

    // update limits (6 meters radius)

    int map_min_i = 0;
    int map_min_j = 0;
    int map_max_i = 0;
    int map_max_j = 0;

    double min_x = robot_x_ - 3.0;
    double min_y = robot_y_ - 3.0;
    double max_x = robot_x_ + 3.0;
    double max_y = robot_y_ + 3.0;

    if(min_x < global_min_x){
        min_x = global_min_x;
    }
    if(min_y < global_min_y){
        min_y = global_min_y;
    }
    if(max_x > global_max_x){
        max_x = global_max_x;
    }
    if(max_y > global_max_y){
        max_y = global_max_y;
    }

    worldToMapEnforceBounds(min_x, min_y, map_min_i, map_min_j);
    worldToMapEnforceBounds(max_x, max_y, map_max_i, map_max_j);

    // gaussian limits
    int limit_min_i = 0;
    int limit_min_j = 0;
    int limit_max_i = 0;
    int limit_max_j = 0;
    int center_x = 0;
    int center_y = 0;
    int A = nav2_costmap_2d::LETHAL_OBSTACLE;
    double sigx = sigx_;
    double sigy = sigy_;

    // we go over every cell in the grid in a 2 meters "radius" around the received pose, to set the gaussian cost:

    worldToMapEnforceBounds(pose_x - 1.0, pose_y - 1.0, limit_min_i, limit_min_j);  // min limits -> from meters to cells
    worldToMapEnforceBounds(pose_x + 1.0, pose_y + 1.0, limit_max_i, limit_max_j);  // max limits -> from meters to cells
    worldToMapEnforceBounds(pose_x, pose_y, center_x, center_y); // pose coordinates -> from meters to cells

    unsigned char * master_array = master_grid.getCharMap();

    for (int j = limit_min_j; j < limit_max_j+1; j++) {
        for (int i = limit_min_i; i < limit_max_i+1; i++) {
            
            // function for rotated gaussian:
            unsigned int cost = round(A*exp(-((pow(((i-center_x)*cos(ori)+(j-center_y)*sin(ori)),2)/(2*pow(sigx,2)))+(pow((-(i-center_x)*sin(ori)+(j-center_y)*cos(ori)),2)/(2*pow(sigy,2))))));

            if(cost > 60 && (i>map_min_i) && (i<map_max_i) && (j>map_min_j) && (j<map_max_j)){ // if it is "significant", we set the cost (in proxemic layer AND in master)
                int index = master_grid.getIndex(i, j);
                setCost(i,j,cost);
                master_array[index] = cost;
            }
        }
    }

    updateWithMax(master_grid, map_min_i, map_min_j, map_max_i, map_max_j); // we update cost in the 6 meters radius
}

// these functions are needed because "LayeredCostmap" calls them
void ProxemicLayer::onFootprintChanged(){}

void ProxemicLayer::reset() {}

bool ProxemicLayer::isClearable() {return false;}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_proxemic_costmap_plugin::ProxemicLayer, nav2_costmap_2d::CostmapLayer)
PLUGINLIB_EXPORT_CLASS(nav2_proxemic_costmap_plugin::ProxemicLayer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(nav2_proxemic_costmap_plugin::ProxemicLayer, nav2_costmap_2d::Costmap2D)
