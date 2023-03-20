#include "nav2_proxemic_costmap_plugin/proxemic_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>


using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_proxemic_costmap_plugin
{

ProxemicLayer::~ProxemicLayer() {}

void ProxemicLayer::onInitialize()
{
    auto node = node_.lock(); //node_ (weak_ptr), node (shared_ptr)

    auto qos_var = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);

    sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>("/poses_topic",qos_var,std::bind(&ProxemicLayer::peopleCallBack, this, std::placeholders::_1));

    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    declareParameter("sigx", rclcpp::ParameterValue(6.0));
    node->get_parameter(name_ + "." + "sigx", sigx_);

    declareParameter("sigy", rclcpp::ParameterValue(6.0));
    node->get_parameter(name_ + "." + "sigy", sigy_);

    need_recalculation_ = true;
    current_ = true;
    update_cost_ = true;

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

    RCLCPP_INFO(node->get_logger(),"resolution: %f", resolution_);

    i_max = 0;
    nuevo = false;
}

void ProxemicLayer::peopleCallBack(const geometry_msgs::msg::PoseArray msg){

    auto node = node_.lock();

    i_max = msg.poses.size();

    RCLCPP_INFO(node->get_logger(),"He recibido %d poses. (callback)",i_max);

    posesx.clear();
    posesy.clear();
    posesz.clear();

    for (int i = 0; i < i_max; i++){

        posesx.push_back(msg.poses[i].position.x);
        posesy.push_back(msg.poses[i].position.y);
        posesz.push_back(msg.poses[i].orientation.z);

        RCLCPP_INFO(node->get_logger(),"He recibido la pose: [x:%f, y:%f, o:%f]", posesx[i], posesy[i], posesz[i]);
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

    if(need_recalculation_){

        *min_x = -6.5;
        *min_y = -3.5;
        *max_x = 4;
        *max_y = 10;

        if(nuevo){
            need_recalculation_ = false;
            nuevo = false;
            update_cost_ = true;
        }

    }else{
        *min_x = -6.5;
        *min_y = -3.5;
        *max_x = 4;
        *max_y = 10;

        update_cost_ = false;
    }

}

void ProxemicLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{

    if (!enabled_) {return;}

    auto node = node_.lock();

    if(update_cost_){                                           // actualizo --> disminuyo coste del "rastro" y borro el "rastro" mas debil

        int size_x = getSizeInCellsX();
        int size_y = getSizeInCellsY();
        unsigned char * master_array = master_grid.getCharMap();

        for (int j = 0; j < size_y; j++) {
            for (int i = 0; i < size_x; i++) {
                unsigned int index = master_grid.getIndex(i,j);
                int cost = getCost(index);
                if(cost > 60){
                    setCost(i, j, cost - 30);
                    master_array[index] = cost - 30;  // si borro esta línea, solamente borro la lectura del laser y el inflado cuando estoy en Set Gaussian, pero va mas rápido

                }else{
                    setCost(i, j, nav2_costmap_2d::FREE_SPACE);
                    
                }

            }
        }
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        RCLCPP_INFO(node->get_logger(),"Actualizo - Borro");


    }else{                                                      // cuando no actualizo, dibujo la gaussiana en la ultima pose que recibi
        int tam = posesx.size();

        for (int k = 0; k < tam; k++){

            setGaussian(master_grid, posesx[k], posesy[k], posesz[k]);

        }

        if(i_max == 0){
            update_cost_ = true;
        }

        need_recalculation_ = false;
        RCLCPP_INFO(node->get_logger(),"Última pose");
    }
}

void ProxemicLayer::setGaussian(nav2_costmap_2d::Costmap2D & master_grid, double pose_x, double pose_y, double ori){
    //______________________________________________________________________________________
    // ----------------------------------COSTE--------------------------------------------
    //______________________________________________________________________________________
    
    auto node = node_.lock();

    int limit_min_i = 0;
    int limit_min_j = 0;
    int limit_max_i = 0;
    int limit_max_j = 0;
    int center_x = 0;
    int center_y = 0;
    int A = nav2_costmap_2d::LETHAL_OBSTACLE;
    double sigx = sigx_;
    double sigy = sigy_;
    double sigu = sigx_;
    double sigv = sigy_;
    //ori = ori*2*M_PI;

    worldToMapEnforceBounds(pose_x - 1.0, pose_y - 1.0, limit_min_i, limit_min_j);
    worldToMapEnforceBounds(pose_x + 1.0, pose_y + 1.0, limit_max_i, limit_max_j);
    worldToMapEnforceBounds(pose_x, pose_y, center_x, center_y);

    unsigned char * master_array = master_grid.getCharMap();


    for (int j = limit_min_j; j < limit_max_j+1; j++) {
        for (int i = limit_min_i; i < limit_max_i+1; i++) {
            
            unsigned int cost = round(A*exp(-((pow(((i-center_x)*cos(ori)+(j-center_y)*sin(ori)),2)/(2*pow(sigx,2)))+(pow((-(i-center_x)*sin(ori)+(j-center_y)*cos(ori)),2)/(2*pow(sigy,2))))));

            if(cost > 60){
                int index = master_grid.getIndex(i, j);
                setCost(i,j,cost);
                master_array[index] = cost;
            }
        }
    }

    int map_min_i = 0;
    int map_min_j = 0;
    int map_max_i = 0;
    int map_max_j = 0;

    worldToMapEnforceBounds(-6.5, -3.5, map_min_i, map_min_j);
    worldToMapEnforceBounds(4, 10, map_max_i, map_max_j);

    updateWithMax(master_grid, map_min_i, map_min_j, map_max_i, map_max_j);
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
