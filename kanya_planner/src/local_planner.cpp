#include "local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initiaized_(false) {}

LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL). initialized(false)
{
    initialize(name, tf, costmap_ros);
}

LocalPlanner::~LocalPlanner(){}

void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)

                              

}