#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

namespace local_plannner{
    class LocalPlanner : public nav_core::BaseLocalPlanner{
        public:
            LocalPlannner();
            LocalPlanner(std::string name, tf2_ros::Buffer* tf,
             costmap_2d::Costmap2DROS* costmap_ros);
            
            ~LocalPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf,  
             costmap_2d::Costmap2DROS* costmap_ros);

            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool isGoalReached();

        private:
            costmap_2d::Costmap2Dros* costmap_ros_;
            tf2_ros::Buffer* tf_;
            bool initaialized_;

  
    };
};

#endif