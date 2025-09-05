#ifndef RACE_GLOBAL_PLANNER_H
#define RACE_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace race_global_planner
{
    class RaceGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
        public:
            RaceGlobalPlanner(); // 必备构造函数
            ~RaceGlobalPlanner(); // 必备析构函数
            RaceGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros); // 避免直接调用初始化函数

            // 实现纯虚函数
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros); // 必备初始化函数
            bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        private:
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            bool initialized_;
            

    }; // class RaceGlobalPlanner

} // namespace race_global_planner

#endif // RACE_GLOBAL_PLANNER_H