// 引入头文件
#include "race_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h> // tf相关的头文件
#include <tf/transform_listener.h> // 自动监听系统中的tf话题，可以调用这里提供的函数完成坐标系转换
#include <tf/transform_datatypes.h> // 提供一些坐标系转换的函数
#include <opencv2/highgui/highgui.hpp> //显示图片窗口
#include <opencv2/imgproc/imgproc.hpp> //绘制图片
#include "curvature.h"

// 注册插件
PLUGINLIB_EXPORT_CLASS(race_global_planner::RaceGlobalPlanner, nav_core::BaseGlobalPlanner) // 注册我们的类为一个插件，前面是类的完整名字，后面是它继承的接口

namespace race_global_planner
{
    RaceGlobalPlanner::RaceGlobalPlanner() :  initialized_(false)
    {
        setlocale(LC_ALL,""); // 将编码本地化，这样才能正确显示中文
        ROS_INFO("RaceGlobalPlanner插件闪亮登场!");
    }
    RaceGlobalPlanner::~RaceGlobalPlanner()
    {
        ROS_INFO("RaceGlobalPlanner插件退场了!");
    }

    tf::TransformListener* tf_listener_; // 在初始化函数的外边定义一下监听器的指针


    RaceGlobalPlanner::RaceGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // TODO
    }
    void RaceGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            tf_listener_ = new tf::TransformListener(); // 在初始化函数里生成这个监听器的对象，因为可能在多个函数里调用，所以定义成一个全局变量。
            costmap_ros_ = costmap_ros;
            // 初始化操作
            costmap_ = costmap_ros->getCostmap(); // 获取代价地图
            // 初始化后就标记为已初始化
            initialized_ = true;
            ROS_INFO("RaceGlobalPlanner插件初始化成功!");
        }
        else
        {
            ROS_WARN("RaceGlobalPlanner插件已经初始化过了,不要重复初始化哒嘿嘿~");
        }
    }
    // start是当前位置，goal是move_base传进来的目标位置，plan是要填充的路径点
    bool RaceGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if (!initialized_)
        {
            // 未初始化时直接报错返回
            ROS_ERROR("RaceGlobalPlanner插件还没有初始化，请先初始化!");
            return false;
        }
        // plan里面就是要走过的路径点
        plan.clear(); // 清空之前的路径点
        // 这里简单起见，直接把起点和终点放进去
        // 实际应用中，这里要根据地图和起点终点计算出一条路径
        plan.push_back(start); // 起点
        plan.push_back(goal); // 终点
        return true;
    } 
} // namespace race_global_planner
