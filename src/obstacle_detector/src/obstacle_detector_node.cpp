#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h> 
#include <obstacle_detector/ObstacleStatus.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <deque>
#include <cmath> 
// #include "complete_mission.h" // 如果不需要其中的定义，建议注释掉避免编译冲突

struct DetectionZone {
    double x, y;          
    double half_size_x;   
    double half_size_y;   
    int current_count = 0;
    long total_window_count = 0;
    std::deque<std::pair<ros::Time, int>> history;
};

class ObstacleDetector {
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber sub_pcl_;
    ros::Subscriber local_pos_sub_; 
    
    ros::Publisher pub_status_;
    
    std::vector<DetectionZone> zones_;
    double h_min_, h_max_;    
    double window_sec_;       
    double hs_ ;

    // === 新增：位置控制相关成员 ===
    bool is_at_detection_point_ = false;
    double target_x_, target_y_, target_z_; // 目标检测点的全局坐标
    double position_threshold_;             // 到达判定阈值(米)

public:
    ObstacleDetector() : private_nh_("~") {
        zones_.resize(4);

        // === 1. 从参数服务器读取坐标 ===
        nh_.param("obs_1_x", zones_[0].x, 0.0);  nh_.param("obs_1_y", zones_[0].y, 0.0);
        nh_.param("obs_2_x", zones_[1].x, 0.0);  nh_.param("obs_2_y", zones_[1].y, 0.0);
        nh_.param("obs_3_x", zones_[2].x, 0.0);  nh_.param("obs_3_y", zones_[2].y, 0.0);
        nh_.param("obs_4_x", zones_[3].x, 0.0);  nh_.param("obs_4_y", zones_[3].y, 0.0);
        nh_.param("half_size",hs_, 0.0);

        for(auto& z : zones_) { z.half_size_x = hs_; z.half_size_y = hs_; }

        nh_.param("z_min", h_min_, 0.0); 
        nh_.param("z_max", h_max_, 0.0);
        nh_.param("window_duration", window_sec_, 0.0);

        // === 新增：读取检测点位置和阈值参数 ===
        nh_.param("target_x", target_x_, 0.0);
        nh_.param("target_y", target_y_, 0.0);
        nh_.param("target_z", target_z_, 0.8);
        nh_.param("position_threshold", position_threshold_, 0.3); // 默认30cm内算到达

        // === 2. 订阅发布 ===
        sub_pcl_ = nh_.subscribe("/cloud_registered", 1, &ObstacleDetector::cloudCallback, this);
        
        local_pos_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/mavros/local_position/odom", 10, &ObstacleDetector::localPosCallback, this);
            
        pub_status_ = nh_.advertise<obstacle_detector::ObstacleStatus>("/obstacle/result", 5);

        ROS_INFO("Detector Init | Target: (%.2f, %.2f, %.2f) | Threshold: %.2fm",
                 target_x_, target_y_, target_z_, position_threshold_);
    }

    // === 新增：位置回调函数 ===
    void localPosCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double cur_x = msg->pose.pose.position.x;
        double cur_y = msg->pose.pose.position.y;
        double cur_z = msg->pose.pose.position.z;

        // 计算三维欧氏距离
        double dist = std::sqrt(
            std::pow(cur_x - target_x_, 2) + 
            std::pow(cur_y - target_y_, 2) + 
            std::pow(cur_z - target_z_, 2)
        );

        // 更新开关状态
        bool new_state = (dist <= position_threshold_);
        if (new_state != is_at_detection_point_) {
            is_at_detection_point_ = new_state;
            ROS_WARN_THROTTLE(2.0, "[POS CHECK] %s | Dist: %.3fm", 
                              is_at_detection_point_ ? "ARRIVED - Detection ON" : "NOT ARRIVED - Detection OFF", 
                              dist);
        }
    }

    // === 原有雷达回调：增加开关判断 ===
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // 不在检测点时直接返回，不消耗CPU也不发布消息
        if (!is_at_detection_point_) return; 

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        ros::Time now = ros::Time::now();
        // === 3. 统计当前帧各区域点数 ===
        for (auto& z : zones_) z.current_count = 0;
        
        for (const auto& pt : cloud->points) {
            if (pt.z < h_min_ || pt.z > h_max_) continue;
            
            for (size_t i = 0; i < zones_.size(); ++i) {
                auto& z = zones_[i];
                if (std::abs(pt.x - z.x) <= z.half_size_x && 
                    std::abs(pt.y - z.y) <= z.half_size_y) {
                    z.current_count++;
                }
            }
        }

        // === 4. 更新3秒滑动窗口 ===
        for (auto& z : zones_) {
            z.history.push_back({now, z.current_count});
            while (!z.history.empty() && (now - z.history.front().first).toSec() > window_sec_) {
                z.history.pop_front();
            }
            z.total_window_count = 0;
            for (const auto& rec : z.history) z.total_window_count += rec.second;
        }

        // === 5. 竞争判断逻辑 ===
        obstacle_detector::ObstacleStatus res;
        res.header = msg->header;
        res.header.stamp = now;

        if (zones_[0].total_window_count == 0 && zones_[1].total_window_count == 0) {
            res.obs_1 = 0; res.obs_2 = 0;
        } else {
            res.obs_1 = (zones_[0].total_window_count >= zones_[1].total_window_count) ? 1 : 0;
            res.obs_2 = 1 - res.obs_1;
        }

        if (zones_[2].total_window_count == 0 && zones_[3].total_window_count == 0) {
            res.obs_3 = 0; res.obs_4 = 0;
        } else {
            res.obs_3 = (zones_[2].total_window_count >= zones_[3].total_window_count) ? 1 : 0;
            res.obs_4 = 1 - res.obs_3;
        }

        res.obstacle_detected = (res.obs_1 || res.obs_2 || res.obs_3 || res.obs_4);
        
        if(res.obs_1 && res.obs_3) res.current_case = "CASE_A(1&3)";
        else if(res.obs_1 && res.obs_4) res.current_case = "CASE_B(1&4)";
        else if(res.obs_2 && res.obs_3) res.current_case = "CASE_C(2&3)";
        else if(res.obs_2 && res.obs_4) res.current_case = "CASE_D(2&4)";
        else res.current_case = "NONE_OR_ERROR";

        pub_status_.publish(res);
        
        ROS_INFO_THROTTLE(1.0, "[%s] Counts: B1:%ld B2:%ld B3:%ld B4:%ld", 
                          res.current_case.c_str(),
                          zones_[0].total_window_count, zones_[1].total_window_count,
                          zones_[2].total_window_count, zones_[3].total_window_count);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detector_node");
    ObstacleDetector detector;
    ros::spin();
    return 0;
}