#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <string>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <obstacle_detector/ObstacleStatus.h>
#include <yolov8_ros_msgs/YoloDetection.h>
// #include "quadrotor_msgs/PositionCommand.h"
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <iostream>
 
using namespace std;

#define ALTITUDE 0.9

mavros_msgs::PositionTarget setpoint_raw;
ros::Publisher planner_goal_pub;
ros::Publisher finish_ego_pub;
std_msgs::Bool finish_ego_flag;

float map_size_z = 0.95;
float ground_height = 0.7;

/************************************************************************
函数功能1：无人机状态回调函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
mavros_msgs::State current_state;   
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void state_cb(const mavros_msgs::State::ConstPtr& msg)  
{
    current_state = *msg;
}



/************************************************************************
函数功能2：回调函数接收无人机的里程计信息
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
tf::Quaternion quat; 
nav_msgs::Odometry local_pos; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
float init_yaw_take_off        =0;
bool  flag_init_position = false;
void  local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void  local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
	    init_yaw_take_off        = yaw;
        flag_init_position = true;		    
    } 
}

/************************************************************************
函数功能3：自主巡航，发布目标位置，控制无人机到达目标，采用local坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool  mission_pos_cruise_flag = false;
bool  mission_pos_cruise(float x, float y, float z, float yaw, float error_max);
bool  mission_pos_cruise(float x, float y, float z, float yaw, float error_max)
{
	if(mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = init_position_x_take_off + x;
	setpoint_raw.position.y = init_position_y_take_off + y;
	setpoint_raw.position.z = init_position_z_take_off + z;
	setpoint_raw.yaw        = yaw;             
	if(fabs(local_pos.pose.pose.position.x-x-init_position_x_take_off)<error_max 
	&& fabs(local_pos.pose.pose.position.y-y-init_position_y_take_off)<error_max
	&& fabs(local_pos.pose.pose.position.z-z-init_position_z_take_off)<error_max)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}


/************************************************************************
函数功能4：自主巡航，发布目标位置，控制无人机到达目标，采用机体坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
float current_position_cruise_last_position_yaw = 0;
bool  current_position_cruise_flag = false;
bool  current_position_cruise(float x, float y, float z, float yaw, float error_max);
bool  current_position_cruise(float x, float y, float z, float yaw, float error_max)
{
	if(current_position_cruise_flag == false)
	{
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		current_position_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = current_position_cruise_last_position_x + x;
	setpoint_raw.position.y = current_position_cruise_last_position_y + y;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw        = yaw;
	double current_yaw,a,b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	if(fabs(local_pos.pose.pose.position.x-current_position_cruise_last_position_x-x)<error_max 
	&& fabs(local_pos.pose.pose.position.y-current_position_cruise_last_position_y-y)<error_max
	&& fabs(current_yaw - yaw )< 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		current_position_cruise_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数功能5: 获取yolo识别目标的位置信息
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
geometry_msgs::PointStamped object_pos; 
double position_detec_x = 0;
double position_detec_y = 0;
double position_detec_z = 0;
void   object_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg);
void   object_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	object_pos = *msg;
}



/************************************************************************
函数功能6: 目标识别，采用速度控制运动，任务是识别到目标后，保持无人机正对着目标
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float object_recognize_track_vel_last_time_position_x = 0;
float object_recognize_track_vel_last_time_position_y = 0;
bool  object_recognize_track_vel_flag = false;
bool  object_recognize_track_vel(string str, float yaw, float altitude, float speed, float error_max);
bool  object_recognize_track_vel(string str, float yaw, float altitude, float speed, float error_max)
{
	//此处false主要是为了获取任务初始时候的位置信息
	if(object_recognize_track_vel_flag == false)
	{
		//获取初始位置，防止无人机飘
		object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
    	object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_vel_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	//此处首先判断是否识别到指定的目标
	if(object_pos.header.frame_id == "ring" || object_pos.header.frame_id == "square")
    {
		//此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
        //获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 
		if(fabs(position_detec_x-320) < error_max && fabs(position_detec_y-240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
		else
		{
			//摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
			if(position_detec_x -320 >= error_max)
			{
				setpoint_raw.velocity.y =  -speed;
			}		
			else if(position_detec_x - 320 <= -error_max)
			{
				setpoint_raw.velocity.y =  speed;
			}	
			else
			{
				setpoint_raw.velocity.y =  0;
			}					  
			//无人机前后移动速度控制
			if(position_detec_y -240 >= error_max)
			{
				setpoint_raw.velocity.x =  -speed;
			}
			else if(position_detec_y - 240 <= -error_max)
			{
				setpoint_raw.velocity.x =  speed;
			}
			else
			{
				setpoint_raw.velocity.x =  0;
			}
			//机体坐标系下发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
        	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 8;
		    setpoint_raw.position.z = init_position_z_take_off+altitude;
		}
	}
	else
	{	
		setpoint_raw.position.x = object_recognize_track_vel_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_vel_last_time_position_y;
		setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.z = init_position_z_take_off + altitude;
		setpoint_raw.yaw        = yaw;
	}
	return false;
}


/************************************************************************
函数功能7: //无人机追踪，仅水平即Y方向，通常用于无人机穿越圆框、方框等使用
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float object_recognize_track_last_time_position_x = 0;
float object_recognize_track_last_time_position_y = 0;
bool  object_recognize_track_flag = false;
bool  object_recognize_track(string str, float yaw, int reverse, float altitude, float error_max,float ctrl_coef);
bool  object_recognize_track(string str, float yaw, int reverse, float altitude, float error_max,float ctrl_coef)
{
	if(object_recognize_track_flag == false)
	{
		object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
    	object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	//此处首先判断是否识别到指定的目标
	if(object_pos.header.frame_id == str)
    {
		//此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
        //获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 

		//摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
		if(position_detec_x -320 >= error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef*reverse;
		}					
		else if(position_detec_x - 320 <= -error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef*reverse;
		}
		else
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y;
		}
	
		if(fabs(position_detec_x-320) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
	}
	else
	{	
		setpoint_raw.position.x = object_recognize_track_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_last_time_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = 0 + altitude;
	setpoint_raw.yaw        = yaw;
	return false;
}


/************************************************************************
函数功能8: //无人机追踪，仅水平即Y方向，通常用于无人机穿越圆框、方框等使用
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float object_recognize_track_omni_last_time_position_x = 0;
float object_recognize_track_omni_last_time_position_y = 0;
bool  object_recognize_track_omni_flag = false;
bool  object_recognize_track_omni(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef);
bool  object_recognize_track_omni(string str, float yaw, int reverse, float altitude,float error_max, float ctrl_coef)
{
	if(object_recognize_track_omni_flag == false)
	{
		object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
    	object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_omni_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	//此处首先判断是否识别到指定的目标
	if(object_pos.header.frame_id == str)
    {
        object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
        //获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 

		//摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
		if(position_detec_x -320 >= error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef*reverse;
		}					
		else if(position_detec_x - 320 <= -error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef*reverse;
		}
		else
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y;
		}
		if(position_detec_y -240 >= error_max)
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x - ctrl_coef*reverse;
		}					
		else if(position_detec_y - 240 <= -error_max)
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x + ctrl_coef*reverse;
		}
		else
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x;
		}	

		if(fabs(position_detec_x-320) < error_max && fabs(position_detec_y-240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
	}
	else
	{	
		setpoint_raw.position.x = object_recognize_track_omni_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_omni_last_time_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = 0 + altitude;
	setpoint_raw.yaw        = yaw;
	return false;
}


/************************************************************************
函数功能9: 用于接收二维码识别返回值
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
ar_track_alvar_msgs::AlvarMarker marker;
float marker_x = 0, marker_y = 0, marker_z = 0;
bool  marker_found = false;
int   ar_track_id_current = 0;
void  ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
void  ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	int count = msg->markers.size();
	if(count!=0)
	{
		marker_found = true;
		for(int i = 0; i<count; i++)
		{
			marker = msg->markers[i];
			ar_track_id_current = marker.id;			
		}	
	}
	else
	{
		marker_found = false;
	}
}

/************************************************************************
函数功能10: 二维码降落,此处代码尚未实际验证，待验证
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float ar_lable_land_last_position_x = 0;
float ar_lable_land_last_position_y = 0;
bool  ar_lable_land_init_position_flag = false;
bool  ar_lable_land(float marker_error_max, int ar_track_id, float altitude);
bool  ar_lable_land(float marker_error_max, int ar_track_id, float altitude)
{
	if(ar_lable_land_init_position_flag == false)
	{
		ar_lable_land_last_position_x = local_pos.pose.pose.position.x;
    	ar_lable_land_last_position_y = local_pos.pose.pose.position.y;
		ar_lable_land_init_position_flag = true;
		ROS_INFO("进入二维码识别和降落");
	}
	//识别到指定的id才进入这个循环，否则不控制
	if(marker.id == ar_track_id && (marker.id!=0))
    {
    	//此处根本摄像头安装方向，进行静态坐标转换
        marker_x = marker.pose.pose.position.x;
        marker_y = marker.pose.pose.position.y;
        marker_z = marker.pose.pose.position.z; 
        /**************************************
        printf("marker_x = %f\r\n",marker_x);
        printf("marker_y = %f\r\n",marker_y);
        printf("marker_z = %f\r\n",marker_z);
        **************************************/
		ar_lable_land_last_position_x = local_pos.pose.pose.position.x;
		ar_lable_land_last_position_y = local_pos.pose.pose.position.y;	
		if(fabs(marker_x) < marker_error_max && fabs(marker_y) < marker_error_max)
		{
			ar_lable_land_init_position_flag = false;
			ROS_INFO("到达目标的正上/前方");	
			return true;
		}
		else
		{	
			//摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
			//无人机左右移动速度控制
			if(marker_x >= marker_error_max)
			{
				setpoint_raw.position.y = local_pos.pose.pose.position.y - 0.15;
			}					
			else if(marker_x <= -marker_error_max)
			{
				setpoint_raw.position.y = local_pos.pose.pose.position.y + 0.15;
			}	
			else
			{
				setpoint_raw.position.y = ar_lable_land_last_position_y;
			}					  
			//无人机前后移动速度控制
			if(marker_y >= marker_error_max)
			{
				setpoint_raw.position.x = local_pos.pose.pose.position.x - 0.15;
			}
			else if(marker_y <= -marker_error_max)
			{
				setpoint_raw.position.x = local_pos.pose.pose.position.x + 0.15;
			}
			else
			{
				setpoint_raw.position.x = ar_lable_land_last_position_x;
			}
		}				
	}
	else
	{	
		setpoint_raw.position.x = ar_lable_land_last_position_x;
		setpoint_raw.position.y = ar_lable_land_last_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = altitude;
	setpoint_raw.yaw        = 0;
	return false;
}




/************************************************************************
函数功能11: move_base速度转换
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool mission_obs_avoid_flag = false;
bool mission_obs_avoid(float x, float y, float z, float yaw)
{
	if(mission_obs_avoid_flag == false)
	{
		mission_obs_avoid_flag = true;
		ROS_INFO("发布避障导航目标点 x = %f, y = %f, z = %f, yaw = %f",x, y , z, yaw);
	}
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(1.0)))
	{
	    ROS_INFO("Waiting for the move_base action server to come up");
	}
	move_base_msgs::MoveBaseGoal first_goal;
	first_goal.target_pose.header.frame_id = "map";
	first_goal.target_pose.header.stamp = ros::Time::now();
	first_goal.target_pose.pose.position.x = x;
	first_goal.target_pose.pose.position.y = y;
	first_goal.target_pose.pose.position.z = ALTITUDE;
	first_goal.target_pose.pose.orientation.w = 1;
    ac.sendGoal(first_goal);
    if(fabs(local_pos.pose.pose.position.x-x)<0.3 && fabs(local_pos.pose.pose.position.y-y)<0.3)
	{
		ROS_INFO("到达目标点，自主导航任务完成");
		return true;
	}
	return false;
}


/************************************************************************
函数功能12: move_base
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
void cmd_to_px4(const geometry_msgs::Twist &msg);
void cmd_to_px4(const geometry_msgs::Twist &msg)
{
	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
	setpoint_raw.coordinate_frame = 8;
	setpoint_raw.velocity.x = msg.linear.x;;
	setpoint_raw.velocity.y = msg.linear.y;
	setpoint_raw.position.z = ALTITUDE;
	setpoint_raw.yaw_rate   = msg.angular.z;
} 


/************************************************************************
函数功能13: 穿越圆框，发布圆框后方目标点，效果良好，有时间以后可以继续优化
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float target_through_init_position_x = 0;
float target_through_init_position_y = 0;
bool  target_through_init_position_flag = false;
bool  target_through(float pos_x, float pos_y, float z, float yaw);
bool  target_through(float pos_x, float pos_y, float z, float yaw)
{
	//初始化位置一次即可，用于获取无人机初始位置
	if(!target_through_init_position_flag)
	{
		target_through_init_position_x = local_pos.pose.pose.position.x;
		target_through_init_position_y = local_pos.pose.pose.position.y;
		target_through_init_position_flag = true;
	}
	setpoint_raw.position.x = target_through_init_position_x + pos_x;
	setpoint_raw.position.y = target_through_init_position_y + pos_y;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw        = yaw;
	
	double current_yaw,a,b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);
	if(fabs(local_pos.pose.pose.position.x-pos_x-target_through_init_position_x)<0.1 
	&& fabs(local_pos.pose.pose.position.y-pos_y-target_through_init_position_y)<0.1
	&& fabs(current_yaw - yaw )< 0.1)
	{
		target_through_init_position_flag = false;
		ROS_INFO("到达目标点，穿越圆框任务完成");
		return true;
	}
	return false;	
}



/************************************************************************
函数功能14:降落
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool  precision_land_init_position_flag = false;
void  precision_land();
void  precision_land()
{
	if(!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
}



/************************************************************************
函数功能:ego_planner导航
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
// quadrotor_msgs::PositionCommand ego_sub;
// void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
// void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
// {
// 	ego_sub = *msg;
// }

/************************************************************************
函数功能: ego_planner是否规划出航线
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
// bool rec_traj_flag = false;
// void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg);
// void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg)
// {
// 	rec_traj_flag = msg->data;
// }

// void PI_attitude_control()
// {
// 	setpoint_raw.coordinate_frame = 1;
// 	setpoint_raw.type_mask = 0b101111100011; // vx vy z yaw
// 	setpoint_raw.velocity.x =  (ego_sub.position.x - local_pos.pose.pose.position.x) * 1;
// 	setpoint_raw.velocity.y =  ego_sub.position.y - local_pos.pose.pose.position.y * 1;
// 	setpoint_raw.position.z = ego_sub.position.z;
// 	setpoint_raw.yaw = ego_sub.yaw;

// 	ROS_INFO("ego: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", ego_sub.velocity.x, ego_sub.velocity.y, ego_sub.position.z, ego_sub.yaw);
// 	ROS_INFO("ego: x = %.2f, y = %.2f, z = %.2f, yaw = %.2f", ego_sub.position.x, ego_sub.position.y, ego_sub.position.z, ego_sub.yaw);
// 	ROS_INFO("已触发控制器: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.position.z, setpoint_raw.yaw);
// }

// /************************************************************************
// 函数功能:ego_planner发布目标点函数
// //1、定义变量
// //2、函数声明
// //3、函数定义
// *************************************************************************/
// float before_ego_pose_x = 0;
// float before_ego_pose_y = 0;
// float before_ego_pose_z = 0;
// bool pub_ego_goal_flag = false;
// bool pub_ego_goal(float x, float y, float z, float err_max, int first_target = 0);
// bool pub_ego_goal(float x, float y, float z, float err_max, int first_target)
// {
// 	before_ego_pose_x = local_pos.pose.pose.position.x;
// 	before_ego_pose_y = local_pos.pose.pose.position.y;
// 	before_ego_pose_z = local_pos.pose.pose.position.z;

// 	double current_yaw, a, b;
// 	tf::Quaternion quat;
// 	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
// 	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

// 	geometry_msgs::PoseStamped target_point;
// 	if (pub_ego_goal_flag == false)
// 	{
// 		pub_ego_goal_flag = true;
// 		target_point.pose.position.x = x;
// 		target_point.pose.position.y = y;
// 		target_point.pose.position.z = z;

// 		planner_goal_pub.publish(target_point);
// 		ROS_INFO("发送目标点( %f, %f, %f )", x, y, z);

// 		finish_ego_flag.data = false;
// 		finish_ego_pub.publish(finish_ego_flag);
// 	}

// 	if (rec_traj_flag == true)
// 	{
// 		PI_attitude_control();
		
		
// 		if (ego_sub.position.z > map_size_z)
// 		{
// 			setpoint_raw.position.z = map_size_z;
// 			std::cout << "exceed map_size_z" << std::endl;
// 		}
// 		else if (ego_sub.position.z < ground_height)
// 		{
// 			setpoint_raw.position.z = ground_height;
// 			std::cout << "lower than ground height" << std::endl;
// 		}
		

// 		if (first_target == 1)
// 		{
// 			std::cout << "get traj" << std::endl;
// 			std::cout << "ego_x : " << ego_sub.position.x << std::endl;
// 			std::cout << "ego_y : " << ego_sub.position.y << std::endl;
// 			std::cout << "ego_z : " << ego_sub.position.z << std::endl;
// 			std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
// 			setpoint_raw.type_mask = 0b101111000000; // 101 111 000 000  vx vy　vz x y z+ yaw
// 			setpoint_raw.coordinate_frame = 1;
// 			setpoint_raw.position.x = ego_sub.position.x;
// 			setpoint_raw.position.y = ego_sub.position.y;
// 			setpoint_raw.position.z = ego_sub.position.z;
// 			setpoint_raw.velocity.x = ego_sub.velocity.x;
// 			setpoint_raw.velocity.y = ego_sub.velocity.y;
// 			setpoint_raw.velocity.z = ego_sub.velocity.z;
// 			setpoint_raw.yaw = 0;
// 		}
// 		if (first_target == 2)
// 		{
// 			std::cout << "get traj" << std::endl;
// 			std::cout << "ego_x : " << ego_sub.position.x << std::endl;
// 			std::cout << "ego_y : " << ego_sub.position.y << std::endl;
// 			std::cout << "ego_z : " << ego_sub.position.z << std::endl;
// 			std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
// 			setpoint_raw.type_mask = 0b101111000000; // 101 111 000 000  vx vy　vz x y z+ yaw
// 			setpoint_raw.coordinate_frame = 1;
// 			setpoint_raw.position.x = ego_sub.position.x;
// 			setpoint_raw.position.y = ego_sub.position.y;
// 			setpoint_raw.position.z = ego_sub.position.z;
// 			setpoint_raw.velocity.x = ego_sub.velocity.x;
// 			setpoint_raw.velocity.y = ego_sub.velocity.y;
// 			setpoint_raw.velocity.z = ego_sub.velocity.z;
// 			setpoint_raw.yaw = 3.14;
// 		}
// 	}
// 	else
// 	{
// 		std::cout << "not rec_tarj" << std::endl;
// 		setpoint_raw.position.x = before_ego_pose_x;
// 		setpoint_raw.position.y = before_ego_pose_y;
// 		setpoint_raw.position.z = before_ego_pose_z;
// 		setpoint_raw.yaw = current_yaw;
// 		setpoint_raw.type_mask = 0b101111111000; // 101 111 111 000  x y z+ yaw
// 		setpoint_raw.coordinate_frame = 1;
// 	}

// 	setpoint_raw.header.stamp = ros::Time::now();

// 	if (fabs(local_pos.pose.pose.position.x - x) < err_max && fabs(local_pos.pose.pose.position.y - y) < err_max)
// 	{
// 		ROS_INFO("到达目标点, ego_planner导航任务完成");
// 		pub_ego_goal_flag = false;

// 		finish_ego_flag.data = true;
// 		finish_ego_pub.publish(finish_ego_flag);

// 		return true;
// 	}
// 	return false;
// }


//函数功能：根据起点终点坐标，发布巡航点


/************************************************************************
函数功能:沿规划好的轨迹路线前进（lll.cpp)
//1、定义变量-------------flag==0为前进，flag==1为返程
//2、函数声明
//3、函数定义
*************************************************************************/
void obstacle_cb(const obstacle_detector::ObstacleStatus::ConstPtr& msg);


// extern mavros_msgs::PositionTarget setpoint_raw; 
// extern nav_msgs::Odometry local_pos; // 或者 current_odom

/**
 * @brief 轨迹跟踪控制器 (非阻塞版本，需在主循环中频繁调用)
 * @param trajectory 轨迹点数组 [x, y]
 * @param height 期望飞行高度
 * @param current_odom 当前里程计数据
 * @param err_max 到达判定阈值 (米)
 * @param offset_x/y/z 坐标系偏移量
 * @return true 如果轨迹全部完成，false 如果仍在进行中
 */
bool traj_follower_step(const std::vector<std::vector<float>>& trajectory, 
                        float height, 
                        const nav_msgs::Odometry& current_odom,
                        float err_max = 0.3,
                        float offset_x = 0.0,
                        float offset_y = 0.0,
                        // --- 速度控制参数 ---
                        float max_vel = 1.0f,      // 最大水平速度 m/s
                        float min_vel = 0.5f,      // 最小速度 m/s
                        float slow_down_radius = 1.0f,// 减速半径 m
						bool reset = false
                        ){
    
    // 静态变量：用于在多次调用之间保持状态
    static int target_idx = 0;
    static bool is_active = true;
    static bool current_point_sent = false;
    static float last_yaw = 0.0f; 

    // 辅助函数：限制数值范围
    auto clamp = [](float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    };

    // 1. 初始化检查
    if (trajectory.empty()) {
        ROS_ERROR("Trajectory is empty!");
        return true; 
    }

    // 如果是第一次调用或新任务开始，重置状态
    if (reset) {
        target_idx = 0;
        current_point_sent = false;
        is_active = true;
        last_yaw = 0.0f;
        ROS_INFO("[TrajFollower] Resetting tracker.");
        return false;
    }

    if (!is_active) {
        return true; 
    }

    // 2. 安全检查：定位数据是否过时
    double time_diff = (ros::Time::now() - current_odom.header.stamp).toSec();
    if (time_diff > 0.5 || time_diff < -0.5) {
        ROS_WARN_THROTTLE(1, "Localization Timeout! Holding position...");
        return false; 
    }

    // 3. 获取当前目标点
    if (target_idx >= trajectory.size()) {
        is_active = false;
        ROS_INFO("[TrajFollower] Trajectory Completed.");
        return true;
    }

    float raw_tx = trajectory[target_idx][0];
    float raw_ty = trajectory[target_idx][1];
    
    float target_x = raw_tx + offset_x;
    float target_y = raw_ty + offset_y;
    float target_z = height; 

    // 4. 获取当前位置
    float cx = current_odom.pose.pose.position.x;
    float cy = current_odom.pose.pose.position.y;

    // 5. 计算水平距离误差
    float dx = target_x - cx;
    float dy = target_y - cy;
    
    float dist_xy = std::sqrt(dx * dx + dy * dy);

    // 6. 步进逻辑：只有当当前点已被"确认发送"且水平距离足够近时，才切换下一个点
    if (current_point_sent && dist_xy < err_max) {
        target_idx++;
        
        // 检查是否全部完成
        if (target_idx >= trajectory.size()) {
            is_active = false;
            ROS_INFO("[TrajFollower] All points reached.");
            
            // 最后一步：保持在最后一个点，速度设为0
            setpoint_raw.type_mask = 0b101111000000; 
            setpoint_raw.coordinate_frame = 1;
            setpoint_raw.position.x = target_x;
            setpoint_raw.position.y = target_y;
            setpoint_raw.position.z = target_z;
            setpoint_raw.velocity.x = 0;
            setpoint_raw.velocity.y = 0;
            setpoint_raw.velocity.z = 0;
            setpoint_raw.yaw = last_yaw;
            return true; 
        }
        
        // 切换到新点，重置发送标志
        current_point_sent = false;
        
        // 更新目标以便计算新的 Yaw 和 速度
        raw_tx = trajectory[target_idx][0];
        raw_ty = trajectory[target_idx][1];
        target_x = raw_tx + offset_x;
        target_y = raw_ty + offset_y;
        
        // 重新计算误差
        dx = target_x - cx;
        dy = target_y - cy;
        dist_xy = std::sqrt(dx * dx + dy * dy);
    }

    // 7. 计算水平速度前馈 (Velocity Feedforward)
    float vx = 0.0f;
    float vy = 0.0f;

    if (dist_xy > 0.01f) {
        // 计算期望速度大小
        float speed = max_vel;
        
        // 如果在减速半径内，线性降低速度
        if (dist_xy < slow_down_radius) {
            speed = max_vel * (dist_xy / slow_down_radius);
        }
        
        // 限制最小速度
        speed = clamp(speed, min_vel, max_vel);

        // 计算速度分量
        vx = speed * (dx / dist_xy);
        vy = speed * (dy / dist_xy);
    }

    // 8. 计算 Yaw (朝向运动方向)
    float target_yaw = last_yaw;

    // 9. 更新全局 setpoint_raw 
    setpoint_raw.type_mask = 0b101111000000; 
    setpoint_raw.coordinate_frame = 1; // FRAME_LOCAL_NED
    setpoint_raw.position.x = target_x;
    setpoint_raw.position.y = target_y;
    setpoint_raw.position.z = target_z; // 填入目标高度作为参考，尽管我们忽略了 VZ
    setpoint_raw.velocity.x = vx;
    setpoint_raw.velocity.y = vy;
    setpoint_raw.velocity.z = 0.0;
    setpoint_raw.yaw = target_yaw;

    // 10. 标记当前点已发送
    current_point_sent = true;

    // 返回 false 表示任务尚未完成
    return false;
}


/**
 * @brief 从参数服务器读取形如 [[x1,y1], [x2,y2]] 的轨迹数据
 * @param nh 节点句柄
 * @param param_name 参数名，例如 "traj1_3/trajectory"
 * @param out_traj 输出容器
 * @return true if success
 */

bool loadTrajectoryArray(ros::NodeHandle& nh, const std::string& param_name, std::vector<std::vector<float>>& out_traj) {
    XmlRpc::XmlRpcValue list;
    
    // 1. 获取参数
    if (!nh.getParam(param_name, list)) {
        ROS_ERROR("Failed to get param: %s", param_name.c_str());
        return false;
    }

    // 2. 检查是否为数组
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Param %s is not an array!", param_name.c_str());
        return false;
    }

    out_traj.clear();
    int size = list.size();

    for (int i = 0; i < size; ++i) {
        XmlRpc::XmlRpcValue point = list[i];

        // 3. 检查点是否为数组且至少有两个元素
        if (point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() >= 2) {
            float x, y;

            // 安全转换 x (兼容 int/double)
            if (point[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
                x = static_cast<float>(static_cast<int>(point[0]));
            else
                x = static_cast<float>(static_cast<double>(point[0]));

            // 安全转换 y
            if (point[1].getType() == XmlRpc::XmlRpcValue::TypeInt)
                y = static_cast<float>(static_cast<int>(point[1]));
            else
                y = static_cast<float>(static_cast<double>(point[1]));

            out_traj.push_back({x, y});
        } else {
            ROS_WARN("Invalid point format at index %d in %s", i, param_name.c_str());
        }
    }

    ROS_INFO("Loaded [%s]: %zu points", param_name.c_str(), out_traj.size());
    return !out_traj.empty();
}

//原版轨迹巡线，只有位置控制
// bool traj_follower_step(const std::vector<std::vector<float>>& trajectory, 
//                         float height, 
//                         const nav_msgs::Odometry& current_odom,
//                         float err_max = 0.3,
//                         float offset_x = 0.0,
//                         float offset_y = 0.0,
//                         float offset_z = 0.0,
// 						bool reset = false){
    
//     // 静态变量：用于在多次调用之间保持状态
//     static int target_idx = 0;
//     static bool is_active = true;
//     static bool current_point_sent = false;
//     static float last_yaw = 0.0; // 记录最后的yaw，防止跳变



//     // 1. 初始化检查
//     if (trajectory.empty()) {
//         ROS_ERROR("Trajectory is empty!");
//         return true; // 视为完成
//     }

//     // 如果是第一次调用或新任务开始，重置状态
// 	if (reset) {
// 		target_idx = 0;
// 		current_point_sent = false;
// 		is_active = true;
// 		ROS_INFO("[TrajFollower] Resetting tracker.");
// 		return false;
//     }

//     if (!is_active) return true;

//     // 2. 安全检查：定位数据是否过时
//     if (ros::Time::now() - current_odom.header.stamp > ros::Duration(0.5)) {
//         ROS_WARN_THROTTLE(1, "Localization Timeout! Holding position...");
//         // 保持上一次的 setpoint_raw 不变，直接返回
//         return false; 
//     }

//     // 3. 获取当前目标点
//     // 防止越界
//     if (target_idx >= trajectory.size()) {
//         is_active = false;
//         ROS_INFO("[TrajFollower] Trajectory Completed.");
//         return true;
//     }

//     float raw_tx = trajectory[target_idx][0];
//     float raw_ty = trajectory[target_idx][1];
    
//     float target_x = raw_tx + offset_x;
//     float target_y = raw_ty + offset_y;
//     float target_z = height;

//     // 4. 获取当前位置
//     float cx = current_odom.pose.pose.position.x;
//     float cy = current_odom.pose.pose.position.y;

//     // 5. 计算距离误差
//     float dx = target_x - cx;
//     float dy = target_y - cy;
//     float dist = std::sqrt(dx * dx + dy * dy);

//     // 6. 步进逻辑：只有当当前点已被"确认发送"且距离足够近时，才切换下一个点
//     if (current_point_sent && dist < err_max) {
//         target_idx++;
        
//         // 检查是否全部完成
//         if (target_idx >= trajectory.size()) {
//             is_active = false;
//             ROS_INFO("[TrajFollower] All points reached.");
//             // 最后一步：保持在最后一个点
//             setpoint_raw.position.x = target_x;
//             setpoint_raw.position.y = target_y;
//             setpoint_raw.position.z = target_z;
//             setpoint_raw.yaw = last_yaw;
//             return true; 
//         }
        
//         // 切换到新点，重置发送标志
//         current_point_sent = false;
        
//         // 更新目标以便计算新的 Yaw
//         raw_tx = trajectory[target_idx][0];
//         raw_ty = trajectory[target_idx][1];
//         target_x = raw_tx + offset_x;
//         target_y = raw_ty + offset_y;
        
//         // 重新计算误差用于下面的 Yaw 计算
//         dx = target_x - cx;
//         dy = target_y - cy;
//     }

//     // 7. 计算 Yaw (朝向下一个点)
//     float target_yaw = last_yaw; // 默认保持
//     // if (dist > 0.05) { // 避免除以零或微小抖动
//     //     target_yaw = std::atan2(dy, dx);
//     //     last_yaw = target_yaw;
//     // }

//     // 8. 更新全局 setpoint_raw 
//     setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
//     setpoint_raw.coordinate_frame = 1;
//     setpoint_raw.position.x = target_x;
//     setpoint_raw.position.y = target_y;
//     setpoint_raw.position.z = target_z;
//     setpoint_raw.yaw = target_yaw;

//     // 9. 标记当前点已发送
//     current_point_sent = true;

//     // 返回 false 表示任务尚未完成，继续在主循环中调用
//     return false;
// }

/*-----------------------------视觉识别--------------------------*/
void yolo_cb(const yolov8_ros_msgs::YoloDetection::ConstPtr& msg);
