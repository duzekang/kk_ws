#include "complete_mission.h"
#include "lib_library.h"

int mission_num = 0;  //确定当前任务
int tmp_marker_id = 1;
int takeoff_id = 0;
int route ;//路线选择
ros::Publisher shoot_control_pub;  // 在 main() 外声明
float target1_x = 0, target1_y = 0; //第一个门前
float target2_x = 0, target2_y = 0; //穿门后
float target3_x = 0, target3_y = 0; //中间点
float target4_x = 0, target4_y = 0; //中间点
float target5_x = 0, target5_y = 0; //投货点

float origin_x = 0.0;
float origin_y = 0.0;
float origin_z = 0.0;

float targetA_x = 0, targetA_y = 0; //A识别点
float targetB_x = 0, targetB_y = 0; //B识别点
string laser_target;
string yolo_result;

float err_max = 10;
float speed = 0;
float height = 0;

//避障时的速度阈值
float max_vel;      // 最大水平速度 m/s
float min_vel;      // 最小速度 m/s
float slow_down_radius;// 减速半径 m

int obs_1_3;
int obs_2_4;
int obs_2_3;
int obs_1_4;

std::vector<std::vector<float>> traj_1_3;
std::vector<std::vector<float>> traj_1_4;
std::vector<std::vector<float>> traj_2_3;
std::vector<std::vector<float>> traj_2_4;
std::vector<std::vector<float>> traj_1_3_back;
std::vector<std::vector<float>> traj_1_4_back;
std::vector<std::vector<float>> traj_2_3_back;
std::vector<std::vector<float>> traj_2_4_back;
std::vector<std::vector<float>>* current_traj_ptr = nullptr;
int current_traj_index = 0;
static bool is_initialized = false;
bool camera_switched = false;

ros::Publisher cargo_drop_pub;    // 投货
ros::Publisher laser_fire_pub;    // 激光
ros::Publisher camera_switch_pub; // 相机节点切换
ros::Publisher traj_pub;          // 避障轨迹

void print_param()
{
  std::cout << "target1 : ( " << target1_x << ", " << target1_y << " )" << std::endl;
  std::cout << "target2 : ( " << target2_x << ", " << target2_y << " )" << std::endl;
  std::cout << "target3 : ( " << target3_x << ", " << target3_y << " )" << std::endl;
  std::cout << "target4 : ( " << target4_x << ", " << target4_y << " )" << std::endl;
  std::cout << "target5 : ( " << target5_x << ", " << target5_y << " )" << std::endl;
  std::cout << "targetA : ( " << targetA_x << ", " << targetA_y << " )" << std::endl;
  std::cout << "targetB : ( " << targetB_x << ", " << targetB_y << " )" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "speed: " << speed << std::endl;
  std::cout << "height: " << height << std::endl;
  std::cout << "max_vel: " << max_vel << std::endl;
  std::cout << "min_vel: " << min_vel << std::endl;
  std::cout << "slow_down_radius: " << slow_down_radius << std::endl;
}

void trigger_laser(bool flag)
{
  std_msgs::Bool msg;
  msg.data = flag;
  laser_fire_pub.publish(msg);
  ROS_INFO("[Laser Trigger] 已发送激光触发信号");
}

void trigger_drop(bool flag)
{
  std_msgs::Bool msg;
  msg.data = flag;
  cargo_drop_pub.publish(msg);
  ROS_INFO("[Drop Trigger] 已发送投货触发信号");
}

void camera_switch(){
  if(!camera_switched){
    std_msgs::String msg;
    msg.data = "forward";
    camera_switch_pub.publish(msg);
    camera_switched = true;
    ROS_INFO("<<<--------已发送前视摄像头切换指令-------->>>");
  }
}

void print_publish()
{
  std::cout<< "setpoint_raw.x:"<<setpoint_raw.position.x<<std::endl;
  
  std::cout<< "setpoint_raw.y:"<<setpoint_raw.position.y<<std::endl;
  
  std::cout<< "setpoint_raw.z:"<<setpoint_raw.position.z<<std::endl;
  
  std::cout<< "setpoint_raw.yaw:"<<setpoint_raw.yaw<<std::endl;

  std::cout<<"now_mission_number:"<<mission_num<<std::endl;

  std::cout<<"route:"<<route<<std::endl;

  std::cout<<"laser_target:"<<laser_target<<std::endl;

}

void obstacle_cb(const obstacle_detector::ObstacleStatus::ConstPtr& msg){

  if(msg->obstacle_detected){
    if(msg->obs_1 && msg->obs_3) route = 103;
    if(msg->obs_1 && msg->obs_4) route = 104;
    if(msg->obs_2 && msg->obs_3) route = 203;
    if(msg->obs_2 && msg->obs_4) route = 204;
  }
}

void yolo_cb(const yolov8_ros_msgs::YoloDetection::ConstPtr& msg){
  yolo_result = msg->class_name;
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "lll");

  // 创建节点句柄
  ros::NodeHandle nh;

  // // 订阅ego_planner规划出来的结果
  // ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

  // // 发布ego_planner目标
  // planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

  // ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  // finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  //订阅识别
  ros::Subscriber yolo_sub = nh.subscribe<yolov8_ros_msgs::YoloDetection>("/yolo/detections", 10, yolo_cb);

  //订阅障碍物类型消息
  ros::Subscriber obstacle_sub = nh.subscribe<obstacle_detector::ObstacleStatus>("/obstacle/result", 10, obstacle_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  shoot_control_pub = nh.advertise<std_msgs::Bool>("/shoot_control_topic", 10);

  cargo_drop_pub = nh.advertise<std_msgs::Bool>("/dropper_open", 10);

  laser_fire_pub = nh.advertise<std_msgs::Bool>("/laser_on", 10);

  camera_switch_pub = nh.advertise<std_msgs::String>("/yolo/switch_camera", 10);

  traj_pub = nh.advertise<std_msgs::Int32>("/trajectory_visualizer/select_id", 10);

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);
  
  //  参数读取
  nh.param<float>("target1_x", target1_x, 0);
  nh.param<float>("target1_y", target1_y, 0);

  nh.param<float>("target2_x", target2_x, 0);
  nh.param<float>("target2_y", target2_y, 0);  

  nh.param<float>("target3_x", target3_x, 0);
  nh.param<float>("target3_y", target3_y, 0);

  nh.param<float>("target4_x", target4_x, 0);
  nh.param<float>("target4_y", target4_y, 0);

  nh.param<float>("target5_x", target5_x, 0);
  nh.param<float>("target5_y", target5_y, 0);

  nh.param<float>("targetA_x", targetA_x, 0);
  nh.param<float>("targetA_y", targetA_y, 0);

  nh.param<float>("targetB_x", targetB_x, 0);
  nh.param<float>("targetB_y", targetB_y, 0);

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("speed", speed, 0.1);
  nh.param<float>("height", height, 1.4);

  nh.param<float>("max_vel", max_vel, 1.0f);
  nh.param<float>("min_vel", min_vel, 0.5f);
  nh.param<float>("slow_down_radius", slow_down_radius, 1.0f);

  loadTrajectoryArray(nh, "traj1_3/trajectory", traj_1_3);
  loadTrajectoryArray(nh, "traj1_4/trajectory", traj_1_4);
  loadTrajectoryArray(nh, "traj2_3/trajectory", traj_2_3);
  loadTrajectoryArray(nh, "traj2_4/trajectory", traj_2_4);
  
  loadTrajectoryArray(nh, "traj1_3_back/trajectory", traj_1_3_back);
  loadTrajectoryArray(nh, "traj1_4_back/trajectory", traj_1_4_back);
  loadTrajectoryArray(nh, "traj2_3_back/trajectory", traj_2_3_back);
  loadTrajectoryArray(nh, "traj2_4_back/trajectory", traj_2_4_back);
  
  print_param();

  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> takeoff_id;
  if (takeoff_id != 1)
    return 0;

  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  //控制offboard和arm循环，初始化飞机状态
  
   while (ros::ok())
   {
     if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
     {
       if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
       {
         ROS_INFO("Offboard enabled");
       }
       last_request = ros::Time::now();
     }
     else
     {
       if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
       {
         if (arming_client.call(arm_cmd) && arm_cmd.response.success)
         {
           ROS_INFO("Vehicle armed");
         }
         last_request = ros::Time::now();
       }
     }
     //  当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
     if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.1 )
     {
        mission_num = 1; //开始任务
        break;
     }
     mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
     mavros_setpoint_pos_pub.publish(setpoint_raw);
     
     ros::spinOnce();
     rate.sleep();
  }

  mission_num = 1; // 起飞完成，进入任务1

  // ==== 正式进入任务流程 ====
  // 定义任务计时标志位
  bool tmp_time_record_start_flag = false;
  // 定义任务计时变量
  
  ros::Time tmp_mission_success_time_record;

  float tmp_x = 0;
  float tmp_y = 0;

  last_request = ros::Time::now(); //更新记录时间，正式进入任务循环

  origin_x = local_pos.pose.pose.position.x;
  origin_y = local_pos.pose.pose.position.y;
  origin_z = local_pos.pose.pose.position.z;//记录起飞后的位置作为偏移量

  //任务开始
  while (ros::ok())
  {
    printf("mission_num = %d\r\n", mission_num);
    switch (mission_num)
    {
    //任务一：...
    case 1: //悬停
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, 0))
      {
        if (lib_time_record_func(2.0, ros::Time::now())){
          mission_num = 2;
          last_request = ros::Time::now();
        }
      }
      else if(ros::Time::now() - last_request >= ros::Duration(4.0))
      {
        mission_num = 2;
        last_request = ros::Time::now();
        ROS_WARN("Timed out!!!!");
      }
      
      break;
    
    case 2://导航到第一个门前
      if (mission_pos_cruise(target1_x, target1_y, ALTITUDE, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
        {
          mission_num = 3;
          last_request = ros::Time::now();
        }
      }
      break;

    case 3://穿门（直接定点穿）
      if (mission_pos_cruise(target2_x, target2_y, ALTITUDE, 0, 0.2))
      {
        if (lib_time_record_func(1.0, ros::Time::now()))
        {
          mission_num = 4;     
          last_request = ros::Time::now();
        }
      }
      break;
    
    case 4://根据识别结果选择不同路线避障
      if(!is_initialized){
        std_msgs::Int32 msg;
        msg.data = route;
        traj_pub.publish(msg);
        switch (route)
        {
          case 103: current_traj_ptr = &traj_1_3; break;
          case 104: current_traj_ptr = &traj_1_4; break;
          case 203: current_traj_ptr = &traj_2_3; break;
          case 204: current_traj_ptr = &traj_2_4; break;
          default:  current_traj_ptr = nullptr;   break;
        }
        //清理之前数据
        if (current_traj_ptr && !current_traj_ptr->empty()) {
          traj_follower_step(*current_traj_ptr, ALTITUDE, local_pos, 0.3, origin_x, origin_y, max_vel, min_vel, slow_down_radius, true); 
          ROS_INFO("Return Trajectory Loaded: %zu points", current_traj_ptr->size());
        } else {
          ROS_ERROR("No return trajectory found for route %d!", route);
        }
        is_initialized = true;
      }


      ROS_INFO("<<<--------Trajectory following-------->>>");
      if (traj_follower_step(*current_traj_ptr, ALTITUDE, local_pos, 0.3, origin_x, origin_y, max_vel, min_vel, slow_down_radius, false)) {
        ROS_INFO("Trajectory following finished.");
        if (lib_time_record_func(1.5, ros::Time::now())) {
          mission_num = 40;
          is_initialized = false;
          last_request = ros::Time::now();
        }
      }
      break;
    
    case 40:
      if(mission_pos_cruise(target4_x, target4_y, ALTITUDE, 0, 0.2)){
          if (lib_time_record_func(1.5, ros::Time::now())){
            mission_num = 5;
            last_request = ros::Time::now();
          }
        }
      break;

    case 5://飞到识别/投货点  
      if(mission_pos_cruise(target5_x, target5_y, ALTITUDE, 0, 0.2))
      {
        if (lib_time_record_func(1.5, ros::Time::now())){
          trigger_drop(true);
          mission_num = 6;
          last_request = ros::Time::now();
        }
      }
      break;
    
    case 6://识别拟打击目标
      trigger_drop(false);//关闭舵机

      if(yolo_result == "A_down" || laser_target == "A"){
        ROS_INFO("<<<--------识别到A，飞往A点-------->>>");
        camera_switch();
        laser_target = "A";
        trigger_laser(true);
        if(mission_pos_cruise(targetA_x, targetA_y, height, 0, 0.2))
        { 
          if (lib_time_record_func(4.0, ros::Time::now())){
            mission_num = 7;
            last_request = ros::Time::now();
          }
        }
      }else if(yolo_result == "B_down" || laser_target == "B"){
        ROS_INFO("<<<--------识别到B，飞往A点-------->>>");
        camera_switch();
        laser_target = "B";
        trigger_laser(true);
        if(mission_pos_cruise(targetB_x, targetB_y, height, 0, 0.2))
        {
          if (lib_time_record_func(4.0, ros::Time::now())){
            mission_num = 7;
            last_request = ros::Time::now();
          }
        }
      }else {
        ROS_INFO("<<<--------未识别到任何目标-------->>>");
      }
      break;

      /*--------开始返程---------*/
      case 7://回到投货位置
        trigger_laser(false);
        
        if(mission_pos_cruise(target5_x, target5_y, ALTITUDE, 0, 0.2)){
          if (lib_time_record_func(1.5, ros::Time::now())){
            mission_num = 70;
            last_request = ros::Time::now();
          }
        }
        break;

      case 70:
        if(mission_pos_cruise(target4_x, target4_y, ALTITUDE, 0, 0.2)){
            if (lib_time_record_func(1.5, ros::Time::now())){
              mission_num = 8;
              last_request = ros::Time::now();
            }
          }
        break;
      
      case 8://回到中间点
        if(mission_pos_cruise(target3_x, target3_y, ALTITUDE, 0, 0.2)){
          if (lib_time_record_func(1.5, ros::Time::now())){
            mission_num = 9;
            last_request = ros::Time::now();
          }
        }
        break;

      case 9://选择路线回到门前
        if(!is_initialized){
          switch (route)
          {
            case 103: current_traj_ptr = &traj_1_3_back; break;
            case 104: current_traj_ptr = &traj_1_4_back; break;
            case 203: current_traj_ptr = &traj_2_3_back; break;
            case 204: current_traj_ptr = &traj_2_4_back; break;
            default: current_traj_ptr = nullptr; break;
          }
          //清理之前的数据
          if (current_traj_ptr && !current_traj_ptr->empty()) {
            traj_follower_step(*current_traj_ptr, ALTITUDE, local_pos, 0.3, origin_x, origin_y, max_vel, min_vel, slow_down_radius, true); 
            ROS_INFO("Return Trajectory Loaded: %zu points", current_traj_ptr->size());
          } else {
            ROS_ERROR("No return trajectory found for route %d!", route);
          }
          is_initialized = true;
        }

        ROS_INFO("<<<--------Trajectory following-------->>>");
        if (traj_follower_step(*current_traj_ptr, ALTITUDE, local_pos, 0.3, origin_x, origin_y, max_vel, min_vel, slow_down_radius, false)) {
          ROS_INFO("Trajectory following finished.");
          // 悬停一小会儿后进入下一个任务
          if (lib_time_record_func(1.5, ros::Time::now())) {
            mission_num = 10;
            is_initialized = false;
            last_request = ros::Time::now();
          }
        }

        break;

      case 10://返程穿门
        if (mission_pos_cruise(target1_x, target1_y, ALTITUDE, 0, 0.2)){
            if (lib_time_record_func(1.5, ros::Time::now())){
              mission_num = 11;
              last_request = ros::Time::now();
            }
        }
        break;

      case 11://回到出发点
        if(mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2)){
          if (lib_time_record_func(1.5, ros::Time::now())){
            mission_num = 0;
            last_request = ros::Time::now();
          }
        }
        break;
      
      //直接降落
      case 0:
        if (lib_time_record_func(2.0, ros::Time::now())){
          ROS_INFO("AUTO.LAND");
          offb_set_mode.request.custom_mode = "AUTO.LAND";
          set_mode_client.call(offb_set_mode);
          mission_num = -1;
        }
        else if(ros::Time::now() - last_request >= ros::Duration(5.0))
        {
          ROS_INFO("AUTO.LAND");
          offb_set_mode.request.custom_mode = "AUTO.LAND";
          set_mode_client.call(offb_set_mode);
          mission_num = -1;
        }
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    print_publish();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}