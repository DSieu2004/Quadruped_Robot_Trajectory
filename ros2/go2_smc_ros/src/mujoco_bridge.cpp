#include "simulation/go2_controller.hpp"
#include <GLFW/glfw3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <thread>

namespace {
constexpr double pi=3.14159265358979323846;
geometry_msgs::msg::Quaternion quaternion(const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Quaternion result; result.w=q.w();result.x=q.x();result.y=q.y();result.z=q.z();return result;
}

class Window {
 public:
  explicit Window(const mjModel* model,const std::string& control_law,
                  const std::string& trajectory_control_law,bool clear_environment)
      :controller_name_(control_law=="pid"?"PID":control_law=="mpc"?"MPC":"SMC"),
       trajectory_controller_name_(trajectory_control_law.empty()?"":
           (trajectory_control_law=="pid"?"PID":trajectory_control_law=="mpc"?"MPC":"SMC")),
       clear_environment_(clear_environment) {
    setenv("__NV_PRIME_RENDER_OFFLOAD","1",1);setenv("__GLX_VENDOR_LIBRARY_NAME","nvidia",1);
    if(!glfwInit()) throw std::runtime_error("GLFW failed");
    const std::string title=clear_environment_?"Go2 low-level "+controller_name_+
                                              " | trajectory "+trajectory_controller_name_+" | MuJoCo"
                                              :"Go2 "+controller_name_+" | ROS 2 SLAM | MuJoCo";
    window_=glfwCreateWindow(1100,750,title.c_str(),nullptr,nullptr);
    if(!window_) throw std::runtime_error("Cannot create MuJoCo window");
    glfwMakeContextCurrent(window_);glfwSwapInterval(1);
    const char* renderer=reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const char* vendor=reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    if(!renderer || !vendor || std::string(vendor).find("NVIDIA")==std::string::npos)
      throw std::runtime_error("NVIDIA rendering required");
    renderer_=renderer;std::cout<<"MuJoCo GPU: "<<renderer_<<std::endl;
    mjv_defaultCamera(&camera_);mjv_defaultOption(&option_);mjv_defaultScene(&scene_);mjr_defaultContext(&context_);
    option_.geomgroup[3]=0;option_.geomgroup[4]=1;
    mjv_makeScene(model,&scene_,3000);mjr_makeContext(model,&context_,mjFONTSCALE_150);
    // Frame the static room, independent of its dimensions; ignore the infinite floor.
    double xmin=0,xmax=0,ymin=0,ymax=0;
    for(int g=0;g<model->ngeom;++g){
      if(model->geom_group[g]!=4)continue;
      const auto* pos=model->geom_pos+3*g;const auto* size=model->geom_size+3*g;
      xmin=std::min(xmin,pos[0]-size[0]);xmax=std::max(xmax,pos[0]+size[0]);
      const double ry=model->geom_type[g]==mjGEOM_CYLINDER?size[0]:size[1];
      ymin=std::min(ymin,pos[1]-ry);ymax=std::max(ymax,pos[1]+ry);
    }
    camera_.distance=std::max(6.5,1.35*std::max(xmax-xmin,ymax-ymin));
    camera_.azimuth=120;camera_.elevation=-55;
    camera_.lookat[0]=0.5*(xmin+xmax);camera_.lookat[1]=0.5*(ymin+ymax);
    glfwSetWindowUserPointer(window_,this);
    glfwSetScrollCallback(window_,[](GLFWwindow* w,double,double y){auto& s=*static_cast<Window*>(glfwGetWindowUserPointer(w));s.camera_.distance=std::clamp(s.camera_.distance*std::exp(-0.1*y),0.5,40.0);});
    glfwSetCursorPosCallback(window_,[](GLFWwindow* w,double x,double y){
      auto& s=*static_cast<Window*>(glfwGetWindowUserPointer(w));
      if(glfwGetMouseButton(w,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS){s.camera_.azimuth-=0.3*(x-s.x_);s.camera_.elevation=std::clamp(s.camera_.elevation-0.3*(y-s.y_),-89.0,-5.0);}
      s.x_=x;s.y_=y;
    });
  }
  ~Window(){mjr_freeContext(&context_);mjv_freeScene(&scene_);glfwDestroyWindow(window_);glfwTerminate();}
  bool render(const mjModel* m,mjData* d,const simulation::ControlStatus& status) {
    if(glfwWindowShouldClose(window_) || glfwGetKey(window_,GLFW_KEY_ESCAPE)==GLFW_PRESS) return false;
    mjrRect viewport{0,0,0,0};glfwGetFramebufferSize(window_,&viewport.width,&viewport.height);
    mjv_updateScene(m,d,&option_,nullptr,&camera_,mjCAT_ALL,&scene_);mjr_render(viewport,&scene_,&context_);
    const std::string control_text = clear_environment_ && !trajectory_controller_name_.empty()
        ? "Trajectory controller: " + trajectory_controller_name_ + " -> /cmd_vel\nRobot low-level: " + controller_name_
        : controller_name_ + " /cmd_vel";
    const std::string text=std::string(status.fallen ? "FALLEN - restart simulation" : control_text)+
       (clear_environment_ ? "\nTrajectory mode: static obstacles removed\nGPU: " : "\nLiDAR -> SLAM Toolbox -> map -> Nav2\nGPU: ")+renderer_;
    mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,"ROS 2 / MuJoCo",text.c_str(),&context_);
    glfwSwapBuffers(window_);glfwPollEvents();return true;
  }
 private:
  std::string controller_name_;
  std::string trajectory_controller_name_;
  bool clear_environment_=false;
  GLFWwindow* window_=nullptr; mjvCamera camera_{};mjvOption option_{};mjvScene scene_{};mjrContext context_{};
  double x_=0,y_=0;std::string renderer_;
};

class Bridge : public rclcpp::Node {
 public:
  Bridge():Node("mujoco_bridge"),model_(nullptr,mj_deleteModel),data_(nullptr,mj_deleteData) {
    const auto scene=declare_parameter<std::string>("scene",GO2_SCENE);
    control_law_name_=declare_parameter<std::string>("control_law","smc");
    if(control_law_name_!="smc"&&control_law_name_!="pid"&&control_law_name_!="mpc")
      throw std::runtime_error("control_law must be smc, pid or mpc");
    trajectory_control_law_name_=declare_parameter<std::string>("trajectory_control_law","");
    if(!trajectory_control_law_name_.empty()&&trajectory_control_law_name_!="smc"&&
       trajectory_control_law_name_!="pid"&&trajectory_control_law_name_!="mpc")
      throw std::runtime_error("trajectory_control_law must be empty, smc, pid or mpc");
    const auto control_law=control_law_name_=="pid"?simulation::ControlLaw::Pid:
        control_law_name_=="mpc"?simulation::ControlLaw::Mpc:simulation::ControlLaw::Smc;
    clear_environment_=declare_parameter<bool>("clear_environment",false);
    const double trajectory_command_acceleration=declare_parameter<double>("trajectory_command_acceleration",0.15);
    if(!std::isfinite(trajectory_command_acceleration)||trajectory_command_acceleration<=0||trajectory_command_acceleration>0.35)
      throw std::runtime_error("trajectory_command_acceleration must be in (0, 0.35]");
    headless=declare_parameter<bool>("headless",false);
    rate=declare_parameter<double>("realtime_factor",1.0);
    const auto log_file=declare_parameter<std::string>("log_file","");
    if(!log_file.empty()){
      log_.open(log_file);if(!log_)throw std::runtime_error("Cannot open simulation log");
      log_<<"time,x,y,z,roll,pitch,yaw,speed,cmd_x,cmd_y,fallen,obstacle_contact_steps,cmd_yaw_rate\n";
    }
    if(!std::isfinite(rate)||rate<=0||rate>10)throw std::runtime_error("realtime_factor must be in (0,10]");
    char error[2048]{};model_.reset(mj_loadXML(scene.c_str(),nullptr,error,sizeof(error)));
    if(!model_)throw std::runtime_error(error);
    if(clear_environment_){
      int removed=0;
      for(int geom=0;geom<model_->ngeom;++geom){
        if(model_->geom_group[geom]!=4)continue;
        model_->geom_contype[geom]=0;model_->geom_conaffinity[geom]=0;model_->geom_rgba[4*geom+3]=0;
        model_->geom_group[geom]=5;
        ++removed;
      }
      RCLCPP_INFO(get_logger(),"Trajectory mode: disabled %d static environment geoms",removed);
    }
    data_.reset(mj_makeData(model_.get()));
    simulation::Go2Config controller_config;
    if(clear_environment_)controller_config.navigation.max_acceleration=trajectory_command_acceleration;
    controller_=std::make_unique<simulation::Go2Controller>(model_.get(),std::vector<navigation::Waypoint>{},controller_config,control_law);
    mj_resetDataKeyframe(model_.get(),data_.get(),0);mju_zero(data_->ctrl,model_->nu);mj_forward(model_.get(),data_.get());controller_->reset(data_.get());
    odom_=create_publisher<nav_msgs::msg::Odometry>("odom",10);
    joints_=create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    scan_=create_publisher<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS());
    clock_=create_publisher<rosgraph_msgs::msg::Clock>("/clock",10);
    status_=create_publisher<std_msgs::msg::String>("simulation/status",10);
    collisions_=create_publisher<std_msgs::msg::UInt64>("simulation/obstacle_contacts",rclcpp::QoS(1).transient_local());
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped laser;
    laser.header.frame_id="base_footprint";laser.child_frame_id="laser";
    laser.transform.translation.z=0.45;laser.transform.rotation.w=1;
    static_tf_->sendTransform(laser);
    command_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,[this](const geometry_msgs::msg::Twist::SharedPtr msg){
      if(std::isfinite(msg->linear.x)&&std::isfinite(msg->linear.y)&&std::isfinite(msg->angular.z)){
        requested_=*msg;last_command_=data_->time;
      }
    });
    RCLCPP_INFO(get_logger(),"500 Hz %s, ideal simulated odometry, 360-degree ray LiDAR; domain isolated by scripts/ros_env.bash",control_law_name_.c_str());
  }
  bool headless=false;double rate=1;
  mjModel* model(){return model_.get();}mjData* data(){return data_.get();}
  const std::string& control_law_name()const{return control_law_name_;}
  const std::string& trajectory_control_law_name()const{return trajectory_control_law_name_;}
  bool clear_environment()const{return clear_environment_;}
  const auto& control_status()const{return controller_->status();}
  void step() {
    mj_step1(model_.get(),data_.get());
    geometry_msgs::msg::Twist cmd=requested_;
    if(data_->time-last_command_>0.5)cmd=geometry_msgs::msg::Twist();
    const Eigen::Quaterniond q(data_->qpos[3],data_->qpos[4],data_->qpos[5],data_->qpos[6]);
    const auto r=q.toRotationMatrix();const double yaw=std::atan2(r(1,0),r(0,0));
    controller_->set_velocity(std::cos(yaw)*cmd.linear.x-std::sin(yaw)*cmd.linear.y,
                              std::sin(yaw)*cmd.linear.x+std::cos(yaw)*cmd.linear.y,cmd.angular.z);
    controller_->update(data_.get(),simulation::Mode::Velocity);mj_step2(model_.get(),data_.get());
    bool hit=false;
    for(int i=0;i<data_->ncon;++i){const auto& c=data_->contact[i];
      if(c.dist<=0&&(model_->geom_group[c.geom[0]]==4||model_->geom_group[c.geom[1]]==4))hit=true;
    }
    if(hit)++obstacle_contacts_;
    for(int i=0;i<model_->nq;++i)if(!std::isfinite(data_->qpos[i]))throw std::runtime_error("Non-finite physics state");
    if(++steps_%5==0){rosgraph_msgs::msg::Clock msg;msg.clock=stamp();clock_->publish(msg);}
    if(steps_%10==0)publish_state();
    if(steps_%50==0){
      publish_scan();std_msgs::msg::UInt64 count;count.data=obstacle_contacts_;collisions_->publish(count);
      if(log_.is_open()){const auto& s=controller_->status();log_<<data_->time<<','<<s.x<<','<<s.y<<','<<s.height<<','<<s.roll<<','<<s.pitch<<','<<s.yaw<<','<<s.speed<<','<<cmd.linear.x<<','<<cmd.linear.y<<','<<s.fallen<<','<<obstacle_contacts_<<','<<cmd.angular.z<<'\n';}
    }
    if(steps_%500==0){
      const auto& s=controller_->status();std_msgs::msg::String msg;
      msg.data=std::string(s.fallen?"FALLEN":control_law_name_+" OK")+" t="+std::to_string(data_->time)+" x="+std::to_string(s.x)+" y="+std::to_string(s.y)+" z="+std::to_string(s.height);
      status_->publish(msg);
      if(s.fallen)RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,
          "Robot fell at t=%.2f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f support=%d contacts=%d foot_z=[%.3f,%.3f,%.3f,%.3f] body_a=[%.2f,%.2f,%.2f] attitude_a=[%.2f,%.2f,%.2f] swing_z=[%.2f,%.2f,%.2f,%.2f] max_torque=%.2f residual=%.2f; restart simulation",
          data_->time,s.height,s.roll,s.pitch,s.yaw,s.support_feet,s.contact_feet,
          s.foot_heights[0],s.foot_heights[1],s.foot_heights[2],s.foot_heights[3],
          s.body_acceleration[0],s.body_acceleration[1],s.body_acceleration[2],
          s.attitude_acceleration[0],s.attitude_acceleration[1],s.attitude_acceleration[2],
          s.swing_acceleration[2],s.swing_acceleration[5],s.swing_acceleration[8],s.swing_acceleration[11],
          s.max_torque,s.force_residual);
    }
  }
 private:
  builtin_interfaces::msg::Time stamp()const{return rclcpp::Time(static_cast<int64_t>(data_->time*1e9),RCL_ROS_TIME);}
  void publish_state(){
    const auto time=stamp();
    const Eigen::Quaterniond q(data_->qpos[3],data_->qpos[4],data_->qpos[5],data_->qpos[6]);
    const auto r=q.toRotationMatrix();const double yaw=std::atan2(r(1,0),r(0,0));
    const Eigen::Quaterniond qyaw(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
    geometry_msgs::msg::TransformStamped odom,base;
    odom.header.stamp=time;odom.header.frame_id="odom";odom.child_frame_id="base_footprint";
    odom.transform.translation.x=data_->qpos[0];odom.transform.translation.y=data_->qpos[1];odom.transform.rotation=quaternion(qyaw);
    base.header.stamp=time;base.header.frame_id="base_footprint";base.child_frame_id="base_link";
    base.transform.translation.z=data_->qpos[2];base.transform.rotation=quaternion(qyaw.conjugate()*q);
    tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{odom,base});
    nav_msgs::msg::Odometry msg;msg.header=odom.header;msg.child_frame_id="base_footprint";
    msg.pose.pose.position.x=data_->qpos[0];msg.pose.pose.position.y=data_->qpos[1];msg.pose.pose.orientation=quaternion(qyaw);
    msg.twist.twist.linear.x=std::cos(yaw)*data_->qvel[0]+std::sin(yaw)*data_->qvel[1];
    msg.twist.twist.linear.y=-std::sin(yaw)*data_->qvel[0]+std::cos(yaw)*data_->qvel[1];
    const Eigen::Vector3d omega=r*Eigen::Map<const Eigen::Vector3d>(data_->qvel+3);msg.twist.twist.angular.z=omega.z();
    for(int k:{0,7,35}){msg.pose.covariance[k]=0.001;msg.twist.covariance[k]=0.002;}
    odom_->publish(msg);
    sensor_msgs::msg::JointState js;js.header.stamp=time;
    for(int j=0;j<model_->njnt;++j){
      if(model_->jnt_type[j]!=mjJNT_HINGE)continue;
      js.name.emplace_back(mj_id2name(model_.get(),mjOBJ_JOINT,j));
      js.position.push_back(data_->qpos[model_->jnt_qposadr[j]]);js.velocity.push_back(data_->qvel[model_->jnt_dofadr[j]]);
    }
    joints_->publish(js);
  }
  void publish_scan(){
    // Ideal, instantaneous, gravity-stabilized 2D LiDAR; only group 4 environment is ray-visible.
    sensor_msgs::msg::LaserScan scan;scan.header.stamp=stamp();scan.header.frame_id="laser";
    constexpr int count=720;scan.angle_min=-pi;scan.angle_increment=2*pi/count;scan.angle_max=scan.angle_min+(count-1)*scan.angle_increment;
    scan.range_min=0.08;scan.range_max=12;scan.scan_time=0.1;scan.time_increment=0;
    const Eigen::Quaterniond q(data_->qpos[3],data_->qpos[4],data_->qpos[5],data_->qpos[6]);
    const auto r=q.toRotationMatrix();const double yaw=std::atan2(r(1,0),r(0,0));
    const mjtNum origin[3]={data_->qpos[0],data_->qpos[1],0.45};
    const mjtByte groups[6]={0,0,0,0,1,0};
    scan.ranges.reserve(count);
    for(int i=0;i<count;++i){const double angle=yaw+scan.angle_min+i*scan.angle_increment;
      const mjtNum direction[3]={std::cos(angle),std::sin(angle),0};int hit=-1;
      const double distance=mj_ray(model_.get(),data_.get(),origin,direction,groups,1,-1,&hit);
      scan.ranges.push_back(distance>=scan.range_min&&distance<=scan.range_max?distance:std::numeric_limits<float>::infinity());
    }
    scan_->publish(scan);
  }
  std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model_;
  std::unique_ptr<mjData,decltype(&mj_deleteData)> data_;
  std::unique_ptr<simulation::Go2Controller> controller_;
  std::string control_law_name_="smc",trajectory_control_law_name_;
  bool clear_environment_=false;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr collisions_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
  geometry_msgs::msg::Twist requested_;double last_command_=-100;std::size_t steps_=0;
  std::uint64_t obstacle_contacts_=0;std::ofstream log_;
};
}
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{
    auto bridge=std::make_shared<Bridge>();std::unique_ptr<Window> window;
    if(!bridge->headless)window=std::make_unique<Window>(bridge->model(),bridge->control_law_name(),
        bridge->trajectory_control_law_name(),bridge->clear_environment());
    const auto start=std::chrono::steady_clock::now();std::size_t steps=0;
    while(rclcpp::ok()){
      if(steps%5==0)rclcpp::spin_some(bridge);
      bridge->step();++steps;
      if(window&&steps%10==0&&!window->render(bridge->model(),bridge->data(),bridge->control_status()))break;
      const auto deadline=start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(steps*bridge->model()->opt.timestep/bridge->rate));
      std::this_thread::sleep_until(deadline);
    }
    rclcpp::shutdown();return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
}
