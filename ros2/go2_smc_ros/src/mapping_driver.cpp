#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>

class Driver:public rclcpp::Node{
 public:
  Driver():Node("mapping_driver"){
    const auto xs=declare_parameter<std::vector<double>>("route_x",{-2.9,4.9,4.9,-2.9,-2.9,0});
    const auto ys=declare_parameter<std::vector<double>>("route_y",{-2.9,-2.9,2.9,2.9,-2.9,0});
    speed_=declare_parameter<double>("cruise_speed",0.10);
    if(xs.empty()||xs.size()!=ys.size()||!std::isfinite(speed_)||speed_<=0||speed_>0.12)
      throw std::invalid_argument("Invalid mapping route or cruise_speed");
    for(std::size_t i=0;i<xs.size();++i){
      if(!std::isfinite(xs[i])||!std::isfinite(ys[i]))throw std::invalid_argument("Non-finite mapping waypoint");
      route_.emplace_back(xs[i],ys[i]);
    }
    pub_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    done_=create_publisher<std_msgs::msg::Bool>("mapping_route_complete",rclcpp::QoS(1).transient_local());
    sub_=create_subscription<nav_msgs::msg::Odometry>("odom",10,[this](nav_msgs::msg::Odometry::SharedPtr m){
      odom_=*m;ready_=true;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
  }
 private:
  void tick(){
    if(!ready_)return;
    geometry_msgs::msg::Twist cmd;
    const double time=rclcpp::Time(odom_.header.stamp).seconds();
    const double x=odom_.pose.pose.position.x,y=odom_.pose.pose.position.y;
    if(time>2.5 && index_<route_.size()){
      double dx=route_[index_].first-x,dy=route_[index_].second-y;
      const double dist=std::hypot(dx,dy);
      if(dist<0.09){RCLCPP_INFO(get_logger(),"Mapping waypoint %zu reached at %.3f, %.3f",index_+1,x,y);++index_;}
      else{
        const double speed=std::min(speed_,0.8*dist);
        const auto& q=odom_.pose.pose.orientation;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        const double vx=speed*dx/dist,vy=speed*dy/dist;
        cmd.linear.x=std::cos(yaw)*vx+std::sin(yaw)*vy;
        cmd.linear.y=-std::sin(yaw)*vx+std::cos(yaw)*vy;
      }
    }
    pub_->publish(cmd);
    std_msgs::msg::Bool complete;complete.data=index_==route_.size();done_->publish(complete);
    if(complete.data&&!reported_){reported_=true;RCLCPP_INFO(get_logger(),"Mapping loop complete. Save /map before starting localization/navigation.");}
  }
  std::vector<std::pair<double,double>> route_;
  double speed_=0.10;
  std::size_t index_=0;bool ready_=false,reported_=false;nav_msgs::msg::Odometry odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Driver>());rclcpp::shutdown();}
