#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>

int main(int argc,char**argv){
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("go2_navigation_goal");
  const double x=node->declare_parameter<double>("x",4.6),y=node->declare_parameter<double>("y",0.0);
  const double target_yaw=node->declare_parameter<double>("yaw",0.0);
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(target_yaw)){rclcpp::shutdown();return 2;}
  using Action=nav2_msgs::action::NavigateToPose;
  auto state_client=node->create_client<lifecycle_msgs::srv::GetState>("/bt_navigator/get_state");
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(30);
  bool active=false;
  while(rclcpp::ok()&&std::chrono::steady_clock::now()<deadline){
    if(!state_client->wait_for_service(std::chrono::seconds(1)))continue;
    auto state=state_client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if(rclcpp::spin_until_future_complete(node,state,std::chrono::seconds(1))==rclcpp::FutureReturnCode::SUCCESS&&state.get()->current_state.id==3){active=true;break;}
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  if(!active){RCLCPP_ERROR(node->get_logger(),"Nav2 is not active");rclcpp::shutdown();return 2;}
  bool fallen=false;std::uint64_t contacts=0;double error=std::numeric_limits<double>::infinity(),yaw_error=error,max_y=0;
  auto status=node->create_subscription<std_msgs::msg::String>("simulation/status",10,[&](std_msgs::msg::String::SharedPtr m){fallen=fallen||m->data.find("FALLEN")!=std::string::npos;});
  auto collision=node->create_subscription<std_msgs::msg::UInt64>("simulation/obstacle_contacts",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::UInt64::SharedPtr m){contacts=m->data;});
  std::ofstream trace;const auto trace_file=node->declare_parameter<std::string>("trace_file","");
  if(!trace_file.empty()){trace.open(trace_file);if(!trace){rclcpp::shutdown();return 2;}trace<<"time,map_x,map_y,distance_to_goal,map_yaw,yaw_error\n";}
  auto client=rclcpp_action::create_client<Action>(node,"navigate_to_pose");
  if(!client->wait_for_action_server(std::chrono::seconds(30))){RCLCPP_ERROR(node->get_logger(),"Nav2 action server unavailable");rclcpp::shutdown();return 2;}
  Action::Goal goal;goal.pose.header.frame_id="map";goal.pose.header.stamp=node->now();goal.pose.pose.position.x=x;goal.pose.pose.position.y=y;
  goal.pose.pose.orientation.w=std::cos(target_yaw/2);goal.pose.pose.orientation.z=std::sin(target_yaw/2);
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback=[&](auto,const std::shared_ptr<const Action::Feedback> feedback){
    const auto& p=feedback->current_pose.pose.position;error=std::hypot(p.x-x,p.y-y);max_y=std::max(max_y,std::abs(p.y));
    const auto& q=feedback->current_pose.pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    yaw_error=std::remainder(yaw-target_yaw,2*3.14159265358979323846);
    if(trace.is_open())trace<<node->now().seconds()<<','<<p.x<<','<<p.y<<','<<error<<','<<yaw<<','<<yaw_error<<'\n';
  };
  auto sent=client->async_send_goal(goal,options);
  if(rclcpp::spin_until_future_complete(node,sent,std::chrono::seconds(10))!=rclcpp::FutureReturnCode::SUCCESS||!sent.get()){
    RCLCPP_ERROR(node->get_logger(),"Nav2 rejected goal");rclcpp::shutdown();return 3;
  }
  RCLCPP_INFO(node->get_logger(),"Nav2 accepted map goal %.2f, %.2f",x,y);
  auto result=client->async_get_result(sent.get());
  const auto wait=rclcpp::spin_until_future_complete(node,result,std::chrono::seconds(180));
  if(wait!=rclcpp::FutureReturnCode::SUCCESS){client->async_cancel_all_goals();RCLCPP_ERROR(node->get_logger(),"Goal timeout; cancel requested");rclcpp::shutdown();return 4;}
  const bool ok=result.get().code==rclcpp_action::ResultCode::SUCCEEDED&&!fallen&&contacts==0&&error<0.25&&std::abs(yaw_error)<0.12;
  std::cout<<(ok?"NAVIGATION SUCCEEDED":"NAVIGATION FAILED")<<" | feedback_error="<<error<<" m | yaw_error="<<yaw_error<<" rad | max_abs_y="<<max_y<<" m | obstacle_contact_steps="<<contacts<<" | fallen="<<fallen<<std::endl;rclcpp::shutdown();return ok?0:5;
}
