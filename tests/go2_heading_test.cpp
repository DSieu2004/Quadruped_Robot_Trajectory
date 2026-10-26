#include "simulation/go2_controller.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
constexpr double pi=3.14159265358979323846;
double wrap(double angle){return std::remainder(angle,2*pi);}
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
}
int main(){
  char error[2048]{};
  std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(QUADRUPED_MODEL_PATH,nullptr,error,sizeof(error)),mj_deleteModel);
  if(!model)throw std::runtime_error(error);
  std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
  simulation::Go2Controller controller(model.get(),{});
  auto reset=[&](double yaw){
    mj_resetDataKeyframe(model.get(),data.get(),0);
    data->qpos[3]=std::cos(yaw/2);data->qpos[4]=data->qpos[5]=0;data->qpos[6]=std::sin(yaw/2);
    mju_zero(data->ctrl,model->nu);mj_forward(model.get(),data.get());controller.reset(data.get());
  };
  auto run=[&](double target,double duration,bool forward){
    const double start=data->time;double max_tilt=0;
    while(data->time-start<duration){
      mj_step1(model.get(),data.get());
      const Eigen::Quaterniond q(data->qpos[3],data->qpos[4],data->qpos[5],data->qpos[6]);
      const auto r=q.toRotationMatrix();const double yaw=std::atan2(r(1,0),r(0,0));
      const double rate=std::clamp(1.4*wrap(target-yaw),-0.18,0.18);
      const double speed=forward?0.07:0;
      controller.set_velocity(speed*std::cos(yaw),speed*std::sin(yaw),rate);
      controller.update(data.get(),simulation::Mode::Velocity);
      const auto& s=controller.status();check(!s.fallen,"Robot fell while turning");
      max_tilt=std::max({max_tilt,std::abs(s.roll),std::abs(s.pitch)});
      for(int a=0;a<model->nu;++a)check(std::isfinite(data->ctrl[a])&&data->ctrl[a]>=model->actuator_ctrlrange[2*a]&&data->ctrl[a]<=model->actuator_ctrlrange[2*a+1],"Turning torque limit violated");
      for(int v=0;v<model->nv;++v)check(data->qfrc_applied[v]==0,"Unexpected generalized force during turning");
      for(int b=0;b<6*model->nbody;++b)check(data->xfrc_applied[b]==0,"Unexpected body force during turning");
      mj_step2(model.get(),data.get());
      for(int w=0;w<mjNWARNING;++w)check(data->warning[w].number==0,"MuJoCo turning warning");
    }
    const auto& s=controller.status();
    std::cout<<"target="<<target<<" yaw="<<s.yaw<<" error="<<wrap(s.yaw-target)<<" xy="<<s.x<<','<<s.y<<" height="<<s.height<<" tilt="<<max_tilt<<std::endl;
    check(std::abs(wrap(s.yaw-target))<0.06,"Yaw target not reached");
    check(max_tilt<0.35&&s.height>0.27,"Turning posture outside tolerance");
    if(!forward)check(std::hypot(s.x,s.y)<0.12,"Excessive translation during in-place turn");
  };
  reset(0);run(pi/2,16,false);run(0,16,false);
  const double initial_y=data->qpos[1];run(pi/2,20,true);
  check(data->qpos[1]-initial_y>0.6,"Forward walking did not follow rotated body heading");
  reset(3.05);run(-3.05,8,false);
  std::cout<<"PASS: signed in-place yaw, forward motion while turning, wraparound, torque and posture limits.\n";
}
