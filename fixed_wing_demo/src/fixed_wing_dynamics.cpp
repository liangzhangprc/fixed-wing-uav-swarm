// BSD-3-Clause. Educational six-degree-of-freedom fixed-wing dynamics.
// Gazebo integrates all motion. No pose/velocity commands are used.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/Utility.hh>

namespace fixed_wing_demo {
class Dynamics final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate {
  gz::sim::Link link;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher, imuPublisher;
  bool actuatorMode{false};
  double rudderCmd{0}, lastImuPublish{-1};
  std::mutex mutex;
  double throttle{0}, rollCmd{0}, pitchCmd{0}, lastPublish{-1}, lastCommand{-100};
  bool newCommand{false};
  static double clip(double x,double a,double b) { return std::clamp(x,a,b); }
  static double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf, gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &) override {
    link=gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm,"base_link"));
    link.EnableVelocityChecks(ecm);
    link.EnableAccelerationChecks(ecm);
    actuatorMode=sdf->HasElement("control_mode") && sdf->Get<std::string>("control_mode")=="actuator";
    const std::string prefix=sdf->HasElement("topic_prefix") ? sdf->Get<std::string>("topic_prefix") : "/fixed_wing";
    node.Subscribe(prefix+"/actuator_command", &Dynamics::ActuatorCommand,this);
    imuPublisher=node.Advertise<gz::msgs::IMU>(prefix+"/imu");
    node.Subscribe(prefix+"/command", &Dynamics::Command,this);
    publisher=node.Advertise<gz::msgs::Odometry>(prefix+"/odometry");
  }
  void ActuatorCommand(const gz::msgs::Twist &msg) {
    if(!actuatorMode) return;
    std::lock_guard<std::mutex> guard(mutex);
    throttle=clip(msg.linear().x(),0,1);
    rollCmd=clip(msg.angular().x(),-1,1);
    pitchCmd=clip(msg.angular().y(),-1,1);
    rudderCmd=clip(msg.angular().z(),-1,1);
    newCommand=true;
  }
  void Command(const gz::msgs::Twist &msg) {
    if(actuatorMode) return;
    std::lock_guard<std::mutex> guard(mutex);
    throttle=clip(msg.linear().x(),0,1);
    rollCmd=clip(msg.angular().x(),-0.65,0.65);
    pitchCmd=clip(msg.angular().y(),-0.30,0.22);
    newCommand=true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    const auto pose=link.WorldPose(ecm);
    const auto velocity=link.WorldLinearVelocity(ecm);
    const auto omega=link.WorldAngularVelocity(ecm);
    if(!pose || !velocity || !omega) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    double thrust,rc,pc,yc;
    {
      std::lock_guard<std::mutex> guard(mutex);
      if(newCommand) { lastCommand=now; newCommand=false; }
      thrust=now-lastCommand<1.0 ? throttle*22.0 : 0.0;
      rc=rollCmd; pc=pitchCmd; yc=rudderCmd;
    }
    const auto &rot=pose->Rot();
    auto bodyV=rot.RotateVectorReverse(*velocity);
    auto bodyW=rot.RotateVectorReverse(*omega);
    const double speed=velocity->Length();
    const double alpha=std::atan2(-bodyV.Z(),std::max(bodyV.X(),0.2));
    // Linear lift curve inside +/- 16 degrees, smooth stall beyond that.
    double cl=clip(0.10+4.8*alpha,-1.20,1.40);
    if(std::abs(alpha)>0.28) cl*=std::max(0.15,1.0-(std::abs(alpha)-0.28)*2.0);
    const double q=0.5*1.225*speed*speed;
    const double cd=0.035+0.065*cl*cl+0.35*std::pow(std::sin(alpha),2);
    gz::math::Vector3d force=rot.RotateVector({thrust,0,0});
    if(speed>0.2) {
      const auto direction=*velocity/speed;
      auto liftDirection=rot.RotateVector(gz::math::Vector3d::UnitZ);
      liftDirection-=direction*liftDirection.Dot(direction);
      if(liftDirection.Length()>0.01) liftDirection.Normalize();
      force+=liftDirection*(q*0.8*cl)-direction*(q*0.8*cd);
      // Vertical tail damps sideslip. Lift is perpendicular to air velocity.
      force+=rot.RotateVector({0,-0.45*speed*bodyV.Y(),0});
    }
    const auto euler=rot.Euler();
    const double course=std::atan2(velocity->Y(),velocity->X());
    const double yawRate=-9.81*std::tan(euler.X())/std::max(speed,10.0);
    // Reduced inner autopilot / control-surface moments; authority grows with airspeed.
    const double authority=clip(speed*speed/196.0,0.10,2.5);
    double rollTorque=clip(authority*(4.0*wrap(rc-euler.X())-1.1*bodyW.X()),-3,3);
    double pitchTorque=clip(authority*(5.0*wrap(pc-euler.Y())-1.3*bodyW.Y()),-3,3);
    double yawTorque=speed>5 ? clip(1.8*wrap(course-euler.Z())+0.9*(yawRate-bodyW.Z()),-1.5,1.5) : -0.5*bodyW.Z();
    if(actuatorMode) {
      // Direct PX4 servo outputs (FRD convention), not attitude targets.
      rollTorque=authority*(2.5*rc-0.35*bodyW.X());
      pitchTorque=authority*(-2.5*pc-0.35*bodyW.Y());
      yawTorque=authority*(-0.8*yc-0.25*bodyW.Z());
      if(speed>5) yawTorque+=1.8*wrap(course-euler.Z());
    }
    link.AddWorldWrench(ecm,force,rot.RotateVector({rollTorque,pitchTorque,yawTorque}));
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    double now=std::chrono::duration<double>(info.simTime).count();
    if(actuatorMode && (now-lastImuPublish>=0.0039 || now<lastImuPublish)) {
      auto pose=link.WorldPose(ecm); auto w=link.WorldAngularVelocity(ecm);
      auto acc=link.WorldLinearAcceleration(ecm);
      if(pose && w && acc) {
        gz::msgs::IMU imu;
        auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
        imu.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
        imu.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
        gz::msgs::Set(imu.mutable_orientation(),pose->Rot());
        gz::msgs::Set(imu.mutable_angular_velocity(),pose->Rot().RotateVectorReverse(*w));
        gz::msgs::Set(imu.mutable_linear_acceleration(),pose->Rot().RotateVectorReverse(*acc-gz::math::Vector3d(0,0,-9.81)));
        imuPublisher.Publish(imu);
        lastImuPublish=now;
      }
    }
    if(now-lastPublish<0.02 && now>=lastPublish) return;
    lastPublish=now;
    auto pose=link.WorldPose(ecm); auto v=link.WorldLinearVelocity(ecm); auto w=link.WorldAngularVelocity(ecm);
    if(!pose || !v || !w) return;
    gz::msgs::Odometry msg;
    auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto frame=msg.mutable_header()->add_data(); frame->set_key("frame_id"); frame->add_value("world");
    auto child=msg.mutable_header()->add_data(); child->set_key("child_frame_id"); child->add_value("base_link");
    gz::msgs::Set(msg.mutable_pose(),*pose);
    gz::msgs::Set(msg.mutable_twist()->mutable_linear(),pose->Rot().RotateVectorReverse(*v));
    gz::msgs::Set(msg.mutable_twist()->mutable_angular(),pose->Rot().RotateVectorReverse(*w));
    publisher.Publish(msg);
  }
};
}
GZ_ADD_PLUGIN(fixed_wing_demo::Dynamics,gz::sim::System,
  fixed_wing_demo::Dynamics::ISystemConfigure,
  fixed_wing_demo::Dynamics::ISystemPreUpdate,
  fixed_wing_demo::Dynamics::ISystemPostUpdate)
