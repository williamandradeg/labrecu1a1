#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/Utility.hh>
#include <atomic>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <array>

namespace celda {
class CellEnvironment : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
 public gz::sim::ISystemPostUpdate {
 gz::transport::Node node;
 gz::transport::Node::Publisher pub;
 std::atomic<bool> belt{false};
 std::atomic<int> requested{-1}; int owner=0; gz::sim::Entity attachment=0;
 std::array<gz::transport::Node::Publisher,3> attachmentPubs;
 gz::sim::Entity box{0},mir{0},u3{0},u5{0},marks{0};
 double last{-1}, offset{0};

 public: void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&,
 gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
   node.Subscribe("/celda/belt/run",&CellEnvironment::Command,this);
   pub=node.Advertise<gz::msgs::Pose_V>("/celda/poses");
   std::array<std::string,3> names{"ur3","ur5","mir"};
   for(int i=0;i<3;i++){
    node.Subscribe<gz::msgs::Empty>("/celda/"+names[i]+"/attach",[this,i](const gz::msgs::Empty&){requested=i+1;});
    node.Subscribe<gz::msgs::Empty>("/celda/"+names[i]+"/detach",[this](const gz::msgs::Empty&){requested=0;});
    attachmentPubs[i]=node.Advertise<gz::msgs::StringMsg>("/celda/"+names[i]+"/attached");
   }
 }

 void Command(const gz::msgs::Boolean& m){belt=m.data();}
 gz::sim::Entity model(const gz::sim::EntityComponentManager& e,const std::string& n){
 return e.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(n));
 }

 public: void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& e) override {
 if(info.paused)return;
 if(!box) {
   box=gz::sim::Model(model(e,"caja")).LinkByName(e,"body");
   mir=gz::sim::Model(model(e,"mir100")).LinkByName(e,"base_link");
   u3=gz::sim::Model(model(e,"ur3")).LinkByName(e,"ur3_vacuum");
   u5=gz::sim::Model(model(e,"ur5")).LinkByName(e,"ur5_vacuum");
   marks=model(e,"belt_marks");
 }
 if(box && mir && u3 && u5) {
   int cmd=requested.exchange(-1);
   if(cmd==0 && attachment){e.RequestRemoveEntity(attachment);attachment=0;owner=0;}
   if(cmd>0 && owner==0){
    auto parent=(cmd==1?u3:(cmd==2?u5:mir));
    attachment=e.CreateEntity();
    e.CreateComponent(attachment,gz::sim::components::DetachableJoint({parent,box,"fixed"}));
    owner=cmd;
   }
 }
 if(belt && box && owner==0) {
   auto pose=gz::sim::worldPose(box,e);
   auto p=pose.Pos();
   // Ideal belt drive, applied only to a parcel resting inside its surface.
   if(p.X()>-1.40 && p.X()<1.24 && std::abs(p.Y())<.20 && p.Z()>.85 && p.Z()<.915){
     double v=std::min(.16,std::max(0.,(1.15-p.X())*3.));
     gz::sim::Link(box).SetLinearVelocity(e,pose.Rot().Inverse()*gz::math::Vector3d(v,0,0));
     gz::sim::Link(box).SetAngularVelocity(e,gz::math::Vector3d::Zero);
   }
   offset=std::fmod(offset+.16*std::chrono::duration<double>(info.dt).count(),.1);
 }
 if(marks){
   for(int i=0;i<30;i++){
     auto ent=gz::sim::Model(marks).LinkByName(e,"mark_"+std::to_string(i));
     if(ent)e.SetComponentData<gz::sim::components::Pose>(ent,gz::math::Pose3d(-1.45+i*.1+offset,0,.803,0,0,0));
   }
 }
 }
 
 public: void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& e) override {
 double t=std::chrono::duration<double>(info.simTime).count();
 if(t-last<.05 && t>=last)return;last=t;
 gz::msgs::Pose_V msg;
 msg.mutable_header()->mutable_stamp()->set_sec((int)t);
 msg.mutable_header()->mutable_stamp()->set_nsec((int)((t-(int)t)*1e9));
 for(auto item:{std::pair<gz::sim::Entity,const char*>(box,"caja"),{mir,"mir100"},{u3,"ur3_tool"},{u5,"ur5_tool"}}){
  if(!item.first)continue;
  auto p=msg.add_pose();gz::msgs::Set(p,gz::sim::worldPose(item.first,e));p->set_name(item.second);
 }
 pub.Publish(msg);
 for(int i=0;i<3;i++){gz::msgs::StringMsg state;state.set_data(owner==i+1?"attached":"detached");attachmentPubs[i].Publish(state);}
 }
};
}
GZ_ADD_PLUGIN(celda::CellEnvironment,gz::sim::System,celda::CellEnvironment::ISystemConfigure,celda::CellEnvironment::ISystemPreUpdate,celda::CellEnvironment::ISystemPostUpdate)
