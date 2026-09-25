#include <ignition/transport/Node.hh>
#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
#include <iostream>
std::atomic<bool> running{true};
void stop(int){running=false;}
int main(){signal(SIGINT,stop);signal(SIGTERM,stop);ignition::transport::Node node;
auto clock=node.Advertise<ignition::msgs::Clock>("/clock");
auto poses=node.Advertise<ignition::msgs::Pose_V>("/world/a0/pose/info");
auto samples=node.Advertise<ignition::msgs::StringMsg>("/world/a0/workcell_measurements");
ignition::msgs::Clock c;ignition::msgs::Pose_V p;ignition::msgs::StringMsg s;s.set_data(std::string(32000,'x'));
for(int i=0;i<30;i++){auto q=p.add_pose();q->set_name("object_"+std::to_string(i));q->mutable_position()->set_x(.1);q->mutable_orientation()->set_w(1.);}
std::cout<<"PUBLISHER_READY"<<std::endl;int64_t tick=0;
while(running){c.mutable_sim()->set_sec(tick/1000);c.mutable_sim()->set_nsec((tick%1000)*1000000);clock.Publish(c);poses.Publish(p);samples.Publish(s);tick++;std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
