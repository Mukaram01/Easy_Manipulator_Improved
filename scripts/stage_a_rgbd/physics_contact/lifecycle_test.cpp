#include "provider.cpp"
#include <sdf/Root.hh>
#include <cassert>
int main() {
  sdf::Root root;
  assert(root.LoadSdfString(R"(<sdf version='1.8'><world name='a0'><plugin name='observer' filename='unused'><run_id>test</run_id><workpieces>part_00 part_01</workpieces><support_collision>a0::support</support_collision><diagnostic_output>/tmp/unused-contact-lifecycle.jsonl</diagnostic_output><diagnostic_step>100</diagnostic_step></plugin></world></sdf>)").empty());
  auto config=root.Element()->GetElement("world")->GetElement("plugin");
  sim::EntityComponentManager ecm;sim::EventManager events;
  auto world=ecm.CreateEntity();ecm.CreateComponent(world,components::Name("a0"));
  WorkcellPhysicsContactMeasurement observer;observer.Configure(world,config,ecm,events);
  auto collision=ecm.CreateEntity();ecm.CreateComponent(collision,components::Collision());
  assert(!ecm.Component<components::ContactSensorData>(collision));
  auto pre=dynamic_cast<sim::ISystemPreUpdate*>(&observer);
  assert(pre && "contact requests must run after collision entity creation");
  sim::UpdateInfo info;pre->PreUpdate(info,ecm);
  assert(ecm.Component<components::ContactSensorData>(collision));
  auto late=ecm.CreateEntity();ecm.CreateComponent(late,components::Collision());
  pre->PreUpdate(info,ecm);
  assert(ecm.Component<components::ContactSensorData>(late));
  // Repeated requests preserve populated contact data instead of replacing it.
  ignition::msgs::Contacts contacts;contacts.add_contact()->add_depth(.00001);
  ecm.Component<components::ContactSensorData>(collision)->Data()=contacts;
  pre->PreUpdate(info,ecm);
  assert(ecm.Component<components::ContactSensorData>(collision)->Data().contact_size()==1);
}
