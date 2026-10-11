// No Server, Configure, physics step, RenderUtil or graphics context.
#include <ignition/gazebo/SystemLoader.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/common/Console.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Plugin.hh>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <dlfcn.h>
#include <filesystem>
#include <set>
int main(int argc,char **argv) {
  if(argc!=3 && argc!=4)return 2;
  ignition::common::Console::SetVerbosity(4);
  Json::Value r;r["graphics_started"]=false;r["physics_steps"]=0;r["execution_goals"]=0;
  r["server_constructed"]=false;r["render_engine_constructed"]=false;
  r["registration_only_preload"]=argc==4;
  bool ok=false;
  try {
    // Reproduce a DT_NEEDED plugin's GLOBAL registration scope without engine Init.
    // dlopen is diagnostic setup; SystemLoader remains the acceptance authority.
    if(argc==4 && !dlopen(argv[3],RTLD_NOW|RTLD_GLOBAL))throw std::runtime_error(dlerror());
    Dl_info hook{};
    if(dladdr(dlsym(RTLD_DEFAULT,"IgnitionPluginHook"),&hook))
      r["global_registration_hook_library"]=hook.dli_fname;
    sdf::Root root;const auto errors=root.Load(argv[1]);
    if(!errors.empty() || root.WorldCount()!=1)throw std::runtime_error("invalid SDF world");
    const auto element=root.WorldByIndex(0)->Element();
    auto declaration=element->GetElement("plugin");
    if(!declaration || declaration->GetNextElement("plugin"))throw std::runtime_error("exactly one plugin required");
    sdf::Plugin plugin;const auto pluginErrors=plugin.Load(declaration);
    if(!pluginErrors.empty())throw std::runtime_error("invalid plugin declaration");
    r["filename"]=plugin.Filename();r["requested_class"]=plugin.Name();
    ignition::gazebo::SystemLoader loader;
    const auto instance=loader.LoadPlugin(plugin);
    r["systemloader_report"]=loader.PrettyStr();
    r["loaded"]=instance.has_value() && static_cast<bool>(*instance);
    ignition::plugin::Loader inspection;
    const auto registered=inspection.LoadLib(plugin.Filename());
    r["registered_classes"]=Json::Value(Json::arrayValue);
    for(const auto &name:registered) {
      r["registered_classes"].append(name);
      for(const auto &alias:inspection.AliasesOfPlugin(name))r["aliases"][name].append(alias);
    }
    r["registration_report"]=inspection.PrettyStr();
    if(!r["loaded"].asBool())throw std::runtime_error("SystemLoader::LoadPlugin returned no instance");
    auto &p=*instance;
    r["instance_class"]=p->Name()?*p->Name():"";
    r["system"]=p->QueryInterface<ignition::gazebo::System>()!=nullptr;
    r["configure"]=p->QueryInterface<ignition::gazebo::ISystemConfigure>()!=nullptr;
    r["update"]=p->QueryInterface<ignition::gazebo::ISystemUpdate>()!=nullptr;
    r["pre_update"]=p->QueryInterface<ignition::gazebo::ISystemPreUpdate>()!=nullptr;
    r["post_update"]=p->QueryInterface<ignition::gazebo::ISystemPostUpdate>()!=nullptr;
    const std::string canonical="ignition::gazebo::v6::systems::WorkcellOwnerPhysics";
    ok=registered.size()==1 && registered.count(canonical) && r["instance_class"]==canonical &&
        plugin.Name()=="ignition::gazebo::systems::WorkcellOwnerPhysics" &&
        inspection.AliasesOfPlugin(canonical)==std::set<std::string>{"gz::sim::systems::WorkcellOwnerPhysics","ignition::gazebo::systems::WorkcellOwnerPhysics"} &&
        r["system"].asBool() && r["configure"].asBool() && r["update"].asBool()
        && !r["pre_update"].asBool() && !r["post_update"].asBool();
    if(argc==4) {
      const auto foreign=inspection.LoadLib(argv[3]);
      for(const auto &name:foreign)r["preloaded_registration_classes"].append(name);
      if(foreign.empty() || foreign.count(canonical))ok=false;
    }
    // Witness actual mapped files while the System instance is alive.
    std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> paths;
    while(std::getline(maps,line)){auto pos=line.find('/');if(pos!=std::string::npos&&line.find(".so",pos)!=std::string::npos)paths.insert(line.substr(pos));}
    for(const auto &path:paths) {
      r["loaded_library_paths"].append(path);
      if(path.find("_dri.so")!=std::string::npos ||
         path.find("libignition-gazebo-physics-system.so")!=std::string::npos)ok=false;
    }
    r["gpu_device_fds"]=Json::Value(Json::arrayValue);
    for(const auto &fd:std::filesystem::directory_iterator("/proc/self/fd")) {
      std::error_code error;const auto target=std::filesystem::read_symlink(fd.path(),error).string();
      if(!error && target.find("/dev/dri/")==0){r["gpu_device_fds"].append(target);ok=false;}
    }
  }catch(const std::exception &e){ok=false;r["failure_reason"]=e.what();}
  r["decision"]=ok?"PASS_SYSTEMLOADER_ONLY":"BLOCKED";
  std::ofstream out(argv[2]);out<<r<<'\n';std::cout<<r<<std::endl;return ok&&out?0:2;
}
