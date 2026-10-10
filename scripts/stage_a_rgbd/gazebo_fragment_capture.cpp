// Disposable Gazebo render owner. No ROS/EPD/planning/controller interfaces.
#include "fragment_mrt.hh"
#include "gl_context_witness.hh"
#include "renderer_identity.hh"
#include "physics_owner/identity_inventory.hh"
#include "inventory_ecm_diagnostics.hh"
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/rendering/ogre2/Ogre2Geometry.hh>
#include <ignition/rendering/ogre2/Ogre2Visual.hh>
#include <ignition/rendering/Material.hh>
#include <OgreItem.h>
#include <OgreSubItem.h>
#include <OgreRay.h>
#include <OgreAxisAlignedBox.h>
#include "material_witness.hh"
#include <thread>

namespace g=ignition::gazebo;
namespace rd=ignition::rendering;
// Flushed stderr survives failure before the final JSON report.
void LiveMark(const char *boundary) {
  std::cerr<<"[workcell-lifecycle] "<<boundary<<std::endl;
}
void LiveMaps(const std::string &path) {
  std::ifstream in("/proc/self/maps");std::ofstream out(path+".maps");out<<in.rdbuf();out.flush();
}
const char *liveFs=R"(#version 330 core
uniform vec3 baseColour;
uniform uint entityId;
layout(location=0) out vec4 rgb;
layout(location=1) out uint identity;
void main(){ rgb=vec4(baseColour,1.0);identity=entityId; }
)";

struct LiveCapture final:g::System,g::ISystemPostUpdate {
  g::RenderUtil render;
  std::string ownerPath,path,session;
  Json::Value record;
  bool initialized=false,done=false,ok=false;
  unsigned epoch=0;
  std::thread::id renderThread;
  LiveCapture(std::string owner,std::string output,std::string run):ownerPath(owner),path(output),session(run) {
    record["session"]=session;record["schema"]="workcell_live_fragment/v1";
    record["contact_authority"]=false;record["execution_goals"]=0;
    record["timing_authority"]="BLOCKED";record["render_owner"]="GazeboRenderUtil";
    record["rgb_profile"]="opaque_unlit_diffuse_MRT";
    record["stock_rgb_equivalence"]="NOT_CLAIMED_DISPOSABLE_UNLIT_RGB_PATH";
    record["geometry_source"]="SceneManager::VisualById::GeometryByIndex::OgreObject";
  }
  void PostUpdate(const g::UpdateInfo &info,const g::EntityComponentManager &ecm) override {
    if(done)return;
    try {
      LiveMark("physics_postupdate_enter");
      LiveMaps(path);
      if(std::filesystem::exists(ownerPath))LiveMark("owner_trace_exists_after_physics_update");
      if(!initialized) {
        renderThread=std::this_thread::get_id();
        LiveMark("renderutil_init_begin");
        render.SetEngineName("ogre2");render.SetSceneName("live_fragment_"+session);
        render.SetHeadlessRendering(false);render.SetEnableSensors(false);render.Init();initialized=true;LiveMark("renderutil_init_complete");
      }
      if(renderThread!=std::this_thread::get_id())throw std::runtime_error("render owner thread changed");
      LiveMark("renderutil_update_begin");
      render.UpdateFromECM(info,ecm);render.Update();LiveMark("renderutil_update_complete");
      Json::Value event;event["epoch"]=++epoch;event["step"]=Json::UInt64(info.iterations);
      event["stamp_ns"]=Json::Int64(info.simTime.count());
      event["renderutil_stamp_ns"]=Json::Int64(render.SimTime().count());
      event["boundary"]="PostUpdate_after_UpdateFromECM_and_Update";
      record["scene_updates"].append(event);
      if(info.iterations!=2)return;
      done=true;
      Json::Value owner;std::ifstream input(ownerPath);std::string line;
      while(std::getline(input,line)) {Json::Value candidate;std::istringstream stream(line);stream>>candidate;
        if(candidate["step"].asUInt64()==info.iterations) {
          if(!owner.isNull())throw std::runtime_error("duplicate owner step");owner=candidate;
        }
      }
      if(owner.isNull() || !owner["complete"].asBool() || owner["session"]!=session ||
         owner["stamp_ns"].asInt64()!=info.simTime.count())throw std::runtime_error("missing/stale owner record");
      LiveMark("physics_configure_and_step_observed_in_valid_owner_trace");
      LiveMark("visual_inventory_join_begin");
      const auto inventory=workcell::IdentityInventory(ecm);
      // Preserve both complete snapshots before validation can reject the frame.
      record["owner_inventory"]=owner["identity_inventory"];record["render_inventory"]=inventory;
      record["inventory_ecm_diagnostics"]=workcell::InventoryEcmDiagnostics(ecm,owner["identity_inventory"],inventory);
      record["inventory_raw_json_equal"]=inventory==owner["identity_inventory"];
      record["inventory_comparison"]=workcell::CompareInventories(owner["identity_inventory"],inventory);
      record["owner_inventory_equal"]=record["inventory_comparison"]["equivalent"];
      if(!record["owner_inventory_equal"].asBool())throw std::runtime_error("owner/render ECM inventory changed");
      if(!record["inventory_comparison"]["schema_valid"].asBool() ||
         !record["inventory_comparison"]["supported_geometry"].asBool())
        throw std::runtime_error("unsupported complete inventory; see inventory_comparison/issues");
      // Fingerprint is calculated over retained records by the strict CPU validator.
      auto mapping=workcell::RendererIdentity(render.SceneManager(),owner,"");
      record["renderer"]=mapping; // Retain the incomplete map BEFORE rejection.
      if(!mapping["complete"].asBool())throw std::runtime_error("incomplete live SceneManager map: "+mapping["first_failure"]["condition"].asString());
      LiveMark("visual_inventory_join_complete");
      auto scene=render.Scene();auto native=std::dynamic_pointer_cast<rd::Ogre2Scene>(scene);
      if(!native)throw std::runtime_error("unsupported native renderer");auto sm=native->OgreSceneManager();
      record["gl_context"]["before_reacquire"]=GlContextWitness();
      record["gl_context"]["same_render_thread"]=renderThread==std::this_thread::get_id();
      auto rs=Ogre::Root::getSingleton().getRenderSystem();
      if(!rs || rs->getName()!="OpenGL 3+ Rendering Subsystem")throw std::runtime_error("unsupported Ogre GL context owner");
      record["gl_context"]["render_system"]=rs->getName();
      // Public Ogre lifecycle API reacquires its EXISTING current context.
      // RenderUtil initialization and capture are on the same checked thread.
      record["gl_context"]["activation_api"]="Ogre::RenderSystem::postExtraThreadsStarted";
      rs->postExtraThreadsStarted();
      record["gl_context"]["after_reacquire"]=GlContextWitness();
      // Flush diagnostics before this or any subsequent MRT gate can reject.
      {std::ofstream diagnostics(path);diagnostics<<record<<'\n';diagnostics.flush();}
      const auto glState=record["gl_context"]["after_reacquire"]["classification"].asString();
      if(glState!="PASS_CURRENT_GL45")throw std::runtime_error("GL4.5 typed clear API required: "+glState);
      const auto world=inventory["worlds"];
      if(world.size()!=1 || !workcell::SameEntityId(world[0]["id"],owner["world_entity"]))throw std::runtime_error("world identity mismatch");
      // Controlled profile: no subset silently selected from a larger world.
      if(inventory["visuals"].size()!=2 || inventory["collisions"].size()!=2 ||
         inventory["links"].size()!=2 || inventory["models"].size()!=2 || owner["shapes"].size()!=2)
        throw std::runtime_error("unsupported/partial two-cube inventory");
      // Retain BOTH original materials before any material gate or replacement.
      std::map<Json::UInt64,Json::Value> materials;
      for(const auto &entry:mapping["visuals"]) {
        const auto visualId=entry["visual_id"].asUInt64();
        const auto evidence=MaterialWitness(ecm,visualId,render.SceneManager().VisualById(visualId));
        materials.emplace(visualId,evidence);record["materials"].append(evidence);
      }
      {std::ofstream diagnostics(path);diagnostics<<record<<'\n';diagnostics.flush();}
      auto &pm=Ogre::HighLevelGpuProgramManager::getSingleton();
      for(auto stage:{Ogre::GPT_VERTEX_PROGRAM,Ogre::GPT_FRAGMENT_PROGRAM}) {
        auto program=pm.createProgram(stage==Ogre::GPT_VERTEX_PROGRAM?"live_vs":"live_fs","General","glsl",stage);
        program->setSource(stage==Ogre::GPT_VERTEX_PROGRAM?vs:liveFs);program->load();
        if(program->hasCompileError())throw std::runtime_error("live shader compile error");
      }
      record["vertex_shader"]=vs;record["fragment_shader"]=liveFs;
      std::set<Ogre::Item*> items;std::set<unsigned> ids;std::map<Ogre::Item*,unsigned> itemIds;
      std::vector<std::pair<Ogre::Item*,Ogre::MaterialPtr>> bound;
      for(const auto &entry:mapping["visuals"]) {
        const auto visualId=entry["visual_id"].asUInt64(),linkId=entry["link_id"].asUInt64();
        Json::Value visualRow,collision,shape;unsigned collisions=0,visuals=0,shapes=0;
        for(const auto &v:inventory["visuals"])if(v["parent"].asUInt64()==linkId){visualRow=v;++visuals;}
        for(const auto &c:inventory["collisions"])if(c["parent"].asUInt64()==linkId){collision=c;++collisions;}
        for(const auto &s:owner["shapes"])if(workcell::SameEntityId(s["collision_id"],collision["id"])){shape=s;++shapes;}
        if(visuals!=1 || collisions!=1 || shapes!=1 || !visualRow["opaque"].asBool() ||
           visualRow["geometry"]["type"]!="BOX" || collision["geometry"]["type"]!="BOX" ||
           shape["shape_type"]!="BoxShape" || shape["shape_node_identity"].asString().empty())
          throw std::runtime_error("ambiguous/unsupported collision ShapeNode ownership");
        auto visual=render.SceneManager().VisualById(visualId);
        if(!visual || visual->GeometryCount()!=1)throw std::runtime_error("unsupported visual geometry inventory");
        auto geometry=std::dynamic_pointer_cast<rd::Ogre2Geometry>(visual->GeometryByIndex(0));
        auto node=std::dynamic_pointer_cast<rd::Ogre2Visual>(visual);
        auto item=geometry?dynamic_cast<Ogre::Item*>(geometry->OgreObject()):nullptr;
        if(!item || !node || geometry->Parent()!=visual || item->getParentSceneNode()!=node->Node() ||
           !items.insert(item).second || item->getNumSubItems()!=1)
          throw std::runtime_error("missing/duplicate/unattached live Ogre Item");
        const unsigned id=visual->Id();
        if(id==0 || id==4294967295u || !ids.insert(id).second)throw std::runtime_error("duplicate/invalid native uint32 ID");
        itemIds.emplace(item,id);
        const auto &material=materials.at(visualId);
        if(!material["failure"].asString().empty())throw std::runtime_error("unsupported native material: "+material["failure"].asString());
        // SceneManager sets Geometry::Material; Visual::Material may be null.
        // Use actual live ECM diffuse only after exact native/ECM opacity and colour checks.
        const auto &colour=material["diffuse"];
        // Same live Ogre Item and geometry; only this disposable unlit material changes.
        auto mat=Ogre::MaterialManager::getSingleton().create("live_mrt_"+std::to_string(id),"General");
        auto pass=mat->getTechnique(0)->getPass(0);pass->setVertexProgram("live_vs");pass->setFragmentProgram("live_fs");
        pass->getVertexProgramParameters()->setNamedAutoConstant("worldViewProj",Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        pass->getFragmentProgramParameters()->setNamedConstant("baseColour",Ogre::Vector3(colour[0].asDouble(),colour[1].asDouble(),colour[2].asDouble()));
        pass->getFragmentProgramParameters()->setNamedConstant("entityId",id);
        Ogre::HlmsMacroblock macro;macro.mDepthCheck=true;macro.mDepthWrite=true;pass->setMacroblock(macro);
        Ogre::HlmsBlendblock blend;blend.mSourceBlendFactor=Ogre::SBF_ONE;blend.mDestBlendFactor=Ogre::SBF_ZERO;pass->setBlendblock(blend);
        mat->load();item->setMaterial(mat);bound.emplace_back(item,mat);
        Json::Value b=entry;b["collision_id"]=collision["id"];b["physics_shape_id"]=shape["physics_shape_id"];
        b["shape_node_identity"]=shape["shape_node_identity"];b["item_id"]=Json::UInt64(item->getId());
        b["geometry_id"]=visual->GeometryByIndex(0)->Id();b["original_item_reused"]=true;
        record["draw_bindings"].append(b);
        Json::Value v;v["id"]=id;v["native_entity"]=std::to_string(item->getId());record["inventory"].append(v);
      }
      auto actual=sm->getMovableObjectIterator("Item");unsigned count=0;
      while(actual.hasMoreElements()){++count;if(!items.count(dynamic_cast<Ogre::Item*>(actual.getNext())))throw std::runtime_error("unexpected renderer Item");}
      if(count!=items.size())throw std::runtime_error("incomplete Item inventory");
      if(sm->getMovableObjectIterator("Entity").hasMoreElements())throw std::runtime_error("unexpected legacy renderer geometry");
      record["inventory_complete"]=true;
      auto camera=sm->createCamera("live_mrt_camera");
      if(camera->getParentSceneNode()!=sm->getRootSceneNode(Ogre::SCENE_DYNAMIC))throw std::runtime_error("unexpected camera attachment");
      // Optical fixture only: separated original cubes overlap in this oblique view.
      // Authored camera constants do not establish object ownership or planning poses.
      camera->setPosition(.20,-.217,.04);camera->lookAt(.35,-.217,.0125);
      record["camera_profile"]="authored_oblique_overlap_fixture";camera->setNearClipDistance(.01);camera->setFarClipDistance(2);
      camera->setAspectRatio(1);camera->setFOVy(Ogre::Radian(1.0471975511965976));
      record["camera_id"]=Json::UInt64(camera->getId());
      Json::Value acquisition;acquisition["session"]=session;acquisition["world_entity"]=world[0]["id"];
      acquisition["step"]=Json::UInt64(info.iterations);acquisition["stamp_ns"]=Json::Int64(info.simTime.count());
      acquisition["scene_id"]=scene->Id();acquisition["frame"]=1;acquisition["update_epoch"]=epoch;
      acquisition["phase"]="PostUpdate_blocking_capture_no_subsequent_server_iteration";record["acquisition"]=acquisition;
      const auto labels=CaptureFragmentMrt(scene,sm,camera,record,path,LiveMark);
      for(const auto &[item,mat]:bound)if(item->getSubItem(0)->getMaterial()!=mat)throw std::runtime_error("material changed during acquisition");
      std::set<unsigned> visible;
      for(auto id:labels){visible.insert(id);const auto key=std::to_string(id);record["pixel_counts"][key]=record["pixel_counts"].get(key,0).asUInt()+1;}
      ids.insert(0);if(visible!=ids)throw std::runtime_error("missing/unexpected visible IDs");
      // Measured fixture occlusion diagnostic, not a renderer precision enclosure.
      // Use each actual native Item's local BOX bounds and full scene transform.
      // Select a pixel whose centre AND four corners intersect both objects with
      // the same near/far ordering. This is not an EPD mask or association rule.
      double best=1e9;Json::Value witness;
      for(unsigned y=0;y<256;++y)for(unsigned x=0;x<256;++x) {
        std::vector<unsigned> order;bool interior=true;
        for(const auto &uv:std::vector<std::pair<double,double>>{{.5,.5},{0,0},{1,0},{0,1},{1,1}}) {
          const auto ray=camera->getCameraToViewportRay((x+uv.first)/256.,(y+uv.second)/256.);
          std::vector<std::pair<double,unsigned>> hits;
          for(const auto &[item,mat]:bound) {
            const auto inv=item->getParentSceneNode()->_getFullTransform().inverse();
            const auto origin=inv*ray.getOrigin();
            const auto direction=(inv*(ray.getOrigin()+ray.getDirection()))-origin;
            const auto box=item->getLocalAabb();
            const auto hit=Ogre::Ray(origin,direction).intersects(Ogre::AxisAlignedBox(box.getMinimum(),box.getMaximum()));
            if(hit.first) {
              hits.emplace_back(hit.second,itemIds.at(item));
            }
          }
          std::sort(hits.begin(),hits.end());
          if(hits.size()!=2 || hits[0].first>=hits[1].first){interior=false;break;}
          if(order.empty())order={hits[0].second,hits[1].second};
          else if(order!=std::vector<unsigned>{hits[0].second,hits[1].second}){interior=false;break;}
        }
        const double distance=(x-127.5)*(x-127.5)+(y-127.5)*(y-127.5);
        if(interior && distance<best) {
          best=distance;witness["x"]=x;witness["y"]=y;witness["near_id"]=order[0];
          witness["far_id"]=order[1];witness["captured_id"]=labels[y*256+x];
        }
      }
      record["occlusion_witness"]=witness;
      record["occlusion_scope"]="tested_pixel_native_BOX_ray_diagnostic_not_numeric_registration_authority";
      if(witness.isNull() || witness["captured_id"]!=witness["near_id"])
        throw std::runtime_error("missing/incorrect native overlap occlusion witness");
      record["occlusion_verified"]=true;

      record["native_result"]="PASS_TESTED_VISUAL_DRAW_PRODUCTION";ok=true;
    }catch(const std::exception &e){done=true;ok=false;record["failure_reason"]=e.what();}
    if(done) {
      // Capture while the live Server/physics plugin still exists, before teardown.
      std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> paths;
      while(std::getline(maps,line)){auto pos=line.find('/');if(pos!=std::string::npos&&line.find(".so",pos)!=std::string::npos)paths.insert(line.substr(pos));}
      for(const auto &p:paths)record["loaded_library_paths"].append(p);
      record["loaded_library_witness_phase"]="live_PostUpdate_before_Server_destruction";
    }
  }
};

int main(int argc,char **argv) {
  // world, owner trace, output prefix, session; caller must use a fresh directory.
  if(argc!=5)return 2;
  const std::string path=argv[3];
  for(const auto &s:{"",".rgb8",".ids.u32"})if(std::filesystem::exists(path+s))return 2;
  auto capture=std::make_shared<LiveCapture>(argv[2],path,argv[4]);
  try {
    g::ServerConfig config;if(!config.SetSdfFile(argv[1]))throw std::runtime_error("invalid disposable SDF");
    LiveMark("server_construction_begin");
    g::Server server(config);LiveMark("server_construction_complete");LiveMaps(path);
    if(server.AddSystem(capture)!=std::optional<bool>(true))throw std::runtime_error("capture owner not installed");
    LiveMark("physics_configuration_and_updates_begin");
    if(!server.Run(true,2,false))throw std::runtime_error("bounded Gazebo run failed");
    LiveMark("server_run_complete");LiveMark("server_teardown_begin");
  }catch(const std::exception &e){capture->ok=false;capture->record["failure_reason"]=e.what();}
  LiveMark("server_scope_exit_complete");
  std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> loaded;
  while(std::getline(maps,line)){auto pos=line.find('/');if(pos!=std::string::npos&&line.find(".so",pos)!=std::string::npos)loaded.insert(line.substr(pos));}
  for(const auto &p:loaded)capture->record["loaded_library_paths"].append(p);
  std::ofstream output(path);output<<capture->record<<'\n';output.flush();LiveMark("native_report_flushed_before_capture_destruction");return capture->ok && output?0:2;
}
