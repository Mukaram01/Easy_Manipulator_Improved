#pragma once
#include "native_pass.hh"
#include "../renderer_identity.hh"
#include "../physics_owner/identity_inventory.hh"
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/gazebo/System.hh>
#include <OgreItem.h>
#include <OgreSubItem.h>

inline Json::Value MatrixWitness(const Ogre::Matrix4 &m) {
  Json::Value a(Json::arrayValue);
  for(unsigned y=0;y<4;++y)for(unsigned x=0;x<4;++x)a.append(m[y][x]);
  return a;
}
inline Json::Value FrozenWitness(ignition::gazebo::RenderUtil &render,
    const ignition::gazebo::EntityComponentManager &ecm,const ignition::gazebo::UpdateInfo &info,
    unsigned epoch,const Json::Value &owner,Ogre::SceneManager *sm,Ogre::Camera *camera,
    const Json::Value &record,const std::map<Ogre::Item*,unsigned> &items) {
  Json::Value r;r["step"]=Json::UInt64(info.iterations);r["stamp_ns"]=Json::Int64(info.simTime.count());
  r["renderutil_stamp_ns"]=Json::Int64(render.SimTime().count());r["update_epoch"]=epoch;
  r["scene_time_ns"]=Json::Int64(render.Scene()->Time().count());
  r["inventory"]=workcell::IdentityInventory(ecm);
  r["renderer"]=workcell::RendererIdentity(render.SceneManager(),owner,"");
  if(!r["renderer"]["complete"].asBool())throw std::runtime_error("paired identity join incomplete");
  r["joins"]=record["draw_bindings"];r["owner_shapes"]=owner["shapes"];
  r["camera"]=record["camera_configuration"];r["camera"]["aspect"]=camera->getAspectRatio();
  r["view"]=MatrixWitness(camera->getViewMatrix());r["projection"]=MatrixWitness(camera->getProjectionMatrix());
  r["projection_rs_depth"]=MatrixWitness(camera->getProjectionMatrixWithRSDepth());
  unsigned count=0;auto actual=sm->getMovableObjectIterator("Item");
  while(actual.hasMoreElements()) {auto item=dynamic_cast<Ogre::Item*>(actual.getNext());++count;
    if(!items.count(item) || item->getNumSubItems()!=1)throw std::runtime_error("paired draw inventory changed");
    Json::Value v;v["renderer_id"]=items.at(item);v["item_id"]=Json::UInt64(item->getId());
    v["world_matrix"]=MatrixWitness(item->getParentSceneNode()->_getFullTransform());
    const auto box=item->getLocalAabb();for(int i=0;i<3;++i){v["aabb_centre"].append(box.mCenter[i]);v["aabb_half_size"].append(box.mHalfSize[i]);}
    v["material"]=item->getSubItem(0)->getMaterial()->getName();
    r["transforms"].append(v);
  }
  if(count!=2 || items.size()!=2 || sm->getMovableObjectIterator("Entity").hasMoreElements())throw std::runtime_error("paired scene inventory changed");
  return r;
}
inline Json::Value PixelRays(Ogre::Camera *camera,int n) {
  Json::Value rays(Json::arrayValue);
  for(const auto &yx:std::vector<std::pair<int,int>>{{0,0},{n/2-1,n/2-1},{n-1,n-1},{105,37}}) {
    auto ray=camera->getCameraToViewportRay((yx.second+.5)/n,(yx.first+.5)/n);
    Json::Value r;r["yx"].append(yx.first);r["yx"].append(yx.second);
    for(int i=0;i<3;++i){r["origin"].append(ray.getOrigin()[i]);r["direction"].append(ray.getDirection()[i]);}
    rays.append(r);
  }
  return rays;
}
inline void RequireFrozen(const Json::Value &a,const Json::Value &b) {
  if(a!=b)throw std::runtime_error("scene/camera/acquisition changed between native passes");
}
inline std::vector<unsigned> CapturePairedFragmentMrt(const ignition::rendering::ScenePtr &scene,
    Ogre::SceneManager *sm,Ogre::Camera *camera,Json::Value &record,const std::string &path,
    void (*mark)(const char*),ignition::gazebo::RenderUtil &render,
    const ignition::gazebo::EntityComponentManager &ecm,const ignition::gazebo::UpdateInfo &info,
    const unsigned &epoch,const Json::Value &owner,const std::map<Ogre::Item*,unsigned> &items) {
  auto &r=record["paired_capture"];r["schema"]="workcell_paired_native/v1";
  r["acquisition_order"].append(256);r["acquisition_order"].append(512);r["separate_render_passes"]=2;
  r["freeze_boundary"]="after sole PreRender and updateSceneGraph; synchronous blocking PostUpdate; sole PostRender after both readbacks";
  // Scene preparation can flush caches/frame listeners. All of it precedes freeze.
  auto target256=PrepareNativePass(sm,camera,256);auto target512=PrepareNativePass(sm,camera,512);
  scene->PreRender();sm->updateSceneGraph();
  r["pixel_rays"].append(PixelRays(camera,256));r["pixel_rays"].append(PixelRays(camera,512));
  auto before=FrozenWitness(render,ecm,info,epoch,owner,sm,camera,record,items);r["frozen"].append(before);
  r["acquisition_wall_ns"].append(Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
  auto labels=CaptureNativePass(target256,record,path,256,mark);
  auto between=FrozenWitness(render,ecm,info,epoch,owner,sm,camera,record,items);r["frozen"].append(between);RequireFrozen(before,between);
  r["acquisition_wall_ns"].append(Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
  CaptureNativePass(target512,r["native512"],path+".native512",512,mark);
  auto after=FrozenWitness(render,ecm,info,epoch,owner,sm,camera,record,items);r["frozen"].append(after);RequireFrozen(before,after);
  r["acquisition_wall_ns"].append(Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
  r["no_scene_or_physics_update_between_acquisitions"]=true;
  scene->PostRender();
  auto cm=Ogre::Root::getSingleton().getCompositorManager2();cm->removeWorkspace(target256.workspace);cm->removeWorkspace(target512.workspace);return labels;
}
