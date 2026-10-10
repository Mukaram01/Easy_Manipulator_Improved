#pragma once
// Read-only camera callback witnesses for the static native prerequisite.
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/ogre2/Ogre2Node.hh>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreGpuProgram.h>
#include <functional>
#include <chrono>
#include <unistd.h>
#include <ignition/rendering/DepthCamera.hh>
#include <ignition/rendering/SegmentationCamera.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Image.hh>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <set>
namespace rd=ignition::rendering;

inline Json::Value nativeMatrix(const Ogre::Matrix4 &m) {
  Json::Value a(Json::arrayValue);
  for(int i=0;i<4;++i){Json::Value row(Json::arrayValue);for(int j=0;j<4;++j)row.append(static_cast<double>(m[i][j]));a.append(row);}
  return a;
}

inline Ogre::Camera *attachedCamera(const rd::CameraPtr &camera,unsigned &count) {
  auto node=std::dynamic_pointer_cast<rd::Ogre2Node>(camera);
  Ogre::Camera *actual=nullptr;count=0;
  if(node && node->Node())for(std::size_t i=0;i<node->Node()->numAttachedObjects();++i)
    if(auto candidate=dynamic_cast<Ogre::Camera*>(node->Node()->getAttachedObject(i))) {actual=candidate;++count;}
  return actual;
}

// Public material registry readback. This is CPU resource state, NOT evidence
// that a particular compositor quad bound these uniforms at GPU execution.
inline Json::Value depthMaterialState(const std::string &name) {
  Json::Value result;result["scope"]="CPU material registry; active GPU binding unknown";
  for(const auto &suffix:{"DepthCamera","DepthCameraFinal"}) {
    auto material=Ogre::MaterialManager::getSingleton().getByName(name+"_"+suffix);
    Json::Value m;m["present"]=!material.isNull();
    if(!material.isNull() && material->getNumTechniques()>0 && material->getTechnique(0)->getNumPasses()>0) {
      auto pass=material->getTechnique(0)->getPass(0);
      m["material"]=material->getName();m["origin"]=material->getOrigin();
      if(pass->hasFragmentProgram()) {
        auto program=pass->getFragmentProgram()->_getBindingDelegate();
        m["fragment_program"]=program->getName();m["source_file"]=program->getSourceFile();
        const auto parameters=pass->getFragmentProgramParameters();
        for(const auto &uniform:{"projectionParams","near","far","min","max"}) {
          auto definition=parameters->_findNamedConstantDefinition(uniform,false);
          if(!definition)continue;
          const auto data=static_cast<const Ogre::GpuProgramParameters &>(*parameters).getFloatPointer(definition->physicalIndex);
          Json::Value values(Json::arrayValue);
          for(unsigned i=0;i<(std::string(uniform)=="projectionParams"?2u:1u);++i) {
            if(std::isfinite(data[i]))values.append(static_cast<double>(data[i]));
            else values.append(data[i]>0?"+INF":"-INF");
          }
          m["uniforms"][uniform]=values;
        }
      }
    }
    result[suffix]=m;
  }
  return result;
}

struct ProbeCameraListener final:Ogre::Camera::Listener {
  Ogre::Camera *camera;
  std::function<void(Ogre::Camera*,const char*)> record;
  ProbeCameraListener(Ogre::Camera *c,std::function<void(Ogre::Camera*,const char*)> fn):camera(c),record(fn){camera->addListener(this);}
  ~ProbeCameraListener(){camera->removeListener(this);}
  void cameraPreRenderScene(Ogre::Camera *c) override {record(c,"scene_pre");}
  void cameraPostRenderScene(Ogre::Camera *c) override {record(c,"scene_post");}
};

inline bool images(const rd::ScenePtr &scene,const std::filesystem::path &output,Json::Value &r,
    const ignition::math::Pose3d &pose=ignition::math::Pose3d::Zero,
    double hfov=1.0471975511965976,bool fixture=true) {
  auto rgb=scene->CreateCamera("probe_rgb");
  auto depth=scene->CreateDepthCamera("probe_depth");
  auto seg=scene->CreateSegmentationCamera("probe_segmentation");
  r["rgb_camera_created"]=static_cast<bool>(rgb);
  r["depth_camera_created"]=static_cast<bool>(depth);
  r["segmentation_camera_created"]=static_cast<bool>(seg);
  if(!rgb || !depth || !seg) {r["reason"]="REQUIRED_CAMERA_UNSUPPORTED";return false;}
  scene->SetBackgroundColor(0,0,0);scene->SetAmbientLight(1,1,1);
  std::vector<rd::CameraPtr> cameras={rgb,depth,seg};
  for(auto &camera:cameras) {
    camera->SetImageWidth(512);camera->SetImageHeight(512);
    camera->SetImageFormat(rd::PF_R8G8B8);camera->SetAspectRatio(1.0);
    camera->SetHFOV(ignition::math::Angle(hfov));
    camera->SetNearClipPlane(.02);camera->SetFarClipPlane(5.0);
    camera->SetAntiAliasing(0);camera->SetLocalPosition(0,0,0);
    camera->SetWorldPose(pose);scene->RootVisual()->AddChild(camera);
  }
  seg->SetSegmentationType(rd::SegmentationType::ST_SEMANTIC);
  seg->EnableColoredMap(false);seg->SetBackgroundLabel(0);
  if(fixture)for(int index=0;index<2;++index) {
    auto visual=scene->CreateVisual("probe_box_"+std::to_string(index));
    visual->AddGeometry(scene->CreateBox());visual->SetLocalScale(.025,.025,.025);
    visual->SetLocalPosition(1.0,index==0?-.05:.05,0.0);
    visual->SetUserData("label",index==0?11:22);
    auto material=scene->CreateMaterial();
    material->SetDiffuse(index==0?1.0:0.0,index==1?1.0:0.0,0.0);
    material->SetEmissive(index==0?1.0:0.0,index==1?1.0:0.0,0.0);
    visual->SetMaterial(material);scene->RootVisual()->AddChild(visual);
  }
  r["native_session_id"]=std::to_string(getpid())+":"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  r["acquisition_events"]=Json::Value(Json::arrayValue);
  int activeBatch=0;
  auto event=[&](int camera,const char *phase,const Json::Value &state=Json::Value()) {
    Json::Value e;e["sequence"]=r["acquisition_events"].size();e["session"]=r["native_session_id"];
    e["batch"]=activeBatch;e["camera"]=camera;e["phase"]=phase;e["state"]=state;
    r["acquisition_events"].append(e);
  };
  std::vector<std::unique_ptr<ProbeCameraListener>> listeners;
  for(int index=0;index<3;++index) {
    unsigned count=0;auto actual=attachedCamera(cameras[index],count);
    if(count!=1 || !actual){r["reason"]="AMBIGUOUS_ATTACHED_CAMERA";return false;}
    listeners.emplace_back(new ProbeCameraListener(actual,[&,index](Ogre::Camera *c,const char *phase) {
      Json::Value state;unsigned inventory=0;auto attached=attachedCamera(cameras[index],inventory);
      state["attached_camera_count"]=inventory;state["attached_matches"]=attached==c;
      state["camera_id"]=Json::UInt64(c->getId());state["name"]=c->getName();
      state["projection"]=nativeMatrix(c->getProjectionMatrix());
      state["rs_projection"]=nativeMatrix(c->getProjectionMatrixWithRSDepth());
      state["view"]=nativeMatrix(c->getViewMatrix(true));
      state["near"]=static_cast<double>(c->getNearClipDistance());state["far"]=static_cast<double>(c->getFarClipDistance());
      auto ab=c->getProjectionParamsAB();state["projection_ab"]=Json::Value(Json::arrayValue);
      state["projection_ab"].append(static_cast<double>(ab.x));state["projection_ab"].append(static_cast<double>(ab.y));
      state["viewport_source"]="getLastViewport; not a GPU sample-position witness";
      auto vp=c->getLastViewport();
      if(vp){state["viewport"]=Json::Value(Json::arrayValue);for(auto v:{vp->getActualLeft(),vp->getActualTop(),vp->getActualWidth(),vp->getActualHeight()})state["viewport"].append(v);}
      else state["viewport"]=Json::Value();
      state["configured_width"]=cameras[index]->ImageWidth();state["configured_height"]=cameras[index]->ImageHeight();
      if(index==1)state["depth_conversion"]=depthMaterialState(depth->Name());
      event(index,phase,state);
    }));
  }
  std::vector<float> depths;std::vector<unsigned char> labels;
  unsigned depthFrames=0,segFrames=0;
  auto depthConnection=depth->ConnectNewDepthFrame([&](const float *data,unsigned w,unsigned h,unsigned c,const std::string &fmt) {
    r["depth_format"]=fmt;r["depth_channels"]=c;
    r["depth_width"]=w;r["depth_height"]=h;++depthFrames;
    depths.assign(data,data+w*h*c);event(1,"readback");
  });
  auto segConnection=seg->ConnectNewSegmentationFrame([&](const uint8_t *data,unsigned w,unsigned h,unsigned c,const std::string &fmt) {
    r["segmentation_format"]=fmt;r["segmentation_channels"]=c;
    r["segmentation_width"]=w;r["segmentation_height"]=h;++segFrames;
    labels.assign(data,data+w*h*c);event(2,"readback");
  });
  // Fixed scene, no clock or external writer. All cameras share each explicit
  // PreRender/Render/PostRender batch, never independently timed Update calls.
  auto image=rgb->CreateImage();
  for(int batch=1;batch<=3;++batch) {
    activeBatch=batch;scene->PreRender();event(-1,"scene_pre_render");
    for(int index=0;index<3;++index) {
      event(index,"render_begin");cameras[index]->Render();event(index,"render_end");
    }
    rgb->PostRender();rgb->Copy(image);event(0,"readback");
    depth->PostRender();seg->PostRender();
    scene->PostRender();event(-1,"scene_post_render");
  }
  listeners.clear();
  r["depth_conversion_status"]="CPU resource readback only; GPU compositor binding and arithmetic UNQUALIFIED";
  auto matrix=[](const ignition::math::Matrix4d &m) {
    Json::Value a(Json::arrayValue);
    for(int i=0;i<4;++i){Json::Value row(Json::arrayValue);for(int j=0;j<4;++j)row.append(m(i,j));a.append(row);}
    return a;
  };
  r["camera_matrices"]=Json::Value(Json::arrayValue);
  r["native_camera_matrices"]=Json::Value(Json::arrayValue);
  bool registration=true;

  for(auto &camera:cameras) {
    Json::Value config;config["projection"]=matrix(camera->ProjectionMatrix());
    config["view"]=matrix(camera->ViewMatrix());config["width"]=camera->ImageWidth();config["height"]=camera->ImageHeight();
    // Depth/segmentation generic getters reconstruct ideal double matrices.
    // Inspect actual Ogre cameras through the supported Ogre2Node API instead.
    unsigned attachedCameras=0;auto actual=attachedCamera(camera,attachedCameras);
    Json::Value native;
    native["attached_camera_count"]=attachedCameras;
    if(actual) {
      native["projection"]=nativeMatrix(actual->getProjectionMatrix());
      native["view"]=nativeMatrix(actual->getViewMatrix(true));
    }
    registration &= attachedCameras==1 && camera->ImageWidth()==512 && camera->ImageHeight()==512;
    if(!r["native_camera_matrices"].empty())registration &= native==r["native_camera_matrices"][0];
    r["native_camera_matrices"].append(native);
    r["camera_matrices"].append(config);
  }
  r["matching_native_camera_matrices"]=registration;
  r["pixel_registration"]=registration?"UNQUALIFIED_RASTERIZATION":"BLOCKED_NATIVE_CAMERA_PROJECTION_DIFFERENCE";
  const auto pixels=image.Data<unsigned char>();
  const auto count=512*512*3;
  auto limits=std::minmax_element(pixels,pixels+count);
  r["rgb_nonblank"]=*limits.first!=*limits.second;
  r["width"]=rgb->ImageWidth();r["height"]=rgb->ImageHeight();
  r["hfov_rad"]=rgb->HFOV().Radian();r["near_m"]=.02;r["far_m"]=5.0;
  r["camera_pose"]=Json::Value(Json::arrayValue);
  for(double v:{0.,0.,0.,0.,0.,0.})r["camera_pose"].append(v);
  r["fixture_scope"]=fixture?"two static 25mm visual BOXes at (1,+/-0.05,0), no physics":"loaded Gazebo render scene; physical alignment unqualified";
  r["shared_render_batches"]=3;r["depth_frames"]=depthFrames;r["segmentation_frames"]=segFrames;
  std::set<int> ids;unsigned finite=0;
  for(float value:depths)if(std::isfinite(value) && value>=.02f && value<=5.f)++finite;
  if(labels.size()==512*512*3)for(std::size_t i=0;i<labels.size();i+=3)if(labels[i])ids.insert(labels[i]);
  r["finite_depth_pixels"]=finite;r["segmentation_labels"]=Json::Value(Json::arrayValue);
  for(int id:ids)r["segmentation_labels"].append(id);
  auto save=[&](const std::string &suffix,const char *data,std::size_t size) {
    std::ofstream stream(output.string()+suffix,std::ios::binary);stream.write(data,size);
    return static_cast<bool>(stream);
  };
  bool written=save(".rgb8",reinterpret_cast<const char*>(pixels),count) &&
      save(".depth.f32",reinterpret_cast<const char*>(depths.data()),depths.size()*sizeof(float)) &&
      save(".labels.rgb8",reinterpret_cast<const char*>(labels.data()),labels.size());
  bool valid=written && r["rgb_nonblank"].asBool() && finite>0 && ids==std::set<int>{11,22} &&
      depths.size()==512*512 && labels.size()==512*512*3 && depthFrames==3 && segFrames==3;
  r["image_production"]=valid?"PASS":"BLOCKED";
  r["reason"]=valid?"STATIC_IMAGE_PRODUCTION_ONLY_ALIGNMENT_UNQUALIFIED":"INVALID_RGB_DEPTH_OR_SEGMENTATION";
  return valid;
}
