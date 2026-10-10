#pragma once
// Shared explicit render batching for the native prerequisite and disposable System.
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/ogre2/Ogre2Node.hh>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
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
  std::vector<float> depths;std::vector<unsigned char> labels;
  unsigned depthFrames=0,segFrames=0;
  auto depthConnection=depth->ConnectNewDepthFrame([&](const float *data,unsigned w,unsigned h,unsigned c,const std::string &fmt) {
    r["depth_format"]=fmt;r["depth_channels"]=c;
    r["depth_width"]=w;r["depth_height"]=h;++depthFrames;
    depths.assign(data,data+w*h*c);
  });
  auto segConnection=seg->ConnectNewSegmentationFrame([&](const uint8_t *data,unsigned w,unsigned h,unsigned c,const std::string &fmt) {
    r["segmentation_format"]=fmt;r["segmentation_channels"]=c;
    r["segmentation_width"]=w;r["segmentation_height"]=h;++segFrames;
    labels.assign(data,data+w*h*c);
  });
  // Fixed scene, no clock or external writer. All cameras share each explicit
  // PreRender/Render/PostRender batch, never independently timed Update calls.
  for(int batch=0;batch<3;++batch) {
    scene->PreRender();
    for(auto &camera:cameras)camera->Render();
    for(auto &camera:cameras)camera->PostRender();
    scene->PostRender();
  }
  auto image=rgb->CreateImage();rgb->Copy(image);
  auto matrix=[](const ignition::math::Matrix4d &m) {
    Json::Value a(Json::arrayValue);
    for(int i=0;i<4;++i){Json::Value row(Json::arrayValue);for(int j=0;j<4;++j)row.append(m(i,j));a.append(row);}
    return a;
  };
  r["camera_matrices"]=Json::Value(Json::arrayValue);
  r["native_camera_matrices"]=Json::Value(Json::arrayValue);
  auto nativeMatrix=[](const Ogre::Matrix4 &m) {
    Json::Value a(Json::arrayValue);
    for(int i=0;i<4;++i){Json::Value row(Json::arrayValue);for(int j=0;j<4;++j)row.append(static_cast<double>(m[i][j]));a.append(row);}
    return a;
  };
  bool registration=true;

  for(auto &camera:cameras) {
    Json::Value config;config["projection"]=matrix(camera->ProjectionMatrix());
    config["view"]=matrix(camera->ViewMatrix());config["width"]=camera->ImageWidth();config["height"]=camera->ImageHeight();
    // Depth/segmentation generic getters reconstruct ideal double matrices.
    // Inspect actual Ogre cameras through the supported Ogre2Node API instead.
    auto node=std::dynamic_pointer_cast<rd::Ogre2Node>(camera);
    Ogre::Camera *actual=nullptr;unsigned attachedCameras=0;
    if(node && node->Node())for(std::size_t i=0;i<node->Node()->numAttachedObjects();++i)
      if(auto candidate=dynamic_cast<Ogre::Camera*>(node->Node()->getAttachedObject(i))) {
        actual=candidate;++attachedCameras;
      }
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
