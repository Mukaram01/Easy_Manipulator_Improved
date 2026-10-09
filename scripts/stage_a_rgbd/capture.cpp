// Read-only Fortress transport adapter to the external EPD C++ inference API.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/camera_info.pb.h>
#include <ignition/msgs/world_stats.pb.h>
#include <ignition/msgs/scene.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/sdf_generator_config.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <tinyxml2.h>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include "ort_cpp_lib/p3_ort_base.hpp"
#include "geometry.hpp"

namespace fs=std::filesystem;
int64_t ns(const ignition::msgs::Time & t) {return t.sec()*1000000000LL+t.nsec();}
std::string frame(const ignition::msgs::Header & h) {
  for (const auto & d:h.data()) if (d.key()=="frame_id" && d.value_size()) return d.value(0);
  return "";
}
bool identity(const ignition::msgs::Pose & pose) {
  const auto & p=pose.position();const auto & q=pose.orientation();
  return std::abs(p.x())<1e-9 && std::abs(p.y())<1e-9 && std::abs(p.z())<1e-9 &&
    std::abs(q.x())<1e-9 && std::abs(q.y())<1e-9 && std::abs(q.z())<1e-9 && std::abs(q.w()-1)<1e-9;
}
Json::Value array(std::initializer_list<double> values) {
  Json::Value a(Json::arrayValue); for(auto v:values) a.append(v); return a;
}
void write(const fs::path & path,const Json::Value & value) {
  std::ofstream out(path); if(!out) throw std::runtime_error("cannot write "+path.string());
  out<<value;
}
int main(int argc,char ** argv) try {
  if(argc!=5) throw std::runtime_error("Usage: stage_a_rgbd_capture MODEL LABELS NEW_OUTPUT_DIR WORLD_NAME");
  const fs::path output(argv[3]);
  if(!fs::create_directory(output)) throw std::runtime_error("output must be a new directory");
  std::ifstream label_file(argv[2]); std::vector<std::string> labels; std::string label;
  while(std::getline(label_file,label)) if(!label.empty()) labels.push_back(label);
  if(labels.empty() || !fs::is_regular_file(argv[1])) throw std::runtime_error("model/labels unavailable");
  std::mutex mutex; std::condition_variable changed;
  ignition::msgs::Image rgb,depth; ignition::msgs::CameraInfo info;
  int64_t now=0, acquisition_clock=0; bool frozen=false;
  ignition::transport::Node node;
  std::function<void(const ignition::msgs::Image &)> on_rgb=[&](const auto & msg) {
    std::lock_guard<std::mutex> lock(mutex);if(frozen) return;rgb=msg;changed.notify_all();};
  std::function<void(const ignition::msgs::Image &)> on_depth=[&](const auto & msg) {
    std::lock_guard<std::mutex> lock(mutex);if(frozen) return;depth=msg;changed.notify_all();};
  std::function<void(const ignition::msgs::CameraInfo &)> on_info=[&](const auto & msg) {
    std::lock_guard<std::mutex> lock(mutex);if(frozen) return;info=msg;changed.notify_all();};
  std::function<void(const ignition::msgs::WorldStatistics &)> on_stats=[&](const auto & msg) {
    std::lock_guard<std::mutex> lock(mutex);now=ns(msg.sim_time());changed.notify_all();};
  if(!node.Subscribe("/stage_a/camera/image",on_rgb) ||
     !node.Subscribe("/stage_a/camera/depth_image",on_depth) ||
     !node.Subscribe("/stage_a/camera/camera_info",on_info) ||
     !node.Subscribe(std::string("/world/")+argv[4]+"/stats",on_stats)) throw std::runtime_error("subscribe failed");
  {
    std::unique_lock<std::mutex> lock(mutex);
    if(!changed.wait_for(lock,std::chrono::seconds(30),[&]{
      if(depth.data().size()!=512*512*4) return false;
      int finite=0;
      for(size_t i=0;i<depth.data().size();i+=4) {
        float z;std::memcpy(&z,depth.data().data()+i,4);
        if(std::isfinite(z) && z>=0.02f && z<=5.f) ++finite;
      }
      if(finite<12) return false;
      return stage_a::fresh(ns(rgb.header().stamp()),ns(depth.header().stamp()),ns(info.header().stamp()),now);
    })) {
      std::cerr<<"last stamps rgb/depth/info/clock: "<<ns(rgb.header().stamp())<<"/"
        <<ns(depth.header().stamp())<<"/"<<ns(info.header().stamp())<<"/"<<now<<std::endl;
      if(rgb.data().size()==512*512*3) {
        cv::Mat last(512,512,CV_8UC3,rgb.mutable_data()->data()),bgr;
        cv::cvtColor(last,bgr,cv::COLOR_RGB2BGR);cv::imwrite((output/"rejected_rgb.png").string(),bgr);
      }
      if(depth.data().size()==512*512*4) {
        std::ofstream raw(output/"rejected_depth.f32",std::ios::binary);
        raw.write(depth.data().data(),depth.data().size());
      }
      throw std::runtime_error("BLOCKED: no fresh synchronized valid RGB/depth/calibration and simulation clock within 30s");
    }
    acquisition_clock=now;frozen=true;
  }
  node.Unsubscribe("/stage_a/camera/image");node.Unsubscribe("/stage_a/camera/depth_image");
  node.Unsubscribe("/stage_a/camera/camera_info");
  const std::string optical="stage_a_camera_optical_frame";
  if(frame(rgb.header())!=optical || frame(depth.header())!=optical || frame(info.header())!=optical)
    throw std::runtime_error("optical frame mismatch");
  if(rgb.width()!=512 || rgb.height()!=512 || depth.width()!=rgb.width() || depth.height()!=rgb.height() ||
     info.width()!=rgb.width() || info.height()!=rgb.height() ||
     rgb.pixel_format_type()!=ignition::msgs::RGB_INT8 || depth.pixel_format_type()!=ignition::msgs::R_FLOAT32 ||
     rgb.step()!=512*3 || depth.step()!=512*4 || rgb.data().size()!=512*512*3 || depth.data().size()!=512*512*4 ||
     info.intrinsics().k_size()!=9) throw std::runtime_error("invalid aligned 512x512 RGB8/32FC1 geometry");
  double fx=info.intrinsics().k(0),fy=info.intrinsics().k(4),cx=info.intrinsics().k(2),cy=info.intrinsics().k(5);
  if(!EPD::validIntrinsics(fx,fy,cx,cy)) throw std::runtime_error("missing calibration");
  for(double d:info.distortion().k()) if(!std::isfinite(d) || d!=0) throw std::runtime_error("distorted camera unsupported");
  cv::Mat image(512,512,CV_8UC3,rgb.mutable_data()->data());
  cv::Mat depth_m(512,512,CV_32FC1,depth.mutable_data()->data());
  cv::Mat bgr;cv::cvtColor(image,bgr,cv::COLOR_RGB2BGR);
  cv::imwrite((output/"rgb.png").string(),bgr);
  {std::ofstream raw(output/"depth.f32",std::ios::binary);raw.write(depth.data().data(),depth.data().size());}
  Json::Value evidence;
  evidence["rgb_stamp_ns"]=Json::Int64(ns(rgb.header().stamp()));
  evidence["depth_stamp_ns"]=Json::Int64(ns(depth.header().stamp()));
  evidence["info_stamp_ns"]=Json::Int64(ns(info.header().stamp()));
  evidence["acquisition_clock_ns"]=Json::Int64(acquisition_clock);
  evidence["depth_encoding"]="32FC1";evidence["depth_units"]="metres";
  evidence["width"]=512;evidence["height"]=512;evidence["frame_id"]=optical;
  evidence["intrinsics"]=array({fx,fy,cx,cy});
  // Independent static-camera pose readback; never read cube poses for inference.
  ignition::msgs::Empty request;ignition::msgs::Scene scene;bool result=false;
  if(node.Request(std::string("/world/")+argv[4]+"/scene/info",request,5000,scene,result) && result) {
    for(const auto & model:scene.model()) if(model.name()=="stage_a_camera") {
      if(model.link_size()!=1 || !identity(model.link(0).pose()) || model.link(0).sensor_size()!=1 ||
         !identity(model.link(0).sensor(0).pose())) continue;
      auto p=model.pose();
      evidence["camera_world_pose"]=array({p.position().x(),p.position().y(),p.position().z(),
        p.orientation().x(),p.orientation().y(),p.orientation().z(),p.orientation().w()});
      evidence["camera_pose_source"]="live_scene_info";
      evidence["camera_is_static"]=model.is_static();
    }
  }
  ignition::msgs::SdfGeneratorConfig config;ignition::msgs::StringMsg live_sdf;
  if(node.Request(std::string("/world/")+argv[4]+"/generate_world_sdf",config,5000,live_sdf,result) && result) {
    tinyxml2::XMLDocument document;
    if(document.Parse(live_sdf.data().c_str())==tinyxml2::XML_SUCCESS) {
      auto sdf=document.FirstChildElement("sdf");auto world=sdf?sdf->FirstChildElement("world"):nullptr;
      for(auto model=world?world->FirstChildElement("model"):nullptr;model;model=model->NextSiblingElement("model")) {
        if(!model->Attribute("name") || std::string(model->Attribute("name"))!="stage_a_camera") continue;
        auto stat=model->FirstChildElement("static");
        evidence["camera_is_static"]=stat && stat->GetText() && std::string(stat->GetText())=="true";
        evidence["camera_definition_source"]="live_generate_world_sdf";
      }
    }
  }
  Ort::P3OrtBase epd(1.f,512,512,512,512,labels.size(),argv[1],boost::none,2,1,
    Ort::SessionExecutionMode::SEQUENTIAL,boost::none,Ort::InputTensorLayout::CHW,false,true,true);
  epd.initClassNames(labels);
  const auto detections=epd.infer(bgr);
  Json::Value snapshot;
  snapshot["schema_version"]="workcell_perception_snapshot/v1";
  snapshot["scene_id"]="ur5_2f_test";snapshot["camera_id"]="stage_a_camera";
  snapshot["timestamp"]=Json::Int64(ns(rgb.header().stamp()));snapshot["frame_id"]=optical;
  snapshot["objects"]=Json::Value(Json::arrayValue);
  int confident=0,rejected=0;
  for(size_t i=0;i<detections.scores.size();++i) {
    if(!std::isfinite(detections.scores[i]) || detections.scores[i]<0.8f || detections.scores[i]>1) continue;
    ++confident;
    const auto & box=detections.bboxes.at(i);const auto & roi_mask=detections.masks.at(i);
    if(box[0]<0 || box[1]<0 || box[2]>512 || box[3]>512 || box[2]<=box[0] || box[3]<=box[1] ||
       roi_mask.cols!=box[2]-box[0] || roi_mask.rows!=box[3]-box[1]) throw std::runtime_error("EPD ROI mask geometry mismatch");
    cv::Mat mask=cv::Mat::zeros(512,512,CV_8UC1);
    cv::Mat binary=roi_mask>0.5f;
    binary.copyTo(mask(cv::Rect(box[0],box[1],box[2]-box[0],box[3]-box[1])));
    cv::imwrite((output/("mask_"+std::to_string(i)+".png")).string(),mask);
    EPD::LocalizedObject object;
    if(!stage_a::localize(object,mask,depth_m,fx,fy,cx,cy)) {++rejected;continue;}
    auto class_index=detections.classIndices.at(i);
    if(class_index>=labels.size()) throw std::runtime_error("EPD label index invalid");
    Json::Value item;
    // Detection IDs are capture-scoped; this path makes no temporal tracking claim.
    item["object_id"]="epd_"+std::to_string(ns(rgb.header().stamp()))+"_"+std::to_string(i);
    item["label"]=labels[class_index];item["confidence"]=detections.scores[i];
    item["centroid"]=array({object.centroid.x,object.centroid.y,object.centroid.z});
    item["attributes"]["position_semantics"]="visible_surface_centroid";
    item["attributes"]["valid_depth_pixels"]=Json::UInt64(object.valid_depth_pixel_count);
    item["attributes"]["mask_pixels"]=cv::countNonZero(mask);
    // Retain EPD's actual filtered metric surface, not reconstructed hidden geometry.
    item["attributes"]["surface_points_optical"]=Json::Value(Json::arrayValue);
    for(const auto & point:object.segmented_pcl)
      item["attributes"]["surface_points_optical"].append(array({point.x,point.y,point.z}));
    item["attributes"]["pixel_pitch_m"]=object.centroid.z/std::min(fx,fy);
    snapshot["objects"].append(item);
  }
  evidence["epd_detections_ge_0_80"]=confident;
  evidence["rejected_geometry_count"]=rejected;
  evidence["valid_observations"]=snapshot["objects"].size();
  evidence["clock_domain"]="gazebo_simulation";
  evidence["inference_api"]="external EPD Ort::P3OrtBase";
  evidence["robot_motion_called"]=false;evidence["ros_bridge_started"]=false;
  // This is a frozen observation, not a current live feed after CPU inference.
  snapshot["source"]=evidence;
  {std::ofstream validation(output/"validation_scene.pb",std::ios::binary);scene.SerializeToOstream(&validation);}
  write(output/"snapshot.json",snapshot);write(output/"capture.json",evidence);
  std::cout<<evidence<<std::endl;
  return snapshot["objects"].empty()?2:0;
} catch(const std::exception & e) {std::cerr<<e.what()<<std::endl;return 1;}
