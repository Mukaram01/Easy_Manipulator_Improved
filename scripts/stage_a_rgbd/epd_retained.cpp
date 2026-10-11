// RGB-only offline caller of external EPD. No simulator identity inputs or ROS node.
#include "ort_cpp_lib/p3_ort_base.hpp"
#include <opencv2/opencv.hpp>
#include <onnxruntime_c_api.h>
#include <set>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=5)return 2; // model, labels, RGB8, fresh output directory
  const std::filesystem::path out=argv[4];Json::Value report;
  try {
    std::ifstream input(argv[3],std::ios::binary);std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),{});
    if(bytes.size()!=256*256*3)throw std::runtime_error("RGB dimensions mismatch");
    std::ifstream file(argv[2]);std::vector<std::string> labels;std::string label;
    while(std::getline(file,label))if(!label.empty())labels.push_back(label);
    if(labels.empty())throw std::runtime_error("missing labels");
    cv::Mat rgb(256,256,CV_8UC3,bytes.data()),resized,bgr;
    cv::resize(rgb,resized,cv::Size(512,512),0,0,cv::INTER_LINEAR);
    cv::cvtColor(resized,bgr,cv::COLOR_RGB2BGR);
    std::ofstream prepared(out/"epd_input.bgr8",std::ios::binary);
    prepared.write(reinterpret_cast<const char*>(bgr.data),512*512*3);prepared.close();
    Ort::P3OrtBase epd(1.f,512,512,512,512,labels.size(),argv[1],boost::none,2,1,
      Ort::SessionExecutionMode::SEQUENTIAL,boost::none,Ort::InputTensorLayout::CHW,false,true,true);
    epd.initClassNames(labels);
    const auto start=std::chrono::steady_clock::now();
    const auto detections=epd.infer(bgr); // Exactly ONE genuine inference call.
    report["inference_seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    report["api"]="external EPD Ort::P3OrtBase::infer";report["device"]="CPU_gpuIdx_none";
    report["onnxruntime_version"]=OrtGetApiBase()->GetVersionString();report["opencv_version"]=CV_VERSION;
    report["detections"]=Json::Value(Json::arrayValue);
    if(detections.bboxes.size()!=detections.scores.size() || detections.classIndices.size()!=detections.scores.size() || detections.masks.size()!=detections.scores.size())throw std::runtime_error("EPD output inventory mismatch");
    for(size_t i=0;i<detections.scores.size();++i) {
      Json::Value d;d["class_index"]=Json::UInt64(detections.classIndices[i]);d["confidence"]=detections.scores[i];
      if(detections.classIndices[i]>=labels.size())throw std::runtime_error("invalid class index");
      d["label"]=labels[detections.classIndices[i]];
      const auto box=detections.bboxes[i];for(auto x:box)d["bbox"].append(x);
      const auto mask=detections.masks[i];
      if(mask.type()!=CV_32FC1 || box[0]<0 || box[1]<0 || box[2]>512 || box[3]>512 ||
        box[2]<=box[0] || box[3]<=box[1] || mask.cols!=box[2]-box[0] || mask.rows!=box[3]-box[1])throw std::runtime_error("EPD ROI geometry mismatch");
      const auto name="mask_"+std::to_string(i)+".f32";d["mask_file"]=name;
      std::ofstream f(out/name,std::ios::binary);
      for(int y=0;y<mask.rows;++y)f.write(reinterpret_cast<const char*>(mask.ptr<float>(y)),mask.cols*sizeof(float));
      if(!f)throw std::runtime_error("mask write failed");report["detections"].append(d);
    }
    report["status"]="INFERENCE_COMPLETE";
  }catch(const std::exception &e){report["status"]="BLOCKED";report["failure"]=e.what();}
  std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> loaded;
  while(std::getline(maps,line)){const auto pos=line.find('/');if(pos!=std::string::npos&&line.find(".so",pos)!=std::string::npos)loaded.insert(line.substr(pos));}
  for(const auto &path:loaded)report["loaded_library_paths"].append(path);
  std::ofstream f(out/"inference.json");f<<report<<'\n';
  return report["status"]=="INFERENCE_COMPLETE"&&f?0:2;
}
