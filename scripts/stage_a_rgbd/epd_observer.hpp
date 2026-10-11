// Diagnostic-only observers. const tensor/image access; no inference or mutation.
#pragma once
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <jsoncpp/json/json.h>
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace epd_observer {
inline std::filesystem::path directory() {
  const char *p=std::getenv("EPD_OBSERVER_DIR");
  if(!p || !*p)throw std::runtime_error("missing isolated observer directory");
  return p;
}
inline void bytes(const std::string &name,const void *data,size_t size) {
  const auto path=directory()/name;
  if(std::filesystem::exists(path))throw std::runtime_error("observer refuses repeated stage: "+name);
  std::ofstream file(path,std::ios::binary);
  file.write(static_cast<const char *>(data),size);
  if(!file)throw std::runtime_error("observer write failed: "+name);
}
inline void metadata(const std::string &name,const Json::Value &value) {
  const std::string text=Json::writeString(Json::StreamWriterBuilder(),value)+"\n";
  bytes(name,text.data(),text.size());
}
inline void tensors(const std::string &stage,const std::vector<Ort::Value> &values,
                    const std::vector<char *> &names) {
  if(values.size()!=names.size())throw std::runtime_error("observer tensor inventory mismatch");
  Json::Value rows(Json::arrayValue);
  for(size_t i=0;i<values.size();++i) {
    const auto info=values[i].GetTensorTypeAndShapeInfo();
    const auto type=info.GetElementType();size_t width=0;
    if(type==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)width=sizeof(float);
    else if(type==ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64)width=sizeof(int64_t);
    else throw std::runtime_error("unsupported observed tensor type");
    Json::Value row;row["index"]=Json::UInt64(i);row["name"]=names[i];
    row["onnx_element_type"]=int(type);row["elements"]=Json::UInt64(info.GetElementCount());
    for(auto d:info.GetShape())row["shape"].append(Json::Int64(d));
    row["file"]=stage+"_"+std::to_string(i)+".bin";
    row["bytes"]=Json::UInt64(info.GetElementCount()*width);
    // GetTensorData<void>() returns the underlying const storage via ORT API.
    bytes(row["file"].asString(),values[i].GetTensorData<void>(),info.GetElementCount()*width);
    rows.append(row);
  }
  metadata(stage+".json",rows);
}
inline void matrix(const std::string &name,const cv::Mat &mask) {
  if(mask.type()!=CV_32FC1)throw std::runtime_error("observer expected float32 mask");
  // ROIs may be strided. Preserve exactly each row's logical bytes.
  std::vector<unsigned char> buffer(mask.total()*sizeof(float));
  for(int y=0;y<mask.rows;++y)
    std::memcpy(buffer.data()+y*mask.cols*sizeof(float),mask.ptr<float>(y),mask.cols*sizeof(float));
  bytes(name+".f32",buffer.data(),buffer.size());
}
inline void roi(size_t raw_index,size_t epd_index,const float *box,float scale_x,float scale_y,
                int xmin,int ymin,int xmax,int ymax,const cv::Mat &full,const cv::Mat &cropped) {
  const std::string name="detection_"+std::to_string(raw_index);
  Json::Value row;row["raw_index"]=Json::UInt64(raw_index);row["epd_index"]=Json::UInt64(epd_index);
  for(int j=0;j<4;++j) {
    row["floating_box"].append(box[j]);
    row["scaled_box"].append(box[j]/(j%2==0?scale_x:scale_y));
    row["truncated_box_before_clamp"].append(static_cast<int>(box[j]/(j%2==0?scale_x:scale_y)));
  }
  for(int v:{xmin,ymin,xmax,ymax})row["roi_xyxy_exclusive"].append(v);
  row["full_shape_hw"].append(full.rows);row["full_shape_hw"].append(full.cols);
  row["roi_shape_hw"].append(cropped.rows);row["roi_shape_hw"].append(cropped.cols);
  row["full_file"]=name+".full.f32";row["roi_file"]=name+".roi.f32";
  matrix(name+".full",full);matrix(name+".roi",cropped);metadata(name+".json",row);
}
}
