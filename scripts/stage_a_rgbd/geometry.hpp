#pragma once
#include "epd_utils_lib/geometry_quality.hpp"

namespace stage_a {
// All arguments use the same simulation clock, never wall time.
inline bool fresh(int64_t rgb, int64_t depth, int64_t info, int64_t now,
                  int64_t max_age = 1000000000) {
  return rgb > 0 && rgb == depth && rgb == info && now >= rgb && now-rgb <= max_age;
}

inline bool localize(EPD::LocalizedObject & object, const cv::Mat & mask,
                     const cv::Mat & depth, double fx, double fy, double cx, double cy) {
  if (!EPD::validIntrinsics(fx,fy,cx,cy) || depth.empty() || mask.empty() ||
      depth.type()!=CV_32FC1 || mask.type()!=CV_8UC1 || mask.size()!=depth.size()) return false;
  std::vector<float> values;
  for (int v=0;v<depth.rows;++v) for (int u=0;u<depth.cols;++u) {
    float z=depth.at<float>(v,u);
    if (mask.at<uint8_t>(v,u) && std::isfinite(z) && z>=0.02f && z<=5.f) values.push_back(z);
  }
  const int masked=cv::countNonZero(mask);
  if (values.size()<12 || masked<16 || values.size()<0.2*masked) return false;
  auto median=[](std::vector<float> data) {
    auto mid=data.begin()+data.size()/2; std::nth_element(data.begin(),mid,data.end()); return *mid;
  };
  float zmedian=median(values);
  std::vector<float> deviations;
  for (float z:values) deviations.push_back(std::abs(z-zmedian));
  // Exclude isolated depth/background contamination; retain the dominant visible surface.
  const float tolerance=std::max(0.003f,3.f*1.4826f*median(deviations));
  if(tolerance>0.03f) return false;  // No supported dominant cube surface.
  cv::Mat filtered=cv::Mat::zeros(mask.size(),CV_8UC1);
  for (int v=0;v<depth.rows;++v) for (int u=0;u<depth.cols;++u) {
    float z=depth.at<float>(v,u);
    if (mask.at<uint8_t>(v,u) && std::isfinite(z) && z>=0.02f && z<=5.f &&
        std::abs(z-zmedian)<=tolerance) filtered.at<uint8_t>(v,u)=255;
  }
  const int retained=cv::countNonZero(filtered);
  if (retained<12 || retained<0.6*values.size()) return false;
  // EPD owns pinhole deprojection and centroid calculation. No table-plane dimensions.
  if(!EPD::populateMaskedDepthCentroid(object,filtered,depth,fx,fy,cx,cy)) return false;
  // Stage-A's 25 mm cubes fit within a 43.4 mm diagonal. A 45 mm
  // observed span rejects merged masks; it is not an invented object dimension.
  double low[3]={INFINITY,INFINITY,INFINITY},high[3]={-INFINITY,-INFINITY,-INFINITY};
  for(const auto & point:object.segmented_pcl) {
    const double xyz[3]={point.x,point.y,point.z};
    for(int axis=0;axis<3;++axis) {low[axis]=std::min(low[axis],xyz[axis]);high[axis]=std::max(high[axis],xyz[axis]);}
  }
  for(int axis=0;axis<3;++axis) if(high[axis]-low[axis]>0.045) return false;
  return true;
}
}
