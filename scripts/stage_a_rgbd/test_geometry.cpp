#include "geometry.hpp"
#include <cassert>
#include <limits>
int main() {
  using namespace stage_a;
  EPD::LocalizedObject object;
  cv::Mat depth(8,8,CV_32FC1,cv::Scalar(1));
  cv::Mat mask(8,8,CV_8UC1,cv::Scalar(255));
  depth.at<float>(0,0)=100; depth.at<float>(0,1)=std::numeric_limits<float>::quiet_NaN();
  assert(localize(object,mask,depth,1000,1000,3.5,3.5));
  assert(std::abs(object.centroid.z-1)<1e-6);
  assert(!localize(object,mask,depth,100,100,3.5,3.5));
  assert(!localize(object,mask,depth,0,100,3.5,3.5));
  assert(!localize(object,mask,cv::Mat(),1000,1000,3.5,3.5));
  depth.setTo(1.f);depth(cv::Rect(0,0,4,8)).setTo(2.f);
  assert(!localize(object,mask,depth,1000,1000,3.5,3.5));
  depth.setTo(std::numeric_limits<float>::infinity());
  assert(!localize(object,mask,depth,1000,1000,3.5,3.5));
  assert(fresh(100,100,100,150,60));
  assert(!fresh(100,101,100,150,60));
  assert(!fresh(100,100,100,200,60));
  assert(!fresh(100,100,100,90,60));
}
