// TEST FIXTURES ONLY: real P3 preprocessing/ROI path with a synthetic OrtBase.
// No Ort::Session is constructed; no model is opened or inferred.
#include "ort_cpp_lib/p3_ort_base.hpp"
#include "epd_observer.hpp"
#include <cassert>
#include <cstring>
#include <limits>

namespace Ort {
class OrtBase::OrtBaseImpl {};
OrtBase::OrtBase(const std::string &,const boost::optional<size_t> &,
 const boost::optional<int> &,const boost::optional<int> &,const boost::optional<SessionExecutionMode> &,
 const boost::optional<std::vector<std::vector<int64_t>>> &,const boost::optional<bool> &) {}
OrtBase::~OrtBase()=default;
bool OrtBase::isInputUint8(int)const{return false;}
std::vector<OrtBase::DataOutputType> OrtBase::operator()(const std::vector<float *> &input) {
  // Verify actual preprocessing sent to the inference boundary.
  for(int c=0;c<3;++c)for(int i=0;i<512*512;++i)
    assert(std::abs(input[0][c*512*512+i]-static_cast<float>((c==0?10:c==1?30:90)/255.0))<1e-7f);
  // OpenCV float scaling can round differently from scalar double division.
  // Compare the entire original/observed input byte-for-byte in preparation.
  epd_observer::bytes("synthetic_preprocessed_input.f32",input[0],3*512*512*sizeof(float));
  static float boxes[]={-1.75f,1.75f,3.9f,4.8f, 510.9f,510.1f,513.2f,512.9f, 1,1,2,2};
  static int64_t labels[]={1,1,1};static float scores[]={.975f,.8f,0.f};
  static std::vector<float> masks(3*512*512,0.f);
  masks[2*512+2]=std::nextafter(.5f,1.f);masks[2*512+1]=.5f;
  masks[2*512+0]=std::nextafter(.5f,0.f);masks[512*512+511*512+511]=1.f;
  return {{boxes,{3,4}},{reinterpret_cast<float *>(labels),{3}},{scores,{3}},{masks.data(),{3,1,512,512}}};
}
}

int main(int argc,char **argv) {
  assert(argc==2);
  // Observer itself must preserve typed ORT storage including edge bit patterns.
  const auto mem=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
  float values[]={-0.f,.5f,std::nextafter(.5f,1.f),std::numeric_limits<float>::infinity()};
  int64_t labels[]={1,4294967297LL};
  unsigned char before[sizeof(values)];std::memcpy(before,values,sizeof(values));
  const int64_t shape[]={4}, label_shape[]={2};
  std::vector<Ort::Value> tensors;
  tensors.push_back(Ort::Value::CreateTensor<float>(mem,values,4,shape,1));
  tensors.push_back(Ort::Value::CreateTensor<int64_t>(mem,labels,2,label_shape,1));
  std::vector<char *> names={const_cast<char *>("float_fixture"),const_cast<char *>("int64_fixture")};
  epd_observer::tensors("synthetic_ort_boundary",tensors,names);
  assert(std::memcmp(before,values,sizeof(values))==0 && labels[1]==4294967297LL);
  Ort::P3OrtBase epd(1.f,512,512,512,512,2,"TEST_FIXTURE_NO_MODEL",boost::none,2,1,
    Ort::SessionExecutionMode::SEQUENTIAL,boost::none,Ort::InputTensorLayout::CHW,false,true,true);
  epd.initClassNames({"__background__","cube"});
  cv::Mat bgr(512,512,CV_8UC3,cv::Scalar(90,30,10));
  const auto d=epd.infer(bgr);
  assert(d.scores.size()==2 && d.bboxes[0]==(std::array<int,4>{0,1,3,4}) &&
         d.bboxes[1]==(std::array<int,4>{510,510,512,512}));
  assert(d.masks[0].at<float>(1,2)==std::nextafter(.5f,1.f) && d.masks[1].at<float>(1,1)==1.f);
  Json::Value summary;
  for(size_t i=0;i<d.scores.size();++i) {
    Json::Value row;for(int x:d.bboxes[i])row["bbox"].append(x);
    row["class"]=Json::UInt64(d.classIndices[i]);row["score"]=d.scores[i];summary.append(row);
    epd_observer::matrix("synthetic_result_"+std::to_string(i),d.masks[i]);
  }
  epd_observer::metadata("synthetic_result.json",summary);
  std::ofstream pass(argv[1]);pass<<"PASS_NO_MODEL_INFERENCE\n";
}
