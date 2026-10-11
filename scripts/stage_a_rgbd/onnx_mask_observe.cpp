// Standalone diagnostic only: one CPU Run on retained tensor bytes, no EPD/ROS.
#include <onnxruntime_cxx_api.h>
#include <json/json.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
namespace fs=std::filesystem;
void require(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
void bytes(const fs::path &path,const void *data,size_t n){
  int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
  require(fd>=0,"exclusive observation write failed");
  const char *p=static_cast<const char *>(data);size_t done=0;
  while(done<n){ssize_t count=::write(fd,p+done,n-done);if(count<=0){::close(fd);throw std::runtime_error("write failed");}done+=count;}
  require(::close(fd)==0,"close failed");
}
void text(const fs::path &path,const std::string &s){bytes(path,s.data(),s.size());}
void dump(const fs::path &out,const std::vector<Ort::Value> &values,const std::vector<std::string> &names){
  require(values.size()==names.size(),"output inventory mismatch");Json::Value rows(Json::arrayValue);
  for(size_t i=0;i<values.size();++i){
    const auto info=values[i].GetTensorTypeAndShapeInfo();const auto type=info.GetElementType();
    require(type==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||type==ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64,"unsupported tensor type");
    size_t size=info.GetElementCount()*(type==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT?4:8);
    const std::string file="outputs_"+std::to_string(i)+".bin";
    bytes(out/file,values[i].GetTensorData<void>(),size);
    Json::Value row;row["index"]=Json::UInt64(i);row["name"]=names[i];row["onnx_element_type"]=int(type);
    row["bytes"]=Json::UInt64(size);row["file"]=file;
    for(auto n:info.GetShape())row["shape"].append(Json::Int64(n));
    rows.append(row);
  }
  text(out/"outputs.json",Json::writeString(Json::StreamWriterBuilder(),rows)+"\n");
}
void maps(const fs::path &path){std::ifstream f("/proc/self/maps");std::string s((std::istreambuf_iterator<char>(f)),{});text(path,s);}
int main(int argc,char **argv){try{
  auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
  if(argc==3&&std::string(argv[1])=="--synthetic"){
    std::vector<float> f={-0.f,.5f,std::nextafter(.5f,1.f)};const auto before=f;
    std::vector<int64_t> l={1,0,4294967297};int64_t shape[1]={3};std::vector<Ort::Value> v;
    v.emplace_back(Ort::Value::CreateTensor<float>(memory,f.data(),f.size(),shape,1));
    v.emplace_back(Ort::Value::CreateTensor<int64_t>(memory,l.data(),l.size(),shape,1));
    dump(argv[2],v,{"synthetic_float","synthetic_int64"});require(std::memcmp(before.data(),f.data(),12)==0,"observer mutated tensor");
    std::cout<<"PASS typed bytes and ordering; no Session or Run\n";return 0;
  }
  require(argc==4,"usage: diagnostic.onnx input.f32 exclusive-output-directory");fs::path out=argv[3];
  std::ifstream f(argv[2],std::ios::binary);std::vector<char> raw((std::istreambuf_iterator<char>(f)),{});
  require(raw.size()==3145728,"wrong retained input size");std::vector<float> input(raw.size()/4);std::memcpy(input.data(),raw.data(),raw.size());
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"Ort");Ort::SessionOptions options;
  options.SetIntraOpNumThreads(2);options.SetInterOpNumThreads(1);options.SetExecutionMode(ORT_SEQUENTIAL);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  Ort::Session session(env,argv[1],options);Ort::AllocatorWithDefaultOptions allocator;
  require(session.GetInputCount()==1&&session.GetOutputCount()==5,"wrong session IO count");
  auto iname=session.GetInputNameAllocated(0,allocator);require(std::string(iname.get())=="images","wrong input name");
  std::vector<std::string> names={"boxes","labels","scores","masks","/network/roi_heads/Slice_12_output_0"};
  std::vector<const char *> output_names;
  for(size_t i=0;i<names.size();++i){auto n=session.GetOutputNameAllocated(i,allocator);require(n.get()==names[i],"wrong output order");output_names.push_back(names[i].c_str());}
  int64_t shape[4]={1,3,512,512};auto value=Ort::Value::CreateTensor<float>(memory,input.data(),input.size(),shape,4);
  const char *input_names[]={"images"};maps(out/"maps_before_run.txt");
  text(out/"run_entered.json","{\"session_run_calls_entered\":1,\"maximum\":1}\n");
  const auto outputs=session.Run(Ort::RunOptions{nullptr},input_names,&value,1,output_names.data(),output_names.size());
  text(out/"run_completed.json","{\"session_run_calls_completed\":1}\n");maps(out/"maps_after_run.txt");
  dump(out,outputs,names);
  const auto head=outputs[4].GetTensorTypeAndShapeInfo();
  require(head.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT&&head.GetShape()==std::vector<int64_t>({3,1,28,28}),"unexpected genuine observation type/shape");
  require(std::memcmp(input.data(),raw.data(),raw.size())==0,"input mutated");
  text(out/"session.json",std::string("{\"ort_version\":\"")+OrtGetApiBase()->GetVersionString()+"\",\"backend\":\"CPU\",\"intra_op_threads\":2,\"inter_op_threads\":1,\"optimization\":\"ORT_ENABLE_ALL\",\"execution\":\"ORT_SEQUENTIAL\"}\n");
  std::cout<<"PASS exactly one CPU Session::Run; observed all five outputs\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}}
