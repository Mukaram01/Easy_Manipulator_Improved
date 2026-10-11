// OFFLINE arithmetic replay only. No Scene/Camera/Server is constructed.
#include <OgreMatrix4.h>
#include <OgreVector3.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <cstring>
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  Json::Value cap;std::ifstream in(argv[1]);in>>cap;
  auto pair=cap["paired_capture"];auto state=pair["frozen"][0];Ogre::Matrix4 view,projection;
  for(int y=0;y<4;++y)for(int x=0;x<4;++x){view[y][x]=state["view"][y*4+x].asFloat();projection[y][x]=state["projection"][y*4+x].asFloat();}
  auto inverse=(projection*view).inverse();Json::Value report;
  report["stage"]="OFFLINE float32 inverse projection-view, near z=-1 and midpoint z=0; matching installed Ogre Camera disassembly";
  bool exact=true;
  for(int i=0;i<2;++i)for(const auto &r:pair["pixel_rays"][i]) {
    const float n=i?512:256,u=(r["yx"][1].asFloat()+.5f)/n,v=(r["yx"][0].asFloat()+.5f)/n;
    const float nx=2*u-1,ny=1-2*v;
    auto origin=inverse*Ogre::Vector3(nx,ny,-1);auto middle=inverse*Ogre::Vector3(nx,ny,0);
    auto direction=middle-origin;direction.normalise();Json::Value row;row["resolution"]=int(n);row["yx"]=r["yx"];
    for(int c=0;c<3;++c) {
      float a=r["origin"][c].asFloat(),b=r["direction"][c].asFloat();
      exact=exact && std::memcmp(&a,&origin[c],sizeof(float))==0 && std::memcmp(&b,&direction[c],sizeof(float))==0;
      row["origin"].append(origin[c]);row["direction"].append(direction[c]);
    }
    report["rays"].append(row);
  }
  report["bitwise_exact"]=exact;std::cout<<report<<std::endl;return exact?0:1;
}
