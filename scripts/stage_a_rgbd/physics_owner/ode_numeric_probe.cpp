// Test oracle only: no DART world, stepping, Gazebo, controller or motion.
#include <Eigen/Geometry>
#include <ode/ode.h>
#include <cfenv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <type_traits>
#ifdef __FAST_MATH__
#error Unsupported fast-math test configuration
#endif
static_assert(std::is_same<dReal,double>::value && std::numeric_limits<double>::is_iec559);
int main() {
  const int modes[]={FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO};
  for(int axis=0;axis<4;++axis)for(double angle:{0.,1e-20,.15,2.0943951023931953,2.9,3.141592653589793}) {
    std::fesetround(FE_TONEAREST);
    const Eigen::Vector3d unit=axis<3?Eigen::Vector3d::Unit(axis):Eigen::Vector3d(1.,2.,3.).normalized();
    const Eigen::Matrix3d r=Eigen::AngleAxisd(angle,unit).toRotationMatrix();
    for(int mode:modes) {
      if(std::fesetround(mode)!=0)return 2;
      Eigen::Quaterniond q(r);dQuaternion native={q.w(),q.x(),q.y(),q.z()};
      dNormalize4(native);dMatrix3 out;dRfromQ(out,native);
      std::cout<<"{\"rounding\":"<<mode<<",\"input\":[";
      for(int i=0;i<3;++i) {
        if(i)std::cout<<",";std::cout<<"[";
        for(int j=0;j<3;++j){if(j)std::cout<<",";std::cout<<"\""<<std::hexfloat<<r(i,j)<<"\"";}
        std::cout<<"]";
      }
      std::cout<<"],\"output\":[";
      for(int i=0;i<3;++i) {
        if(i)std::cout<<",";std::cout<<"[";
        for(int j=0;j<3;++j){if(j)std::cout<<",";std::cout<<"\""<<std::hexfloat<<out[4*i+j]<<"\"";}
        std::cout<<"]";
      }
      std::cout<<"]}\n";
    }
  }
  std::fesetround(FE_TONEAREST);
}
