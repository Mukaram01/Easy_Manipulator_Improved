// TEST FIXTURES ONLY: no renderer/server/inference is instantiated.
#include "paired_capture.hh"
#include <cassert>
int main() {
  Json::Value a;a["step"]=2;a["epoch"]=2;a["geometry"]=.025;
  a["projection"]=MatrixWitness(Ogre::Matrix4::IDENTITY);
  RequireFrozen(a,a);
  for(const auto key:{"step","epoch","geometry","projection"}) {
    auto b=a;b[key]=0;bool rejected=false;
    try{RequireFrozen(a,b);}catch(const std::runtime_error&){rejected=true;}
    assert(rejected);
  }
  auto matrix=MatrixWitness(Ogre::Matrix4::IDENTITY);
  assert(matrix.size()==16 && matrix[0].asDouble()==1 && matrix[5].asDouble()==1 && matrix[10].asDouble()==1 && matrix[15].asDouble()==1);
  std::cout<<"PASS_TEST_FIXTURES_ONLY frozen state and matrix storage\n";
}
