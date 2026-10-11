#include "uniform_readback.hh"
#include <cassert>
#include <limits>
int main() {
  Ogre::GpuConstantDefinition d;d.constType=Ogre::GCT_FLOAT2;
  d.elementSize=2;d.arraySize=1;d.physicalIndex=1;
  assert(readableFloatUniform(d,2,3));
  assert(!readableFloatUniform(d,2,2));
  d.physicalIndex=std::numeric_limits<std::size_t>::max();assert(!readableFloatUniform(d,2,3));
  d.physicalIndex=0;d.constType=Ogre::GCT_INT2;assert(!readableFloatUniform(d,2,3));
  d.constType=Ogre::GCT_DOUBLE2;assert(!readableFloatUniform(d,2,3));
  d.constType=Ogre::GCT_FLOAT1;assert(!readableFloatUniform(d,2,3));
  assert(readableFloatUniform(d,1,3));d.arraySize=2;assert(!readableFloatUniform(d,1,3));
  d.arraySize=1;d.elementSize=0;assert(!readableFloatUniform(d,1,3));
}
