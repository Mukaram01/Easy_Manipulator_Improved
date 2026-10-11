#pragma once
#include <OgreGpuProgramParams.h>
// Reject unsupported uniform layouts before touching Ogre's float storage.
inline bool readableFloatUniform(const Ogre::GpuConstantDefinition &d,
    std::size_t components,std::size_t floatCount) {
  return (components==1 || components==2) &&
      d.constType==(components==2?Ogre::GCT_FLOAT2:Ogre::GCT_FLOAT1) &&
      d.arraySize==1 && d.elementSize>=components &&
      d.physicalIndex<=floatCount && components<=floatCount-d.physicalIndex;
}
