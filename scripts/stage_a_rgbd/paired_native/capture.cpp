// Compile the unchanged production/fixture owner into a separate opt-in binary.
// Only its capture call is intercepted; the original translation unit is untouched.
#include "paired_capture.hh"
#define CaptureFragmentMrt(scene,sm,camera,record,path,mark) \
  CapturePairedFragmentMrt(scene,sm,camera,record,path,mark,render,ecm,info,epoch,owner,itemIds)
#include "../gazebo_fragment_capture.cpp"
