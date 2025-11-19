/**
 * @file convex_hull_compat.cpp
 * @brief Compatibility helpers for btConvexHullComputer symbol changes
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <LinearMath/btConvexHullComputer.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#if defined(__GNUC__)
#define TESSERACT_WEAK __attribute__((weak))
#else
#define TESSERACT_WEAK
#endif

// Bullet 3.25 changed btConvexHullComputer::compute to accept a void pointer.
// Older binary releases do not provide that symbol, which breaks linking when
// the newer headers are used. Provide a weak definition that forwards the call
// to the legacy implementation so we remain compatible with both versions.
btScalar TESSERACT_WEAK btConvexHullComputer::compute(const void* coords,
                                                      bool /*use32BitIndices*/,
                                                      int stride,
                                                      int count,
                                                      btScalar shrink,
                                                      btScalar shrinkClamp)
{
  return compute(static_cast<const btScalar*>(coords), stride, count, shrink, shrinkClamp);
}

#undef TESSERACT_WEAK
