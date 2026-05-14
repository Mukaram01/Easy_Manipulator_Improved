#include "workcell_studio_layout_merge.hpp"

namespace fs = boost::filesystem;
namespace workcell_builder {
LayoutMergeResult merge_workcell_studio_layout(const fs::path& scene_dir)
{
  LayoutMergeResult out;
  out.layout_applied = fs::exists(scene_dir / "layout" / "workcell_studio_layout.yaml");
  if (!out.layout_applied) {
    out.warnings.push_back("Run Generate Scene to apply layout");
  }
  return out;
}
}
