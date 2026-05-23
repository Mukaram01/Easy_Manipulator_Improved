#include <gtest/gtest.h>

#include "gui/mainwindow.h"
#include "rviz_preview_runner.hpp"

namespace
{

TEST(WorkcellBuilderCompileCoverage, MainWindowAndRvizPreviewRunnerHeadersCompileTogether)
{
  workcell_builder::PreviewReadinessStatus status;
  status.ready = false;
  EXPECT_FALSE(status.ready);
}

}  // namespace
