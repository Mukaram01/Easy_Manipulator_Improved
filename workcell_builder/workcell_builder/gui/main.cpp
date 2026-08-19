// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Keep the existing application entrypoint byte-for-byte in main_legacy.inc and
// make the Home shell composition explicit at MainWindow construction.
// This avoids the startup event filters/timers that caused the v2-v5 regressions.

#include <QHBoxLayout>
#include <QPalette>
#include <QPixmap>
#include <QRegularExpression>
#include <QSizePolicy>

#include "gui/mainwindow.h"
#include "gui/scene3d_viewport_widget.h"
#include "gui/scene_select.h"
#include "gui/startup_dialog.h"
#include "workcell_builder_ui_utils.hpp"
#include "home_workcells_target_shell.hpp"
#include "home_workcell_preview_web.hpp"

class WorkcellStudioHomeMainWindow final : public MainWindow
{
public:
  explicit WorkcellStudioHomeMainWindow(
    const QString & startup_workspace = QString(),
    const QString & startup_ros_distro = QString(),
    QWidget * parent = nullptr)
  : MainWindow(startup_workspace, startup_ros_distro, parent)
  {
    const QString workspace = startup_workspace.trimmed().isEmpty()
      ? QString::fromLocal8Bit(qgetenv("WORKCELL_WORKSPACE_ROOT"))
      : startup_workspace;
    workcell_builder::home_workcells::configure_target_shell(this, workspace);
    workcell_builder::home_workcells::install_home_snapshot_preview(this, workspace);
  }
};

// main_legacy.inc includes the same headers again, but they are protected by
// pragma-once/include guards. Only MainWindow references in the implementation
// below are redirected to the construction wrapper above.
#define MainWindow WorkcellStudioHomeMainWindow
#include "main_legacy.inc"
#undef MainWindow
