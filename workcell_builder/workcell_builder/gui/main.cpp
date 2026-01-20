// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QApplication>
#include <iostream>
#include <string>

#include "gui/mainwindow.h"
#include "path_resolver.h"


int main(int argc, char * argv[])
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--workcell-root" || arg == "--workspace-root") && i + 1 < argc) {
      PathResolver::set_workspace_root_override(argv[++i]);
      continue;
    }
    if (arg == "--scenes-root" && i + 1 < argc) {
      PathResolver::set_scenes_root_override(argv[++i]);
      continue;
    }
  }
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
