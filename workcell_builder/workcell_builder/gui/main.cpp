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
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <iostream>
#include <string>

#include "include/path_resolver.h"
#include "gui/mainwindow.h"

namespace fs = boost::filesystem;

namespace {
struct RootConfig
{
  enum class Source
  {
    kNone,
    kWorkcell,
    kScenes
  };
  Source source{ Source::kNone };
  fs::path root;
};

RootConfig parse_root_from_args(int argc, char * argv[])
{
  RootConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--workcell-root" || arg == "--scenes-root") {
      if (i + 1 < argc) {
        config.source = (arg == "--workcell-root") ? RootConfig::Source::kWorkcell :
          RootConfig::Source::kScenes;
        config.root = fs::path(argv[i + 1]);
      }
      break;
    }
  }
  return config;
}

fs::path fallback_root()
{
  const char * home = std::getenv("HOME");
  if (home && *home) {
    return fs::path(home) / "workcell_ws";
  }
  return fs::current_path() / "workcell_ws";
}
}  // namespace


int main(int argc, char * argv[])
{
  // Precedence for the root path: CLI flag > WORKCELL_BUILDER_ROOT > fallback.
  const RootConfig cli_config = parse_root_from_args(argc, argv);
  workcell_builder::PathResolver & resolver = workcell_builder::PathResolver::instance();
  if (cli_config.source == RootConfig::Source::kWorkcell && !cli_config.root.empty()) {
    resolver.set_workcell_root(cli_config.root);
  } else if (cli_config.source == RootConfig::Source::kScenes && !cli_config.root.empty()) {
    resolver.set_scenes_root(cli_config.root);
  } else if (const char * env_root = std::getenv("WORKCELL_BUILDER_ROOT")) {
    if (*env_root) {
      resolver.set_workcell_root(env_root);
    } else {
      resolver.set_workcell_root(fallback_root());
    }
  } else {
    resolver.set_workcell_root(fallback_root());
  }

  QApplication a(argc, argv);
  MainWindow w(resolver.workcell_root());
  w.show();
  return a.exec();
}
