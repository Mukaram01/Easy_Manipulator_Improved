// Copyright 2021 ROS Industrial Consortium Asia Pacific
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

#include <gtest/gtest.h>

#include "emd/grasp_execution/context.hpp"
#include "emd/grasp_execution/exception.hpp"

TEST(TestContext, MissingFileThrows)
{
  grasp_execution::WorkcellContext ctx;
  EXPECT_THROW(ctx.init_from_yaml("nonexistent_file.yaml"),
               grasp_execution::ContextFileLoadingException);
}

