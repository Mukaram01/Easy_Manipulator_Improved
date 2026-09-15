#include "task_intent_model.hpp"

#include <gtest/gtest.h>

TEST(TaskIntentModel, CanonicalHashGoldenIsStableAcrossYamlFormatting)
{
  const std::string a = "b: 2\na: [0.0, 1]\n";
  const std::string b = "# comment\na: [0, 1.000000]\nb: 2.0\n";
  EXPECT_EQ(workcell_builder::canonical_task_intent_sha256(a), workcell_builder::canonical_task_intent_sha256(b));
  EXPECT_EQ(workcell_builder::canonical_task_intent_sha256(a), "517ae15b859520c5bba87fca695ad2166ff17d3bc3a527c3c85dfb5fa68d5c4b");
}
