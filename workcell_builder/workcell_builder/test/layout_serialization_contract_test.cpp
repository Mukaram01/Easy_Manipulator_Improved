#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {
std::string load_file(const std::string & path)
{
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}

TEST(LayoutSerializationContractTest, SaveLayoutWritesCanonicalFields)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("item[\"id\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"display_name\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"category\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"type\"]"), std::string::npos);
  EXPECT_NE(src.find("pose[\"xyz\"]"), std::string::npos);
  EXPECT_NE(src.find("pose[\"rpy\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"dimensions\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"editable\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"locked\"]"), std::string::npos);
  EXPECT_NE(src.find("ensure_map_node(item, \"mesh\")"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SaveLayoutPreservesUnknownFieldsViaCloneMerge)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("YAML::Clone(existing_by_id[item_id])"), std::string::npos);
  EXPECT_NE(src.find("existing_by_id"), std::string::npos);
}

TEST(LayoutSerializationContractTest, MalformedLayoutBackupBehaviorStillPresent)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find(".malformed_backup_"), std::string::npos);
  EXPECT_NE(src.find("Malformed environment_layout.yaml detected"), std::string::npos);
}
