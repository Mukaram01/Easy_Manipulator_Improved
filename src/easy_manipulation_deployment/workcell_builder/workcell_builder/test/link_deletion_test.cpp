#include <gtest/gtest.h>

#include "attributes/object.h"

TEST(WorkcellBuilder, DeleteLinkRemovesDependentJoints)
{
  Object object;
  // Create links
  Link l1; l1.name = "link1";
  Link l2; l2.name = "link2";
  Link l3; l3.name = "link3";
  object.link_vector = {l1, l2, l3};

  // Create joints
  Joint j1; j1.name = "joint1"; j1.parent_link = l1; j1.child_link = l2;
  Joint j2; j2.name = "joint2"; j2.parent_link = l2; j2.child_link = l3;
  Joint j3; j3.name = "joint3"; j3.parent_link = l1; j3.child_link = l3; // unrelated to link2
  object.joint_vector = {j1, j2, j3};

  // Delete link2
  std::vector<size_t> removed = object.remove_link_and_joints(1);

  EXPECT_EQ(object.link_vector.size(), 2u);
  EXPECT_EQ(object.link_vector[0].name, "link1");
  EXPECT_EQ(object.link_vector[1].name, "link3");

  // Joints referencing link2 should be removed
  EXPECT_EQ(removed.size(), 2u);
  ASSERT_EQ(object.joint_vector.size(), 1u);
  EXPECT_EQ(object.joint_vector[0].name, "joint3");
  for (const auto & joint : object.joint_vector) {
    EXPECT_NE(joint.parent_link.name, "link2");
    EXPECT_NE(joint.child_link.name, "link2");
  }
}
