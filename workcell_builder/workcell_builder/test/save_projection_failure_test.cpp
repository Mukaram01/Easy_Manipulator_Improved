#include <gtest/gtest.h>

#include "save_projection_failure.hpp"

using workcell_builder::projection_failure_message;
using workcell_builder::projection_failure_reason;

TEST(SaveProjectionFailure, TracebackIsReducedToItsLastLineWithAHint)
{
  const QString traceback = QStringLiteral(
    "Traceback (most recent call last):\n"
    "  File \"/repo/scripts/workcell_studio_layout_merge.py\", line 263, in <module>\n"
    "    a = ap.parse_args(); rep = merge(a.scene_dir, a.deleted_item_id, save_authored=a.save_authored)\n"
    "  File \"/repo/scripts/physical_destination.py\", line 68, in contains\n"
    "    raise ValueError('placement/object outside target usable placement geometry')\n"
    "ValueError: placement/object outside target usable placement geometry\n");
  const QString reason = projection_failure_reason(traceback, QString());
  EXPECT_FALSE(reason.contains(QStringLiteral("Traceback")));
  EXPECT_FALSE(reason.startsWith(QStringLiteral("ValueError")));
  EXPECT_TRUE(reason.startsWith(QStringLiteral("placement/object outside target usable placement geometry")));
  EXPECT_TRUE(reason.contains(QStringLiteral("destination zone")));
  EXPECT_TRUE(projection_failure_message(traceback, QString()).startsWith(QStringLiteral("Save failed: authored layout projection rejected the edit: ")));
}

TEST(SaveProjectionFailure, FallsBackToStdoutThenToAGenericReason)
{
  EXPECT_EQ(projection_failure_reason(QString(), QStringLiteral("merge: blocked by task binding\n")),
    QStringLiteral("merge: blocked by task binding"));
  EXPECT_EQ(projection_failure_reason(QString(), QString()), QStringLiteral("the projection helper reported no detail"));
  // A sentence that merely contains ": " is not treated as an exception prefix.
  EXPECT_EQ(projection_failure_reason(QStringLiteral("asset bin_01: catalog mesh must be an existing STL"), QString()),
    QStringLiteral("catalog mesh must be an existing STL"));
}
