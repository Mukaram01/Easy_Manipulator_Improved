#include <gtest/gtest.h>
#include <QFile>
#include <QString>
#include "gui/transform_clipboard_utils.h"

TEST(TransformClipboardUtils, ParsesValidFormat)
{
  double x=0,y=0,z=0,r=0,p=0,yaw=0;
  QString error;
  EXPECT_TRUE(workcell_builder::parse_transform_clipboard_text("x=1 y=2 z=3 r=0.1 p=-0.2 yaw=3.14", &x,&y,&z,&r,&p,&yaw,&error));
  EXPECT_DOUBLE_EQ(x, 1.0); EXPECT_DOUBLE_EQ(y, 2.0); EXPECT_DOUBLE_EQ(z, 3.0);
  EXPECT_DOUBLE_EQ(r, 0.1); EXPECT_DOUBLE_EQ(p, -0.2); EXPECT_DOUBLE_EQ(yaw, 3.14);
}

TEST(TransformClipboardUtils, RejectsInvalidFormat)
{
  QString error;
  EXPECT_FALSE(workcell_builder::parse_transform_clipboard_text("x=1 y=2", nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,&error));
  EXPECT_FALSE(error.isEmpty());
}

TEST(MainWindowUiAcceptance, HasSelectionTransformActionTokens)
{
  QFile file("gui/mainwindow.cpp");
  ASSERT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(file.readAll());
  EXPECT_NE(text.indexOf("Apply | Revert | Copy Transform | Paste Transform | Live update"), -1);
}
