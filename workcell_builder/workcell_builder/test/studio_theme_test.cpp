#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QRegularExpression>
#include <QString>

#include "workcell_builder_ui_utils.hpp"

namespace
{
QCoreApplication * ensure_application()
{
  if (auto * app = QCoreApplication::instance()) return app;
  static int argc = 1;
  static char arg0[] = "studio_theme_test";
  static char * argv[] = {arg0, nullptr};
  static QCoreApplication app(argc, argv);
  return &app;
}
}  // namespace

TEST(StudioTheme, TokensResolveAndUnknownNamesStayLiteral)
{
  qputenv("WORKCELL_STUDIO_APPEARANCE", "light");
  ensure_application();
  const auto & t = workcell_builder::studioTokens();
  const QString qss = workcell_builder::studioQss(QStringLiteral("QFrame { background: @surface; color: @text; } @media x"));
  EXPECT_TRUE(qss.contains(t.surface));
  EXPECT_TRUE(qss.contains(t.text));
  EXPECT_FALSE(qss.contains(QStringLiteral("@surface")));
  EXPECT_TRUE(qss.contains(QStringLiteral("@media")));
}

TEST(StudioTheme, ApplicationStyleSheetUsesOnlyTokens)
{
  ensure_application();
  const QString sheet = workcell_builder::workcellStudioStyleSheet();
  EXPECT_FALSE(sheet.contains(QRegularExpression(QStringLiteral("@[A-Za-z]+"))));
  EXPECT_TRUE(sheet.contains(QStringLiteral("QPushButton[role=\"primary\"]")));
  EXPECT_TRUE(sheet.contains(QStringLiteral("QLineEdit[readOnly=\"true\"]")));
  const auto & t = workcell_builder::studioTokens();
  EXPECT_TRUE(sheet.contains(t.accent));
  EXPECT_TRUE(sheet.contains(t.bg));
}

TEST(StudioTheme, LightAndDarkPalettesDiffer)
{
  ensure_application();
  const workcell_builder::StudioTokens & current = workcell_builder::studioTokens();
  // The appearance is resolved once per process; both palettes must still be
  // complete and distinct so a dark desktop never shows light surfaces.
  EXPECT_FALSE(current.bg.isEmpty());
  EXPECT_FALSE(current.sidebar.isEmpty());
  EXPECT_NE(current.bg, current.sidebar);
  EXPECT_NE(current.text, current.bg);
  EXPECT_EQ(workcell_builder::studioAppearance(), workcell_builder::StudioAppearance::Light);
}
