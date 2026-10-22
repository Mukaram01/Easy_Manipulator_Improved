#include <gtest/gtest.h>

#include <QApplication>
#include <QDoubleSpinBox>

#include "inspector_spin_config.hpp"

namespace
{
QApplication * ensure_app()
{
  if (QApplication::instance()) return qobject_cast<QApplication *>(QApplication::instance());
  static int argc = 0;
  static char * argv[] = {nullptr};
  static QApplication app(argc, argv);
  return &app;
}
}  // namespace

TEST(InspectorSpinConfig, TranslationStepsOneCentimetreAndKeepsMillimetres)
{
  ASSERT_NE(ensure_app(), nullptr);
  QDoubleSpinBox spin;
  spin.setPrefix(QStringLiteral("X "));
  workcell_builder::configure_inspector_translation_spin(&spin);
  EXPECT_DOUBLE_EQ(spin.singleStep(), 0.01);
  EXPECT_EQ(spin.decimals(), 3);
  EXPECT_EQ(spin.suffix(), QStringLiteral(" m"));
  EXPECT_FALSE(spin.isAccelerated());

  spin.setValue(0.45);
  spin.stepUp();
  EXPECT_NEAR(spin.value(), 0.46, 1e-9);      // was 1.45 with the 1.0 m default step
  spin.stepDown();
  spin.stepDown();
  EXPECT_NEAR(spin.value(), 0.44, 1e-9);

  // Direct numeric entry keeps millimetres and sign.
  spin.setValue(-0.123);
  EXPECT_NEAR(spin.value(), -0.123, 1e-9);
  EXPECT_EQ(spin.text(), QStringLiteral("X -0.123 m"));
  spin.setValue(1.2346);
  EXPECT_NEAR(spin.value(), 1.235, 1e-9);     // rounds to the displayed precision, never truncates to 1 m
  EXPECT_LE(spin.minimum(), -1000.0);
  EXPECT_GE(spin.maximum(), 1000.0);
}

TEST(InspectorSpinConfig, RotationStepsOneDegree)
{
  ASSERT_NE(ensure_app(), nullptr);
  QDoubleSpinBox spin;
  workcell_builder::configure_inspector_rotation_spin(&spin);
  EXPECT_DOUBLE_EQ(spin.singleStep(), 1.0);
  EXPECT_EQ(spin.suffix(), QString::fromUtf8("°"));
  spin.setValue(-90.5);
  spin.stepUp();
  EXPECT_NEAR(spin.value(), -89.5, 1e-9);
}
