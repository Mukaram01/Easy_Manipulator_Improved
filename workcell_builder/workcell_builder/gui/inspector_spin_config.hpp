#pragma once

// Inspector transform spin boxes: precise, predictable editing.
// Translation is authored in metres; one arrow step moves 1 cm, never 1 m.
// Rotation is shown in degrees; one arrow step is 1°.

#include <QDoubleSpinBox>
#include <cmath>
#include <QSizePolicy>

namespace workcell_builder
{

constexpr double kInspectorTranslationStepMetres = 0.01;
constexpr int kInspectorTranslationDecimals = 3;      // millimetre precision survives Apply/Save
constexpr double kInspectorTranslationRangeMetres = 1000.0;
constexpr double kInspectorRotationStepDegrees = 1.0;
constexpr int kInspectorRotationDecimals = 2;
constexpr double kInspectorRotationRangeDegrees = 360.0;

inline void configure_inspector_spin_geometry(QDoubleSpinBox * spin)
{
  if (!spin) return;
  spin->setMinimumWidth(64);
  spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  spin->setKeyboardTracking(false);      // commit on Enter/focus-out, not per keystroke
  spin->setAccelerated(false);           // holding the arrow never accelerates into large jumps
}

inline void configure_inspector_translation_spin(QDoubleSpinBox * spin)
{
  if (!spin) return;
  configure_inspector_spin_geometry(spin);
  spin->setDecimals(kInspectorTranslationDecimals);
  spin->setRange(-kInspectorTranslationRangeMetres, kInspectorTranslationRangeMetres);
  spin->setSingleStep(kInspectorTranslationStepMetres);
  spin->setSuffix(QStringLiteral(" m"));
}

inline void configure_inspector_rotation_spin(QDoubleSpinBox * spin)
{
  if (!spin) return;
  configure_inspector_spin_geometry(spin);
  spin->setDecimals(kInspectorRotationDecimals);
  spin->setRange(-kInspectorRotationRangeDegrees, kInspectorRotationRangeDegrees);
  spin->setSingleStep(kInspectorRotationStepDegrees);
  spin->setSuffix(QString::fromUtf8("°"));
}

// True when the field shows something other than the item's exact value
// rounded to the display precision. Untouched fields keep their exact value on
// Apply, so editing Y never rounds X or Z (a flush mate stays flush).
inline bool inspector_field_changed(const QDoubleSpinBox * spin, double exact_display_units)
{
  if (!spin) return false;
  const double half_quantum = 0.5 * std::pow(10.0, -spin->decimals());
  return std::abs(spin->value() - exact_display_units) > half_quantum + 1e-12;
}

inline double inspector_applied_value(const QDoubleSpinBox * spin, double exact_display_units)
{
  return inspector_field_changed(spin, exact_display_units) ? spin->value() : exact_display_units;
}

}  // namespace workcell_builder
