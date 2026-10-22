#pragma once

// Inspector transform spin boxes: precise, predictable editing.
// Translation is authored in metres; one arrow step moves 1 cm, never 1 m.
// Rotation is shown in degrees; one arrow step is 1°.

#include <QDoubleSpinBox>
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

}  // namespace workcell_builder
