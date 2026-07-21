#pragma once

#include <algorithm>
#include "embedded_web_curated_add_controller.hpp"

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE

#include <QApplication>
#include <QTimer>

namespace workcell_builder
{
inline void bootstrapEmbeddedWebCuratedAddControllers()
{
  if (!qApp || qApp->property("workcell_embedded_curated_add_bootstrap").toBool()) return;
  qApp->setProperty("workcell_embedded_curated_add_bootstrap", true);
  auto * timer = new QTimer(qApp);
  timer->setInterval(500);
  QObject::connect(timer, &QTimer::timeout, qApp, []() {
    for (QWidget * window : QApplication::topLevelWidgets()) {
      installEmbeddedWebCuratedAddControllers(window);
    }
  });
  timer->start();
  QTimer::singleShot(0, qApp, []() {
    for (QWidget * window : QApplication::topLevelWidgets()) {
      installEmbeddedWebCuratedAddControllers(window);
    }
  });
}
}  // namespace workcell_builder

inline void workcellBuilderBootstrapCuratedAddControllers()
{
  workcell_builder::bootstrapEmbeddedWebCuratedAddControllers();
}
Q_COREAPP_STARTUP_FUNCTION(workcellBuilderBootstrapCuratedAddControllers)

#endif
