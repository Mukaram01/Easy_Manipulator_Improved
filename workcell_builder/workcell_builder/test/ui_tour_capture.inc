// UI tour: visits every Studio page, every visible Scene Builder tab and the New Cell wizard at one window
// size and captures the real window (the application's own X11 window, so the 3D/WebEngine view is
// included). Each shot also records a layout audit: visible buttons/labels whose text does not fit
// (clipped), and enabled controls that lie outside the window (unreachable). Read-only: nothing is saved.
// Included inside class TestMain (studio_full_cycle_acceptance.hpp).
static QJsonObject ui_tour_audit(QWidget * root)
{
  QJsonArray clipped, unreachable;
  const QRect window_rect(QPoint(0, 0), root->size());
  for (auto * widget : root->findChildren<QWidget *>()) {
    if (!widget->isVisible() || widget->width() <= 0) continue;
    const QPoint origin = widget->mapTo(root, QPoint(0, 0));
    // Only widgets actually shown in an unscrolled region count; scroll areas legitimately hide content.
    bool in_scroll = false;
    for (auto * p = widget->parentWidget(); p && p != root; p = p->parentWidget())
      if (qobject_cast<QAbstractScrollArea *>(p)) { in_scroll = true; break; }
    QString text;
    bool wraps = false;
    if (auto * label = qobject_cast<QLabel *>(widget)) {
      text = label->text();
      // Rich text (markup) cannot be measured as plain text; such labels are judged visually instead.
      wraps = label->wordWrap() || label->textFormat() == Qt::RichText || Qt::mightBeRichText(text) ||
              !label->pixmap(Qt::ReturnByValue).isNull();
    } else if (auto * button = qobject_cast<QAbstractButton *>(widget)) {
      text = button->text();
    }
    if (!text.isEmpty() && !wraps && !text.contains('\n') &&
        widget->fontMetrics().horizontalAdvance(text) > widget->contentsRect().width() + 2 &&
        (qobject_cast<QPushButton *>(widget) ? widget->sizeHint().width() > widget->width() + 2 : true))
      clipped.append(QJsonObject{{"class", widget->metaObject()->className()}, {"name", widget->objectName()},
                                 {"text", text.left(80)}, {"width", widget->width()},
                                 {"needed", widget->fontMetrics().horizontalAdvance(text)}});
    const bool control = qobject_cast<QAbstractButton *>(widget) || qobject_cast<QComboBox *>(widget) ||
                         qobject_cast<QLineEdit *>(widget);
    if (control && widget->isEnabled() && !in_scroll &&
        !window_rect.intersects(QRect(origin, widget->size())))
      unreachable.append(QJsonObject{{"class", widget->metaObject()->className()}, {"name", widget->objectName()},
                                     {"text", text.left(80)}});
  }
  // Content cut off by a scroll area that cannot scroll horizontally (outermost overflowing widget only).
  QJsonArray overflow;
  for (auto * area : root->findChildren<QScrollArea *>()) {
    if (!area->isVisible() || area->horizontalScrollBarPolicy() != Qt::ScrollBarAlwaysOff || !area->widget()) continue;
    const int limit = area->viewport()->mapTo(root, QPoint(area->viewport()->width(), 0)).x();
    std::function<void(QWidget *)> visit = [&](QWidget * w) {
      for (auto * child : w->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) {
        if (!child->isVisible()) continue;
        const int right = child->mapTo(root, QPoint(child->width(), 0)).x();
        if (right > limit + 1) {
          // Follow the widest-minimum descendant to name what forces the width.
          QJsonArray chain;
          for (QWidget * w = child; w;) {
            const QSize min = w->minimumSize().width() > 0 ? w->minimumSize() : w->minimumSizeHint();
            chain.append(QString("%1#%2(%3)").arg(w->metaObject()->className(), w->objectName()).arg(min.width()) +
                         (qobject_cast<QAbstractButton *>(w) ? " '" + qobject_cast<QAbstractButton *>(w)->text().left(30) + "'" :
                          qobject_cast<QLabel *>(w) ? " '" + qobject_cast<QLabel *>(w)->text().left(30) + "'" : QString()));
            QWidget * widest = nullptr; int best = 0;
            for (auto * c : w->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) {
              if (!c->isVisible()) continue;
              const int m = std::max(c->minimumSize().width(), c->minimumSizeHint().width());
              if (m > best) { best = m; widest = c; }
            }
            w = widest;
            if (chain.size() > 14) break;
          }
          overflow.append(QJsonObject{{"class", child->metaObject()->className()},
            {"name", child->objectName()}, {"overflow_px", right - limit}, {"min_width", child->minimumSizeHint().width()},
            {"chain", chain}});
        }
        else visit(child);
      }
    };
    visit(area->widget());
  }
  // Leaf-level culprits: visible widgets whose effective minimum width (Ignored policy = 0) is wide.
  QJsonArray wide;
  for (auto * editor : root->findChildren<QWidget *>(QStringLiteral("environmentTaskEditor"))) {
    if (!editor->isVisible()) continue;
    for (auto * w : editor->findChildren<QWidget *>()) {
      if (!w->isVisible() || w->sizePolicy().horizontalPolicy() == QSizePolicy::Ignored) continue;
      const int m = std::max(w->minimumWidth(), w->minimumSizeHint().width());
      bool leafish = true;
      for (auto * c : w->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly))
        if (c->isVisible() && std::max(c->minimumWidth(), c->minimumSizeHint().width()) >= m - 40) leafish = false;
      if (m > 380 && leafish)
        wide.append(QString("%1#%2(%3) %4").arg(w->metaObject()->className(), w->objectName()).arg(m)
                    .arg(qobject_cast<QLabel *>(w) ? qobject_cast<QLabel *>(w)->text().left(40) :
                         qobject_cast<QAbstractButton *>(w) ? qobject_cast<QAbstractButton *>(w)->text().left(40) : QString()));
    }
  }
  // Vertical squeeze: widgets given less height than their own minimum (rows then overlap).
  QJsonArray squeezed;
  for (auto * w : root->findChildren<QWidget *>()) {
    if (!w->isVisible() || !w->layout()) continue;
    const int need = w->layout()->hasHeightForWidth() ? w->layout()->totalHeightForWidth(w->width()) : w->minimumSizeHint().height();
    if (need > w->height() + 2 && !qobject_cast<QAbstractScrollArea *>(w))
      squeezed.append(QString("%1#%2 '%3' h=%4 need=%5").arg(w->metaObject()->className(), w->objectName(),
        qobject_cast<QGroupBox *>(w) ? qobject_cast<QGroupBox *>(w)->title() : QString()).arg(w->height()).arg(need));
  }
  // Per-page minimum widths of the inspector tabs (a tab widget is as wide as its widest page).
  QJsonObject tab_pages;
  for (auto * tabs : root->findChildren<QTabWidget *>(QStringLiteral("sceneBuilderInspectorTabs")))
    for (int i = 0; i < tabs->count(); ++i)
      tab_pages[tabs->tabText(i)] = tabs->widget(i)->minimumSizeHint().width();
  return QJsonObject{{"clipped", clipped}, {"unreachable", unreachable}, {"overflow", overflow}, {"task_editor_wide", wide},
                     {"squeezed", squeezed}, {"inspector_tab_min_widths", tab_pages}};
}

static QPixmap ui_tour_grab(QWidget * widget)
{
  // The window's own X11 contents include composited GL/WebEngine views that QWidget::grab cannot read.
  QPixmap shot = widget->windowHandle() && widget->windowHandle()->screen()
    ? widget->windowHandle()->screen()->grabWindow(widget->winId()) : QPixmap();
  if (!shot.isNull() && !shot.size().isEmpty()) {
    // XWayland returns an all-black image for composited windows; fall back to Qt's own rendering then.
    const QImage image = shot.toImage().scaled(64, 64);
    for (int y = 0; y < image.height(); ++y)
      for (int x = 0; x < image.width(); ++x)
        if (qGray(image.pixel(x, y)) > 8) return shot;
  }
  return widget->grab();
}

static void start_ui_tour(MainWindow * window, QApplication * app, const QString & output, const QString & scene,
                          const QString & tag)
{
  struct Shot { QString name; std::function<void()> enter; int settle_ms; };
  auto shots = std::make_shared<std::vector<Shot>>();
  auto report = std::make_shared<QJsonObject>();
  QDir().mkpath(output);
  auto page = [window](MainWindow::StudioPage p) { return [window, p]() { window->show_studio_page(p); }; };
  auto open_scene = [window, scene]() {
    int index = -1;
    for (size_t i = 0; i < window->scene_browser_result_.scenes.size(); ++i)
      if (QString::fromStdString(window->scene_browser_result_.scenes[i].scene_dir.filename().string()) == scene) index = int(i);
    if (index >= 0) window->open_scene_builder_for_scene_index(index, QStringLiteral("UI tour"));
  };
  shots->push_back({"01_home", page(MainWindow::StudioPage::DashboardPage), 1500});
  shots->push_back({"02_scene_builder", open_scene, 6000});
  shots->push_back({"03_existing_scenes", page(MainWindow::StudioPage::ExistingScenesPage), 800});
  shots->push_back({"04_demo_mode", page(MainWindow::StudioPage::DemoModePage), 800});
  shots->push_back({"05_plan_simulate", page(MainWindow::StudioPage::PlanSimulatePage), 1200});
  shots->push_back({"06_diagnostics", page(MainWindow::StudioPage::DiagnosticsPage), 800});
  shots->push_back({"07_validation", page(MainWindow::StudioPage::ValidationPage), 800});
  shots->push_back({"08_export", page(MainWindow::StudioPage::ExportPage), 800});
  // Scene Builder inspector tabs, discovered at run time (no hard-coded tab list).
  shots->push_back({"09_scene_builder_tabs", [=]() {
    window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
    auto * builder = window->studio_pages_->widget(int(MainWindow::StudioPage::SceneBuilderPage));
    int n = 0;
    for (auto * tabs : builder->findChildren<QTabWidget *>()) {
      if (!tabs->isVisible()) continue;
      for (int i = 0; i < tabs->count(); ++i) {
        tabs->setCurrentIndex(i);
        QCoreApplication::processEvents(); QThread::msleep(250); QCoreApplication::processEvents();
        const QString name = QString("09_tab_%1_%2").arg(++n, 2, 10, QChar('0'))
          .arg(tabs->tabText(i).remove(QRegularExpression("[^A-Za-z0-9]+")).left(24));
        ui_tour_grab(window).save(output + "/" + tag + "_" + name + ".png");
        (*report)[name] = ui_tour_audit(window);
      }
      tabs->setCurrentIndex(0);
    }
  }, 300});
  // The New Cell wizard is modal: capture it from a timer while exec() runs, then cancel it.
  shots->push_back({"10_new_cell_wizard", [=]() {
    window->show_studio_page(MainWindow::StudioPage::DashboardPage);
    QTimer::singleShot(1500, [=]() {
      if (auto * dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget())) {
        ui_tour_grab(dialog).save(output + "/" + tag + "_10_new_cell_wizard.png");
        (*report)["10_new_cell_wizard"] = ui_tour_audit(dialog);
        dialog->reject();
      }
    });
    window->open_new_scene_creation_flow();
  }, 500});
  // Reset Layout (System page): saved arrangement cleared, product defaults re-seeded and applied.
  shots->push_back({"11_reset_layout", [=]() {
    window->show_studio_page(MainWindow::StudioPage::DiagnosticsPage);
    (*report)["tour_window"] = QJsonObject{{"width", window->width()}, {"height", window->height()},
                                           {"device_pixel_ratio", window->devicePixelRatioF()}};
    if (auto * reset = window->findChild<QPushButton *>(QStringLiteral("resetLayoutButton"))) reset->click();
    QSettings settings;
    (*report)["reset_layout"] = QJsonObject{
      {"clicked", window->findChild<QPushButton *>(QStringLiteral("resetLayoutButton")) != nullptr},
      {"saved_geometry_cleared", !settings.contains(QStringLiteral("window/geometry"))},
      {"preferred_left_width", settings.value(QStringLiteral("scene_builder/preferred_left_width")).toInt()},
      {"preferred_right_width", settings.value(QStringLiteral("scene_builder/preferred_right_width")).toInt()},
      {"main_splitter_sizes_cleared", !settings.contains(QStringLiteral("scene_builder/main_splitter_sizes"))}};
  }, 800});
  auto index = std::make_shared<size_t>(0);
  auto step = std::make_shared<std::function<void()>>();
  *step = [=]() {
    if (*index >= shots->size()) {
      QJsonArray splitter_sizes;
      if (window->scene_builder_splitter_) for (int size : window->scene_builder_splitter_->sizes()) splitter_sizes.append(size);
      (*report)["window"] = QJsonObject{{"width", window->width()}, {"height", window->height()},
                                        {"x", window->x()}, {"y", window->y()}, {"maximized", window->isMaximized()},
                                        {"device_pixel_ratio", window->devicePixelRatioF()},
                                        {"scene_splitter_sizes", splitter_sizes}};
      QFile f(output + "/" + tag + "_audit.json");
      if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(*report).toJson());
      window->close(); app->exit(0); return;
    }
    const Shot shot = (*shots)[(*index)++];
    shot.enter();
    QTimer::singleShot(shot.settle_ms, [=]() {
      if (!shot.name.startsWith("09_") && !shot.name.startsWith("10_")) {  // 11_: captured after the reset
        ui_tour_grab(window).save(output + "/" + tag + "_" + shot.name + ".png");
        (*report)[shot.name] = ui_tour_audit(window);
      }
      (*step)();
    });
  };
  QTimer::singleShot(2500, [step]() { (*step)(); });
}
