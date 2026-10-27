// Align & Arrange acceptance (included inside TestMain): drives the real Scene
// Builder on a disposable cell copy. The bin is aligned to the workbench with a
// viewport-picked reference, previewed, cancelled, applied, undone, redone,
// saved and reopened; Keep relative is followed by a workbench move; locked,
// zone and self references are refused; a missed Mate click keeps both picks;
// and an optional AirPick cell must reach the read-only diagnostic Product View.
// Fake hardware only: no planning, execution or robot motion.

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
static void start_align_arrange_acceptance(MainWindow * window, QApplication * app, const QString & output,
    const QString & scene_name, QSize window_size, const QString & airpick_scene)
{
  struct Session {
    int step = 0, ticks = 0, sub = 0, corner = 0;
    QString scene;
    QJsonObject evidence;
    bool js_pending = false; QVariant js_result;
    QByteArray env_hash, layout_hash;
    std::array<double, 6> bin_before{}, bin_target{};
    qint64 pick_seq_before = 0;
  };
  using Step = std::function<bool()>;
  auto s = std::make_shared<Session>();
  auto steps = std::make_shared<std::vector<std::pair<QString, Step>>>();
  QDir().mkpath(output);
  s->evidence = {{"result", "FAIL"}, {"fake_hardware_only", true}, {"execution_goals_sent", 0}, {"robot_motion_commanded", false}};
  const QString bin = "target_bin_default", table = "support_surface_table", zone = "default_drop_zone";
  auto write = [=]() {
    s->evidence["studio_log_tail"] = window->studio_log_->toPlainText().right(6000);
    QFile f(output + "/align_arrange.json");
    if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson());
  };
  auto require = [](bool value, const QString & message) { if (!value) throw std::runtime_error(message.toStdString()); };
  auto item = [=](const QString & id) {
    for (const auto & p : window->all_scene_preview_items_) if (p.id == id) return p;
    throw std::runtime_error(("scene item missing: " + id).toStdString());
  };
  auto pose = [=](const QString & id) {
    const auto p = item(id);
    return std::array<double, 6>{p.x, p.y, p.z, p.roll, p.pitch, p.yaw};
  };
  auto pose_json = [](const std::array<double, 6> & v) { return QJsonArray{v[0], v[1], v[2], v[3], v[4], v[5]}; };
  auto same = [](const std::array<double, 6> & a, const std::array<double, 6> & b, double tol) {
    for (int k = 0; k < 6; ++k) if (std::abs(a[k] - b[k]) > tol) return false;
    return true;
  };
  auto file_hash = [](const QString & path) {
    QFile f(path); if (!f.open(QIODevice::ReadOnly)) return QByteArray();
    return QCryptographicHash::hash(f.readAll(), QCryptographicHash::Sha256).toHex();
  };
  auto layout_path = [=]() { return s->scene + "/layout/workcell_studio_layout.yaml"; };
  auto files_unchanged = [=]() {
    return file_hash(s->scene + "/environment.yaml") == s->env_hash && file_hash(layout_path()) == s->layout_hash;
  };
  auto shot = [=](const QString & name) {
    QCoreApplication::processEvents();
    const QString script = QDir(window->detect_workspace_root()).filePath(
      "src/easy_manipulation_deployment/workcell_builder/workcell_builder/test/x11_window_capture.py");
    const int code = QProcess::execute("python3", {script, QString::number(window->winId()), output + "/" + name + ".png"});
    if (code != 0) window->grab().save(output + "/" + name + ".png");
    QJsonArray shots = s->evidence.value("screenshots").toArray(); shots.append(name); s->evidence["screenshots"] = shots;
  };
  auto align = [=]() { return window->align_controller_; };
  auto panel = [=]() { return window->align_panel_; };
  auto tool = [=](const char * name) {
    auto * b = panel()->findChild<QAbstractButton *>(name);
    require(b, QString("Align panel control missing: ") + name);
    return b;
  };
  auto web = [=]() -> QWebEngineView * {
    for (auto * view : window->scene_preview_widget_->findChildren<QWebEngineView *>()) if (view->isVisible()) return view;
    return nullptr;
  };
  auto js = [=](const QString & script) {
    auto * view = web(); require(view, "the Product View is not visible");
    s->js_pending = true; s->js_result = QVariant();
    view->page()->runJavaScript(script, [s](const QVariant & value) { s->js_result = value; s->js_pending = false; });
  };
  auto mouse_click = [=](const QPoint & p) {
    auto * view = web(); if (!view) return;
    QWidget * target = view->focusProxy() ? view->focusProxy() : view;
    const QPoint local = target->mapFrom(view, p);
    for (auto type : {QEvent::MouseMove, QEvent::MouseButtonPress, QEvent::MouseButtonRelease}) {
      const auto button = type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
      const auto buttons = type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton;
      QMouseEvent event(type, QPointF(local), QPointF(target->window()->mapFromGlobal(target->mapToGlobal(local))),
        QPointF(target->mapToGlobal(local)), button, Qt::MouseButtons(buttons), Qt::NoModifier);
      QApplication::sendEvent(target, &event);
    }
  };
  auto modal_watch = new QTimer(window); modal_watch->setInterval(250);
  QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
    for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
      if (!message->isVisible()) continue;
      s->evidence["failure"] = "unexpected message: " + message->text();
      shot("failure"); write(); message->reject(); app->exit(1);
    }
  }); modal_watch->start();
  auto add = [=](const QString & name, Step step) { steps->push_back({name, step}); };
  auto open_cell = [=](const QString & name) {
    s->scene = window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/" + name;
    QStringList blockers; QJsonObject diagnostics;
    require(window->load_scene_for_scene3d_smoke(name, s->scene, &blockers, &diagnostics), "cell did not open: " + blockers.join("; "));
    window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
  };
  auto web_ready = [=]() {
    auto * preview = window->scene_preview_widget_;
    return preview && preview->embedded_web_authoring_active();
  };
  // Waits for the Product View, then forces the requested window size.
  auto wait_view = [=](const QString & tag) {
    return [=]() -> bool {
      if (s->ticks < 10 || !web_ready()) return false;
      if (window->size() != window_size && s->ticks < 200) {
        window->setWindowState(Qt::WindowNoState); window->resize(window_size); return false;
      }
      s->evidence[tag + "_window_size"] = QJsonArray{window->width(), window->height()};
      return s->ticks > 40;
    };
  };
  // A real viewport click on a world point (projected by the viewer itself).
  auto click_world = [=](const QString & tag, std::array<double, 3> p, std::function<bool()> done) {
    return [=]() -> bool {
      if (s->js_pending) return false;
      if (s->sub == 0) {
        js(QString("window.__WORKCELL_EDITOR_API_V1__.projectWorldPoint([%1,%2,%3])").arg(p[0], 0, 'g', 12).arg(p[1], 0, 'g', 12).arg(p[2], 0, 'g', 12));
        s->sub = 1; return false;
      }
      if (s->sub == 1) {
        const auto projected = s->js_result.toMap();
        require(!projected.isEmpty() && projected.value("inFront").toBool(), tag + ": point is not in view");
        const QPoint target(qRound(projected.value("x").toDouble()), qRound(projected.value("y").toDouble()));
        s->evidence[tag + "_widget_point"] = QJsonArray{target.x(), target.y()};
        mouse_click(target);
        s->sub = 2; s->ticks = 0; return false;
      }
      if (done()) return true;
      // The viewer may still be wiring up pickable parts: click again, at most three times.
      if (s->ticks >= 50) {
        const int attempts = s->evidence.value(tag + "_attempts").toInt() + 1;
        s->evidence[tag + "_attempts"] = attempts;
        require(attempts < 3, tag + ": the click had no effect");
        s->sub = 0;
      }
      return false;
    };
  };
  auto arrange_state = [=](const QString & tag, std::function<void(const QJsonObject &)> check) {
    return [=]() -> bool {
      if (s->js_pending) return false;
      if (s->sub == 0) { if (s->ticks < 6) return false; js("window.__WORKCELL_EDITOR_API_V1__.getArrangeState()"); s->sub = 1; return false; }
      const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
      s->evidence[tag] = state;
      check(state);
      return true;
    };
  };

  // ---- Steps -----------------------------------------------------------------
  add("load cell", [=]() { if (s->ticks < 5) return false; open_cell(scene_name); return true; });
  add("wait for the Product View", wait_view("cell"));
  add("baseline", [=]() {
    s->env_hash = file_hash(s->scene + "/environment.yaml");
    s->layout_hash = file_hash(layout_path());
    QFile::copy(layout_path(), output + "/layout_before.yaml");
    QFile::copy(s->scene + "/environment.yaml", output + "/environment_before.yaml");
    s->bin_before = pose(bin);
    s->evidence["bin_before"] = pose_json(s->bin_before);
    s->evidence["table_pose"] = pose_json(pose(table));
    window->apply_scene_selection(bin, "", false, false);
    require(window->inspector_align_button_ && window->inspector_align_button_->isVisible(), "Align & Arrange button is not in the Inspector");
    shot("01_bin_selected");
    return true;
  });
  add("start from the Inspector", [=]() {
    window->inspector_align_button_->click();
    require(window->align_active() && panel()->isVisible(), "Align & Arrange panel did not open");
    require(align()->moving() == bin, "the selected bin is not the part to move");
    require(!tool("alignCentreX")->isEnabled() && tool("alignCentreX")->toolTip().startsWith("Choose a reference part first"),
      "operations are enabled before a reference, or do not say why");
    require(!tool("alignApply")->isEnabled(), "Apply is enabled before a preview");
    s->evidence["panel_audit_start"] = TestMain::ui_tour_audit(window);
    shot("02_panel_open");
    return true;
  });
  add("reference: click the workbench in the 3D view", click_world("reference_click", {1.05, -0.48, 0.0}, [=]() {
    if (align()->reference() != table) return false;
    require(window->current_selected_scene_item_id_ == bin, "the part to move lost the selection");
    return true;
  }));
  add("gizmo held off, both parts highlighted", arrange_state("arrange_after_reference", [=](const QJsonObject & state) {
    require(state.value("active").toBool() && state.value("reference").toString() == table, "viewer has no arrange preview");
    require(!state.value("gizmoEnabled").toBool(), "the move gizmo is enabled during Align & Arrange");
    require(state.value("highlightCount").toInt() == 2, "both parts are not highlighted");
  }));
  add("preview: centre on reference + place on top", [=]() {
    if (s->sub == 0) {
      tool("alignCentreBoth")->click();
      tool("alignPlaceOnTop")->click();
      const auto & preview = align()->preview();
      require(preview.ok, "preview failed: " + preview.error);
      require(files_unchanged(), "a preview wrote scene files");
      require(same(pose(bin), s->bin_before, 0.), "the bin moved during preview");
      require(tool("alignApply")->isEnabled(), "Apply is disabled with a valid preview");
      s->evidence["preview_centre_on_top"] = align()->evidence();
      s->sub = 1; s->ticks = 0; return false;
    }
    if (s->ticks < 8) return false;
    shot("03_preview_centre_on_top");
    return true;
  });
  add("ghost drawn", arrange_state("arrange_preview", [=](const QJsonObject & state) {
    require(state.value("ghostCount").toInt() >= 1, "no ghost is drawn");
  }));
  add("Cancel leaves files and pose unchanged", [=]() {
    tool("alignCancel")->click();
    require(!window->align_active() && !panel()->isVisible(), "Cancel did not close the panel");
    require(files_unchanged() && !window->layout_dirty_, "Cancel changed scene files or dirty state");
    require(same(pose(bin), s->bin_before, 0.), "Cancel moved the bin");
    return true;
  });
  add("viewer preview cleared", arrange_state("arrange_after_cancel", [=](const QJsonObject & state) {
    require(!state.value("active").toBool() && state.value("ghostCount").toInt() == 0, "preview stayed in the viewer");
  }));
  add("refusals: self, zone, generated robot, locked part", [=]() {
    window->apply_scene_selection(bin, "", false, false);
    window->begin_align_arrange();
    require(window->align_active(), "Align & Arrange did not start again");
    align()->set_reference(bin);
    QJsonObject refused{{"self", align()->status()}};
    align()->set_reference(zone);
    refused["zone"] = align()->status();
    QString robot;
    for (const auto & p : window->all_scene_preview_items_) if (p.robot_candidate || p.source_layer != "editable_layout") { robot = p.id; break; }
    if (!robot.isEmpty()) { align()->set_reference(robot); refused["generated"] = align()->status(); refused["generated_id"] = robot; }
    require(align()->reference().isEmpty(), "a refused part became the reference");
    require(refused.value("self").toString().contains("part being moved") && refused.value("zone").toString().contains("task zone"),
      "refusals do not explain themselves");
    if (!robot.isEmpty()) {
      require(!refused.value("generated").toString().isEmpty(), "a generated part was not refused");
      refused["moving_generated"] = window->align_moving_blocker(robot);
      require(!refused.value("moving_generated").toString().isEmpty(), "a generated part may be moved");
    }
    s->evidence["refusals"] = refused;
    return true;
  });
  add("reference from the list; centre X, align back, +50 mm X; Apply", [=]() {
    auto * combo = panel()->findChild<QComboBox *>("alignReference");
    require(combo, "reference list missing");
    const int index = combo->findData(table);
    require(index > 0, "the workbench is not offered as a reference");
    combo->setCurrentIndex(index);
    emit combo->activated(index);
    require(align()->reference() == table, "choosing from the list did not set the reference");
    tool("alignCentreX")->click();
    tool("alignBack")->click();
    auto * offset = panel()->findChild<QDoubleSpinBox *>("alignOffsetX");
    require(offset && offset->isEnabled(), "X offset is not available after Centre X");
    offset->setValue(50.);
    const auto preview = align()->preview();
    require(preview.ok, "preview failed: " + preview.error);
    const auto target = preview.report.value("target_world").toObject().value("xyz").toArray();
    s->bin_target = {target[0].toDouble(), target[1].toDouble(), target[2].toDouble(), s->bin_before[3], s->bin_before[4], s->bin_before[5]};
    s->evidence["apply_preview"] = align()->evidence();
    s->evidence["panel_audit_preview"] = TestMain::ui_tour_audit(window);
    return true;
  });
  add("preview shown before Apply", [=]() { if (s->ticks < 10) return false; shot("04_preview_back_edge_offset"); return true; });
  add("Apply", [=]() {
    tool("alignApply")->click();
    require(!window->align_active(), "Apply did not finish the session");
    require(same(pose(bin), s->bin_target, 1e-9), "the bin is not at the previewed target");
    require(window->layout_dirty_, "Apply did not mark the layout dirty");
    s->evidence["bin_after_apply"] = pose_json(pose(bin));
    return true;
  });
  add("Applied pose shown", [=]() { if (s->ticks < 15) return false; shot("05_applied"); return true; });
  add("Undo and Redo", [=]() {
    window->undo_layout_edit();
    require(same(pose(bin), s->bin_before, 1e-12), "Undo did not restore the bin");
    window->redo_layout_edit();
    require(same(pose(bin), s->bin_target, 1e-9), "Redo did not reapply the alignment");
    return true;
  });
  add("Save Layout", [=]() {
    if (s->sub == 0) { window->save_layout_button_->click(); s->sub = 1; s->ticks = 0; return false; }
    if (s->ticks < 10 || window->layout_dirty_) return false;
    QFile::remove(output + "/layout_after_save.yaml"); QFile::remove(output + "/environment_after_save.yaml");
    QFile::copy(layout_path(), output + "/layout_after_save.yaml");
    QFile::copy(s->scene + "/environment.yaml", output + "/environment_after_save.yaml");
    s->evidence["saved_hashes_changed"] = QJsonObject{{"environment", file_hash(s->scene + "/environment.yaml") != s->env_hash},
      {"layout", file_hash(layout_path()) != s->layout_hash}};
    return true;
  });
  add("close and reopen the cell", [=]() { if (s->ticks < 5) return false; open_cell(scene_name); return true; });
  add("wait for the reopened Product View", wait_view("reopen"));
  add("reopened placement is identical", [=]() {
    const auto reopened = pose(bin);
    s->evidence["bin_after_reopen"] = pose_json(reopened);
    double worst = 0.;
    for (int k = 0; k < 6; ++k) worst = std::max(worst, std::abs(reopened[k] - s->bin_target[k]));
    s->evidence["reopen_max_abs_error"] = worst;
    require(worst < 1e-9, "the reopened bin differs from the applied pose by " + QString::number(worst));
    shot("06_reopened");
    return true;
  });
  add("Keep relative: bin follows a workbench move; undo both", [=]() {
    window->apply_scene_selection(bin, "", false, false);
    window->begin_align_arrange();
    align()->set_reference(table);
    tool("alignCentreY")->click();
    auto * keep = panel()->findChild<QRadioButton *>("alignKeepRelative");
    require(keep && keep->isEnabled(), "Keep relative is not available for the workbench");
    keep->click();
    require(align()->preview().ok && align()->preview().request.value("parent_object").toString() == table,
      "Keep relative does not plan a mount on the workbench");
    tool("alignApply")->click();
    const auto aligned = pose(bin);
    const auto relation = YAML::Load(item(bin).mounting_yaml.toStdString());
    require(relation["parent_object"].as<std::string>("") == table.toStdString(), "the bin is not mounted on the workbench");
    auto moved_table = pose(table);
    QJsonObject move{{"id", table}, {"world_pose", QJsonObject{{"xyz", QJsonArray{moved_table[0] + 0.1, moved_table[1], moved_table[2]}},
      {"rpy", QJsonArray{moved_table[3], moved_table[4], moved_table[5]}}}}};
    require(window->commit_mounting_request(move), "moving the workbench failed");
    const auto followed = pose(bin);
    s->evidence["keep_relative"] = QJsonObject{{"aligned", pose_json(aligned)}, {"after_table_move", pose_json(followed)}};
    require(std::abs(followed[0] - aligned[0] - 0.1) < 1e-9 && std::abs(followed[1] - aligned[1]) < 1e-9, "the bin did not follow the workbench");
    window->undo_layout_edit();
    window->undo_layout_edit();
    require(same(pose(bin), s->bin_target, 1e-9), "undo did not restore the saved bin");
    return true;
  });
  add("Mate: rim and surface chosen, then a missed click", [=]() {
    if (s->sub == 0) {
      window->apply_scene_selection(bin, "", false, false);
      window->begin_visual_mate();
      QString error;
      require(window->mate_active(), "Mate did not start");
      require(window->mate_controller_->choose_datum(workcell_builder::MateToolController::Role::Child, bin, "rim", &error), error);
      require(window->mate_controller_->choose_datum(workcell_builder::MateToolController::Role::Parent, table, "surface", &error), error);
      require(window->mate_controller_->step() == workcell_builder::MateToolController::Step::Ready, "Mate is not ready");
      s->sub = 1; s->ticks = 0; return false;
    }
    if (s->sub == 1) { if (s->ticks < 10 || s->js_pending) return false; js("window.__WORKCELL_EDITOR_API_V1__.projectWorldPoint([0,0,0])"); s->sub = 2; return false; }
    if (s->sub == 2) {
      if (s->js_pending) return false;
      const auto canvas = s->js_result.toMap().value("canvas").toMap();
      require(!canvas.isEmpty(), "canvas geometry unavailable");
      // Background above the cell: points along the upper band of the canvas, away from overlays.
      const double left = canvas.value("left").toDouble(), top = canvas.value("top").toDouble();
      const double width = canvas.value("width").toDouble(), height = canvas.value("height").toDouble();
      const double fractions[4][2] = {{0.12, 0.18}, {0.88, 0.18}, {0.5, 0.14}, {0.12, 0.85}};
      require(s->corner < 4, "no background point produced a missed click");
      const QPoint point(qRound(left + width * fractions[s->corner][0]), qRound(top + height * fractions[s->corner][1]));
      s->pick_seq_before = window->mate_seen_pick_seq_;
      s->evidence["missed_click_point"] = QJsonArray{point.x(), point.y()};
      mouse_click(point);
      s->sub = 3; s->ticks = 0; return false;
    }
    if (window->mate_seen_pick_seq_ <= s->pick_seq_before) {
      if (s->ticks < 30) return false;
      ++s->corner; s->sub = 1; s->ticks = 10; return false;  // an overlay took the click: try the next point
    }
    const auto picks = window->mate_controller_->evidence().value("picks").toArray();
    const auto last = picks.last().toObject().value("resolution").toObject();
    QJsonArray tried = s->evidence.value("missed_click_attempts").toArray();
    tried.append(QJsonObject{{"point", s->evidence.value("missed_click_point")}, {"reason_code", last.value("reason_code")}});
    s->evidence["missed_click_attempts"] = tried;
    if (last.value("reason_code").toString() != "NO_ASSET") {
      require(window->mate_controller_->step() == workcell_builder::MateToolController::Step::Ready, "a part click dropped a pick");
      ++s->corner; s->sub = 1; s->ticks = 10; return false;
    }
    require(window->mate_active(), "a missed click cancelled Mate");
    require(window->mate_controller_->step() == workcell_builder::MateToolController::Step::Ready, "a missed click dropped a pick");
    require(window->mate_controller_->status().contains("missed every part"), "the status does not explain the missed click");
    require(same(pose(bin), s->bin_target, 1e-9), "the missed click moved the bin");
    s->evidence["mate_missed_click"] = window->mate_controller_->evidence();
    shot("07_mate_missed_click_keeps_picks");
    window->mate_controller_->cancel();
    require(!window->mate_active(), "Mate did not cancel");
    require(same(pose(bin), s->bin_target, 1e-9), "cancelled Mate moved the bin");
    return true;
  });
  add("saved files untouched since Save", [=]() {
    require(file_hash(layout_path()) == file_hash(output + "/layout_after_save.yaml") &&
      file_hash(s->scene + "/environment.yaml") == file_hash(output + "/environment_after_save.yaml"),
      "scene files changed after Save without a Save");
    return true;
  });
  if (!airpick_scene.isEmpty()) {
    add("AirPick: open the cell", [=]() { if (s->ticks < 5) return false; open_cell(airpick_scene); return true; });
    add("AirPick: read-only diagnostic Product View", [=]() {
      auto * preview = window->scene_preview_widget_;
      if (s->js_pending) return false;
      if (!preview->property("diagnostic_preview_active").toBool()) { require(s->ticks < 800, "AirPick did not reach a diagnostic Product View"); return false; }
      if (s->sub == 0) { js("window.__WORKCELL_EDITOR_API_V1__.getArrangeState()"); s->sub = 1; return false; }
      const QString shown = QJsonObject::fromVariantMap(s->js_result.toMap()).value("sceneId").toString();
      if (shown != airpick_scene) { require(s->ticks < 800, "the Product View never showed " + airpick_scene); s->sub = 0; return false; }
      if (s->sub == 1) { s->sub = 2; s->ticks = 0; return false; }
      if (s->ticks < 30) return false;
      s->evidence["airpick_diagnostic_preview"] = QJsonObject{{"diagnostic_preview_active", true}, {"viewer_scene", shown},
        {"save_layout_enabled", window->save_layout_button_->isEnabled()}};
      shot("08_airpick_diagnostic_preview");
      return true;
    });
  }
  add("finish", [=]() { s->evidence["result"] = "PASS"; write(); return true; });

  // ---- Runner ----------------------------------------------------------------
  auto * timer = new QTimer(window); timer->setInterval(100);
  QObject::connect(timer, &QTimer::timeout, window, [=]() {
    try {
      if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
      if (s->step >= int(steps->size())) { timer->stop(); modal_watch->stop(); window->close(); app->exit(0); return; }
      const auto & current = (*steps)[s->step];
      require(++s->ticks < 900, "timed out at step: " + current.first);
      if (current.second()) {
        QJsonArray done = s->evidence.value("steps").toArray(); done.append(current.first); s->evidence["steps"] = done;
        ++s->step; s->ticks = 0; s->sub = 0;
      }
    } catch (const std::exception & error) {
      s->evidence["failure"] = QString::fromStdString(error.what());
      s->evidence["failed_step"] = s->step < int(steps->size()) ? (*steps)[s->step].first : QString();
      if (window->align_controller_) s->evidence["align_state"] = window->align_controller_->evidence();
      shot("failure"); write(); timer->stop(); app->exit(1);
    }
  });
  timer->start();
}
#endif  // WORKCELL_BUILDER_HAS_WEBENGINE
