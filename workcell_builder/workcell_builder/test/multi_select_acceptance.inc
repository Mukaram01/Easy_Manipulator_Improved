// Multi-selection and arrangement toolbar acceptance (included inside TestMain).
// Drives the real Scene Builder on a disposable fixture (ur5_2f_test plus two
// more bins, docs/acceptance/multi-select-arrange-v1/build_fixture.py):
// plain 3D click + Ctrl+click in 3D + Ctrl+click in the hierarchy, toolbar
// availability, Align preview/Cancel/Apply, one-step Undo/Redo with the
// selection kept, exact-gap distribution, Save/reopen parity, atomic Fixed
// mount followed by a reference move, and the toolbar overflow. Every geometry
// result is re-checked with an independent resolver plan (distance 0 = done).
// Fake hardware only: no planning, execution or robot motion.

static void start_multi_select_acceptance(MainWindow * window, QApplication * app, const QString & output,
    const QString & scene_name, QSize window_size, const QString & airpick_scene = QString())
{
  struct Session {
    int step = 0, ticks = 0, sub = 0;
    QString scene;
    QJsonObject evidence;
    bool js_pending = false; QVariant js_result;
    QByteArray env_hash, layout_hash;
    QHash<QString, std::array<double, 6>> before, aligned, distributed;
    QByteArray env_before_open, layout_before_open;
    bool expect_switch_dialog = false;
    bool native = false;  // native Scene3D fallback: selection through the hierarchy, no viewer JS
    QString switch_choice;
    int corner = 0;
    qint64 pick_seq_before = 0;
  };
  using Step = std::function<bool()>;
  auto s = std::make_shared<Session>();
  auto steps = std::make_shared<std::vector<std::pair<QString, Step>>>();
  QDir().mkpath(output);
  s->evidence = {{"result", "FAIL"}, {"fake_hardware_only", true}, {"execution_goals_sent", 0}, {"robot_motion_commanded", false}};
  const QString table = "support_surface_table", bin = "target_bin_default", reject = "reject_bin", spare = "spare_bin";
  const QStringList bins{bin, reject, spare};
  auto write = [=]() {
    s->evidence["studio_log_tail"] = window->studio_log_->toPlainText().right(6000);
    QFile f(output + "/multi_select.json");
    if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson());
  };
  auto require = [](bool value, const QString & message) { if (!value) throw std::runtime_error(message.toStdString()); };
  auto pose = [=](const QString & id) {
    for (const auto & p : window->all_scene_preview_items_) if (p.id == id) return std::array<double, 6>{p.x, p.y, p.z, p.roll, p.pitch, p.yaw};
    throw std::runtime_error(("scene item missing: " + id).toStdString());
  };
  auto poses = [=](const QStringList & ids) { QHash<QString, std::array<double, 6>> r; for (const auto & id : ids) r.insert(id, pose(id)); return r; };
  auto json6 = [](const std::array<double, 6> & v) { return QJsonArray{v[0], v[1], v[2], v[3], v[4], v[5]}; };
  auto same = [=](const QHash<QString, std::array<double, 6>> & expected, double tol) {
    for (auto it = expected.constBegin(); it != expected.constEnd(); ++it) {
      const auto now = pose(it.key());
      for (int k = 0; k < 6; ++k) if (std::abs(now[k] - it.value()[k]) > tol) return false;
    }
    return true;
  };
  auto file_hash = [](const QString & path) {
    QFile f(path); if (!f.open(QIODevice::ReadOnly)) return QByteArray();
    return QCryptographicHash::hash(f.readAll(), QCryptographicHash::Sha256).toHex();
  };
  auto files_unchanged = [=]() {
    return file_hash(s->scene + "/environment.yaml") == s->env_hash &&
      file_hash(s->scene + "/layout/workcell_studio_layout.yaml") == s->layout_hash;
  };
  auto shot = [=](const QString & name) {
    QCoreApplication::processEvents();
    const QString script = QDir(window->detect_workspace_root()).filePath(
      "src/easy_manipulation_deployment/workcell_builder/workcell_builder/test/x11_window_capture.py");
    if (QProcess::execute("python3", {script, QString::number(window->winId()), output + "/" + name + ".png"}) != 0) {
      window->grab().save(output + "/" + name + ".png");
    }
    QJsonArray shots = s->evidence.value("screenshots").toArray(); shots.append(name); s->evidence["screenshots"] = shots;
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
  auto mouse_click = [=](const QPoint & p, Qt::KeyboardModifiers modifiers) {
    auto * view = web(); if (!view) return;
    QWidget * target = view->focusProxy() ? view->focusProxy() : view;
    const QPoint local = target->mapFrom(view, p);
    for (auto type : {QEvent::MouseMove, QEvent::MouseButtonPress, QEvent::MouseButtonRelease}) {
      const auto button = type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
      const auto buttons = type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton;
      QMouseEvent event(type, QPointF(local), QPointF(target->window()->mapFromGlobal(target->mapToGlobal(local))),
        QPointF(target->mapToGlobal(local)), button, Qt::MouseButtons(buttons), modifiers);
      QApplication::sendEvent(target, &event);
    }
  };
  // A real viewport click on a world point the viewer projects itself; retried while the viewer wires up picking.
  auto click_world = [=](const QString & tag, std::array<double, 3> p, Qt::KeyboardModifiers modifiers, std::function<bool()> done) {
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
        mouse_click(target, modifiers);
        s->sub = 2; s->ticks = 0; return false;
      }
      if (done()) return true;
      if (s->ticks >= 50) {
        const int attempts = s->evidence.value(tag + "_attempts").toInt() + 1;
        s->evidence[tag + "_attempts"] = attempts;
        require(attempts < 3, tag + ": the click had no effect");
        s->sub = 0;
      }
      return false;
    };
  };
  auto tree_ctrl_click = [=](const QString & id) {
    auto * tree = window->scene_hierarchy_tree_;
    QTreeWidgetItem * row = nullptr;
    for (QTreeWidgetItemIterator it(tree); *it; ++it) if ((*it)->data(0, Qt::UserRole + 1).toString() == id) { row = *it; break; }
    require(row, "hierarchy row missing: " + id);
    tree->scrollToItem(row);
    const QPoint centre = tree->visualItemRect(row).center();
    for (auto type : {QEvent::MouseButtonPress, QEvent::MouseButtonRelease}) {
      QMouseEvent event(type, centre, tree->viewport()->mapToGlobal(centre), Qt::LeftButton,
        type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton, Qt::ControlModifier);
      QApplication::sendEvent(tree->viewport(), &event);
    }
  };
  auto action = [=](const QString & tool) {
    auto * a = window->arrange_toolbar_->action(tool); require(a, "toolbar tool missing: " + tool); return a;
  };
  auto arrange = [=]() { return window->arrange_controller_; };
  // Independent check: re-planning the very same operation now must move nothing.
  auto replan_moves_nothing = [=](const QJsonObject & request, const QString & tag) {
    const auto response = window->run_mounting_request(request);
    require(response.value("ok").toBool(), tag + ": verification plan failed: " + response.value("error").toString());
    const auto moved = response.value("result").toObject().value("report").toObject().value("moved").toArray();
    double worst = 0.;
    for (const auto & entry : moved) worst = std::max(worst, entry.toObject().value("distance_m").toDouble());
    s->evidence[tag + "_residual_m"] = worst;
    require(worst < 1e-9, tag + ": the result is not exact; a fresh plan still moves a part by " + QString::number(worst));
  };
  auto modal_watch = new QTimer(window); modal_watch->setInterval(250);
  QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
    for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
      if (!message->isVisible()) continue;
      if (s->expect_switch_dialog && message->objectName() == QStringLiteral("unsavedLayoutSwitchDialog")) {
        s->evidence["switch_dialog_" + s->switch_choice] = message->text();
        for (auto * button : message->buttons()) if (button->text() == s->switch_choice) { button->click(); break; }
        s->expect_switch_dialog = false;
        continue;
      }
      s->evidence["failure"] = "unexpected message: " + message->text();
      shot("failure"); write(); message->reject(); app->exit(1);
    }
  }); modal_watch->start();
  auto add = [=](const QString & name, Step step) { steps->push_back({name, step}); };
  // Steps that need the Web3D viewer (JS state, canvas clicks). In native mode they are skipped
  // and listed in the evidence, so a native run never reports a viewer check it did not make.
  auto add_web = [=](const QString & name, Step step) {
    steps->push_back({name, [=]() -> bool {
      if (!s->native) return step();
      QJsonArray skipped = s->evidence.value("skipped_in_native").toArray(); skipped.append(name);
      s->evidence["skipped_in_native"] = skipped;
      return true;
    }});
  };
  auto open_cell = [=]() {
    s->scene = window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/" + scene_name;
    QStringList blockers; QJsonObject diagnostics;
    require(window->load_scene_for_scene3d_smoke(scene_name, s->scene, &blockers, &diagnostics), "cell did not open: " + blockers.join("; "));
    window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
  };
  auto wait_view = [=](const QString & tag) {
    return [=]() -> bool {
      auto * preview = window->scene_preview_widget_;
      s->native = false;  // the native renderer was retired; viewer-only steps always run
      s->evidence["backend"] = s->native ? "native_scene3d" : "embedded_web3d";
      if (s->ticks < 10 || !(preview && (s->native || preview->embedded_web_authoring_active()))) return false;
      // An empty window_size means "maximized on --screen": keep what the window manager granted.
      if (window_size.isValid() && window->size() != window_size && s->ticks < 200) {
        window->setWindowState(Qt::WindowNoState); window->resize(window_size); return false;
      }
      if (!window_size.isValid() && !window->isMaximized() && s->ticks < 200) return false;
      s->evidence[tag + "_window_size"] = QJsonArray{window->width(), window->height()};
      s->evidence[tag + "_maximized"] = window->isMaximized();
      if (auto * screen = window->windowHandle() ? window->windowHandle()->screen() : nullptr) {
        s->evidence[tag + "_screen"] = QJsonObject{{"name", screen->name()}, {"width", screen->geometry().width()},
          {"height", screen->geometry().height()}, {"device_pixel_ratio", screen->devicePixelRatio()}};
      }
      return s->ticks > 60;
    };
  };
  auto viewer_selection = [=](const QString & tag, std::function<void(const QJsonObject &)> check) {
    return [=]() -> bool {
      if (s->js_pending) return false;
      if (s->sub == 0) { if (s->ticks < 8) return false; js("window.__WORKCELL_EDITOR_API_V1__.getSelectionSetState()"); s->sub = 1; return false; }
      const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
      s->evidence[tag] = state;
      check(state);
      return true;
    };
  };

  add("load fixture cell", [=]() {
    if (s->ticks < 5) return false;
    const QString dir = window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/" + scene_name;
    s->env_before_open = file_hash(dir + "/environment.yaml");
    s->layout_before_open = file_hash(dir + "/layout/workcell_studio_layout.yaml");
    open_cell();
    return true;
  });
  add("wait for the Product View", wait_view("cell"));
  add("baseline", [=]() {
    s->env_hash = file_hash(s->scene + "/environment.yaml");
    s->layout_hash = file_hash(s->scene + "/layout/workcell_studio_layout.yaml");
    s->before = poses(QStringList{table} + bins);
    // Opening a cell must never write its authored files (one candidate for the earlier Cancel flake).
    s->evidence["open_wrote_authored_files"] = QJsonObject{{"environment", s->env_hash != s->env_before_open},
      {"layout", s->layout_hash != s->layout_before_open}};
    require(s->env_hash == s->env_before_open && s->layout_hash == s->layout_before_open, "opening the cell rewrote authored files");
    QJsonObject b; for (auto it = s->before.constBegin(); it != s->before.constEnd(); ++it) b[it.key()] = json6(it.value());
    s->evidence["poses_before"] = b;
    require(!action("align_front")->isEnabled(), "Align is enabled without a selection");
    return true;
  });
  add("native: select the workbench from the hierarchy", [=]() {
    if (!s->native) return true;
    window->apply_scene_selection(table, QString(), false, false);
    tree_ctrl_click(bin);
    require(window->selection_set_.members() == (QStringList{table, bin}), "native: hierarchy selection failed");
    return true;
  });
  add_web("plain 3D click selects the workbench (reference)", click_world("table_click", {0.2, -0.3, 0.0}, Qt::NoModifier, [=]() {
    return window->selection_set_.members() == QStringList{table};
  }));
  add_web("Ctrl+click in 3D adds the destination bin", click_world("bin_ctrl_click", {0.45, 0.22, 0.12}, Qt::ControlModifier, [=]() {
    return window->selection_set_.members() == (QStringList{table, bin});
  }));
  add("Ctrl+click in the hierarchy adds the reject and spare bins", [=]() {
    tree_ctrl_click(reject);
    tree_ctrl_click(spare);
    require(window->selection_set_.members() == (QStringList{table, bin, reject, spare}),
      "hierarchy Ctrl+click did not extend the selection: " + window->selection_set_.members().join(","));
    require(window->selection_set_.reference() == table, "the first selected part is not the reference");
    return true;
  });
  add("selection shown everywhere", [=]() -> bool {
    if (!s->native) return viewer_selection("viewer_selection", [=](const QJsonObject & state) {
      require(state.value("highlightCount").toInt() == 4, "the 3D view does not outline all four parts");
      require(state.value("reference").toString() == table, "the 3D view shows another reference");
      require(!state.value("gizmoEnabled").toBool(), "the single-part move gizmo is active with four parts selected");
    })();
    return true;
  });
  add("selection shown in the hierarchy, Inspector and chip", [=]() -> bool {
    int selected_rows = 0;
    for (QTreeWidgetItemIterator it(window->scene_hierarchy_tree_); *it; ++it) if ((*it)->isSelected()) ++selected_rows;
    require(selected_rows == 4, QString("the hierarchy shows %1 selected rows").arg(selected_rows));
    require(window->inspector_multi_selection_label_->isVisible(), "the Inspector does not summarise the selection");
    auto * chip = window->arrange_toolbar_->findChild<QToolButton *>("arrangeSelectionChip");
    require(chip && chip->accessibleName() == "4 selected · ref: Industrial Workbench" && chip->text().startsWith("4 selected · ref: "),
      "selection chip: " + (chip ? chip->text() : QString()));
    s->evidence["chip"] = chip->text();
    s->evidence["inspector_summary"] = window->inspector_multi_selection_label_->text();
    s->evidence["current_selected_id"] = window->current_selected_scene_item_id_;
    return true;
  });
  add("toolbar availability explains itself", [=]() {
    QJsonObject tools;
    for (const auto & tool : {"align_front", "dist_x", "gap", "move", "mount", "flush", "concentric", "group"}) {
      tools[tool] = QJsonObject{{"enabled", action(tool)->isEnabled()}, {"tooltip", action(tool)->toolTip()}};
    }
    s->evidence["toolbar_tools"] = tools;
    require(action("align_front")->isEnabled() && action("dist_x")->isEnabled() && action("mount")->isEnabled(), "arrangement tools are disabled");
    require(!action("flush")->isEnabled() && action("flush")->toolTip().contains("exactly two parts"), "Flush is not explained");
    require(!action("group")->isEnabled() && action("group")->toolTip().contains("not part of the scene contract"), "Group is not explained");
    s->evidence["audit_selection"] = TestMain::ui_tour_audit(window);
    shot("01_multi_selection_toolbar");
    return true;
  });
  add("Delete with four parts selected is refused, not applied to the reference", [=]() {
    auto * remove = window->scene_builder_action(QStringLiteral("layout.remove"));
    require(remove && !remove->isEnabled() && remove->toolTip().contains("one part at a time"), "Delete is not disabled with an explanation");
    window->delete_selected_item();  // the Delete key's command
    for (const auto & id : QStringList{table} + bins) {
      bool present = false;
      for (const auto & p : window->all_scene_preview_items_) present = present || p.id == id;
      require(present && !window->deleted_layout_item_ids_.contains(id), id + " was deleted");
    }
    require(window->selection_set_.size() == 4 && !window->layout_dirty_, "the refused Delete changed the scene or selection");
    s->evidence["multi_delete"] = QJsonObject{{"refused", true}, {"tooltip", remove->toolTip()}};
    return true;
  });
  add_web("Viewer keyboard nudge with four parts selected moves nothing", [=]() {
    if (s->js_pending) return false;
    if (s->sub == 0) {
      js("window.dispatchEvent(new KeyboardEvent('keydown',{code:'KeyA',key:'a',bubbles:true}));"
         "document.querySelector('canvas')?.dispatchEvent(new KeyboardEvent('keydown',{code:'KeyA',key:'a',bubbles:true}));"
         "window.__WORKCELL_EDITOR_API_V1__.getState().dirty");
      s->sub = 1; s->ticks = 0; return false;
    }
    if (s->ticks < 15) return false;
    require(same(s->before, 0.) && !window->layout_dirty_, "a keyboard nudge moved a part of the multi-selection");
    s->evidence["viewer_nudge_blocked"] = true;
    return true;
  });
  add("Align front: preview ghosts, then Cancel", [=]() {
    if (s->sub == 0) {
      // Baseline at the moment the tool starts (the Product View may refine poses after load).
      const auto at_start = poses(QStringList{table} + bins);
      if (!same(s->before, 0.)) s->evidence["refined_after_load"] = true;
      s->before = at_start;
      s->env_hash = file_hash(s->scene + "/environment.yaml");
      s->layout_hash = file_hash(s->scene + "/layout/workcell_studio_layout.yaml");
      action("align_front")->trigger();
      require(window->arrange_active() && arrange()->preview().ok, "Align front did not preview: " + arrange()->preview().error);
      require(arrange()->preview().ghost.size() == 3, QString("expected 3 ghosts, got %1").arg(arrange()->preview().ghost.size()));
      if (!(files_unchanged() && same(s->before, 0.))) {
        // Something other than the preview moved a part before the tool started: record it, rebaseline.
        s->evidence["state_changed_before_preview"] = true;
      }
      s->evidence["align_preview"] = arrange()->evidence();
      s->sub = 1; s->ticks = 0; return false;
    }
    if (s->sub == 1) { if (s->ticks < 12) return false; shot("02_align_front_preview"); s->sub = 2; return false; }
    window->arrange_toolbar_->findChild<QPushButton *>("arrangeCancel")->click();
    QStringList moved;
    for (auto it = s->before.constBegin(); it != s->before.constEnd(); ++it) {
      const auto now = pose(it.key());
      for (int k = 0; k < 6; ++k) if (now[k] != it.value()[k]) { moved << QStringLiteral("%1[%2] %3 -> %4").arg(it.key()).arg(k).arg(it.value()[k], 0, 'g', 17).arg(now[k], 0, 'g', 17); break; }
    }
    s->evidence["cancel_check"] = QJsonObject{{"moved", QJsonArray::fromStringList(moved)}, {"files_unchanged", files_unchanged()},
      {"layout_dirty", window->layout_dirty_}};
    require(!window->arrange_active() && moved.isEmpty() && files_unchanged(), "Cancel changed the scene: " + moved.join("; ") +
      (files_unchanged() ? QString() : QStringLiteral(" (scene files changed)")));
    require(window->selection_set_.size() == 4, "Cancel lost the selection");
    return true;
  });
  add("Align front: Apply is one exact transaction", [=]() {
    action("align_front")->trigger();
    const auto request = arrange()->preview().request;
    const int undo_before = static_cast<int>(window->undo_stack_.size());
    window->arrange_toolbar_->findChild<QPushButton *>("arrangeApply")->click();
    require(!window->arrange_active(), "Apply did not finish");
    require(static_cast<int>(window->undo_stack_.size()) == undo_before + 1, "Apply did not create exactly one undo step");
    replan_moves_nothing(request, "align_front");
    s->aligned = poses(QStringList{table} + bins);
    require(same({{table, s->before.value(table)}}, 0.), "the reference moved");
    return true;
  });
  add("Undo and Redo the whole alignment in one step, keeping the selection", [=]() {
    window->undo_layout_edit();
    require(same(s->before, 1e-12), "Undo did not restore every part");
    require(window->selection_set_.size() == 4 && window->selection_set_.reference() == table, "Undo lost the selection");
    window->redo_layout_edit();
    require(same(s->aligned, 1e-12), "Redo did not reapply every part");
    require(window->selection_set_.size() == 4, "Redo lost the selection");
    return true;
  });
  add("Checks reports the generated handoff and validation as Stale after an edit", [=]() {
    QJsonObject statuses;
    for (const auto & step : window->scene_workflow_steps()) statuses[step.label] = window->scene_workflow_status_text(step.status);
    s->evidence["checks_after_edit"] = statuses;
    require(window->layout_dirty_, "the alignment did not mark the layout dirty");
    require(statuses.value("Generate YAML").toString() == "Stale", "Generate YAML is not Stale after an edit: " + statuses.value("Generate YAML").toString());
    require(statuses.value("Save Layout").toString() != "Done", "Save Layout shows Done with unsaved edits");
    return true;
  });
  add("Align left with ΔX +30 mm and Keep relative mounts every member exactly", [=]() {
    action("align_left")->trigger();
    auto * offset = window->arrange_toolbar_->findChild<QDoubleSpinBox *>("arrangeAlignOffsetX");
    auto * keep = window->arrange_toolbar_->findChild<QCheckBox *>("arrangeKeepRelative");
    require(offset && offset->isVisible() && keep && keep->isVisible(), "align offset / keep relative are not shown");
    offset->setValue(30.);
    keep->setChecked(true);
    require(arrange()->preview().ok, "align+offset preview failed: " + arrange()->preview().error);
    const auto request = arrange()->preview().request;
    window->arrange_toolbar_->findChild<QPushButton *>("arrangeApply")->click();
    replan_moves_nothing(QJsonObject{{"operation", "arrange"}, {"op", "align"}, {"reference", table},
      {"members", request.value("members")}, {"params", QJsonObject{{"rules", QJsonObject{{"x", "min"}}},
      {"offset_m", QJsonArray{0.03, 0., 0.}}}}}, "align_left_offset");
    for (const auto & id : bins) {
      QString relation;
      for (const auto & p : window->all_scene_preview_items_) if (p.id == id) relation = p.mounting_yaml;
      require(YAML::Load(relation.toStdString())["parent_object"].as<std::string>("") == table.toStdString(), id + " is not kept relative");
    }
    window->undo_layout_edit();
    require(same(s->aligned, 1e-12), "Undo did not revert align+keep relative in one step");
    return true;
  });
  add("Distribute along X with an exact 60 mm gap", [=]() {
    if (s->sub == 0) {
      action("gap")->trigger();
      require(window->arrange_active(), "the gap tool did not open");
      auto * gap = window->arrange_toolbar_->findChild<QDoubleSpinBox *>("arrangeGap");
      require(gap && gap->isVisible(), "the gap field is not shown");
      gap->setValue(60.);
      require(arrange()->preview().ok, "gap preview failed: " + arrange()->preview().error);
      s->evidence["gap_preview"] = arrange()->evidence();
      s->sub = 1; s->ticks = 0; return false;
    }
    if (s->ticks < 12) return false;
    shot("03_gap_preview");
    const auto request = arrange()->preview().request;
    window->arrange_toolbar_->findChild<QPushButton *>("arrangeApply")->click();
    replan_moves_nothing(request, "gap_60mm");
    s->distributed = poses(QStringList{table} + bins);
    return true;
  });
  add("Applied arrangement shown", [=]() { if (s->ticks < 15) return false; shot("04_applied"); return true; });
  add("Save Layout", [=]() {
    if (s->sub == 0) { window->save_layout_button_->click(); s->sub = 1; s->ticks = 0; return false; }
    if (s->ticks < 10 || window->layout_dirty_) return false;
    QFile::remove(output + "/layout_after_save.yaml");
    QFile::copy(s->scene + "/layout/workcell_studio_layout.yaml", output + "/layout_after_save.yaml");
    QFile::remove(output + "/environment_after_save.yaml");
    QFile::copy(s->scene + "/environment.yaml", output + "/environment_after_save.yaml");
    return true;
  });
  add("close and reopen", [=]() { if (s->ticks < 5) return false; open_cell(); return true; });
  add("wait for the reopened Product View", wait_view("reopen"));
  add("reopened placements are identical", [=]() {
    double worst = 0.;
    for (auto it = s->distributed.constBegin(); it != s->distributed.constEnd(); ++it) {
      const auto now = pose(it.key());
      for (int k = 0; k < 6; ++k) worst = std::max(worst, std::abs(now[k] - it.value()[k]));
    }
    s->evidence["reopen_max_abs_error"] = worst;
    require(worst < 1e-9, "reopened poses differ by " + QString::number(worst));
    return true;
  });
  add("Fixed mount is atomic and the bins follow the workbench", [=]() {
    window->apply_scene_selection(table, QString(), false, false);
    window->toggle_scene_selection(bin);
    window->toggle_scene_selection(reject);
    action("mount")->trigger();
    require(arrange()->preview().ok, "mount preview failed: " + arrange()->preview().error);
    window->arrange_toolbar_->findChild<QPushButton *>("arrangeApply")->click();
    for (const auto & id : {bin, reject}) {
      QString relation;
      for (const auto & p : window->all_scene_preview_items_) if (p.id == id) relation = p.mounting_yaml;
      require(YAML::Load(relation.toStdString())["parent_object"].as<std::string>("") == table.toStdString(),
        id + " is not mounted on the workbench");
    }
    const auto mounted = poses({bin, reject});
    const auto t = pose(table);
    require(window->commit_mounting_request({{"id", table}, {"world_pose", QJsonObject{{"xyz", QJsonArray{t[0] + 0.1, t[1], t[2]}},
      {"rpy", QJsonArray{t[3], t[4], t[5]}}}}}), "moving the workbench failed");
    for (const auto & id : {bin, reject}) require(std::abs(pose(id)[0] - mounted.value(id)[0] - 0.1) < 1e-9, id + " did not follow");
    window->undo_layout_edit();  // the workbench move
    action("detach")->trigger();
    require(arrange()->preview().ok, "detach preview failed: " + arrange()->preview().error);
    window->arrange_toolbar_->findChild<QPushButton *>("arrangeApply")->click();
    for (const auto & id : {bin, reject}) {
      QString relation;
      for (const auto & p : window->all_scene_preview_items_) if (p.id == id) relation = p.mounting_yaml;
      require(YAML::Load(relation.toStdString())["parent_object"].as<std::string>("world") == "world", id + " is still mounted");
      require(std::abs(pose(id)[0] - mounted.value(id)[0]) < 1e-9, id + " moved on detach");
    }
    window->undo_layout_edit();  // detach
    window->undo_layout_edit();  // mount
    require(same(s->distributed, 1e-9), "two undos did not restore the saved arrangement");
    return true;
  });
  add("narrow view: tools stay reachable through the overflow menu", [=]() {
    auto * toolbar = window->arrange_toolbar_;
    int visible = 0, total = 0;
    for (auto * button : toolbar->tools()->findChildren<QToolButton *>()) { ++total; visible += button->isVisible() ? 1 : 0; }
    const int in_menu = toolbar->more_button()->menu()->actions().size();
    s->evidence["toolbar_overflow"] = QJsonObject{{"tools_width", toolbar->tools()->width()}, {"visible_tool_buttons", visible},
      {"tool_buttons", total}, {"more_visible", toolbar->more_button()->isVisible()}, {"more_menu_entries", in_menu}};
    require(visible == total || (toolbar->more_button()->isVisible() && in_menu > 0), "hidden tools without the More menu");
    require(visible > 0, "no arrangement tool is visible");
    s->evidence["audit_final"] = TestMain::ui_tour_audit(window);
    return true;
  });
  add_web("Mate: rim and surface chosen, then a missed click keeps both picks", [=]() {
    if (s->js_pending) return false;
    if (s->sub == 0) {
      window->apply_scene_selection(bin, QString(), false, false);
      window->begin_visual_mate();
      QString error;
      require(window->mate_active(), "Mate did not start");
      require(window->mate_controller_->choose_datum(workcell_builder::MateToolController::Role::Child, bin, "rim", &error), error);
      require(window->mate_controller_->choose_datum(workcell_builder::MateToolController::Role::Parent, table, "surface", &error), error);
      s->sub = 1; s->ticks = 0; return false;
    }
    if (s->sub == 1) { if (s->ticks < 10) return false; js("window.__WORKCELL_EDITOR_API_V1__.projectWorldPoint([0,0,0])"); s->sub = 2; return false; }
    if (s->sub == 2) {
      const auto canvas = s->js_result.toMap().value("canvas").toMap();
      require(!canvas.isEmpty() && s->corner < 4, "no background point produced a missed click");
      const double fractions[4][2] = {{0.12, 0.18}, {0.88, 0.18}, {0.5, 0.14}, {0.12, 0.85}};
      const QPoint point(qRound(canvas.value("left").toDouble() + canvas.value("width").toDouble() * fractions[s->corner][0]),
        qRound(canvas.value("top").toDouble() + canvas.value("height").toDouble() * fractions[s->corner][1]));
      s->pick_seq_before = window->mate_seen_pick_seq_;
      mouse_click(point, Qt::NoModifier);
      s->sub = 3; s->ticks = 0; return false;
    }
    if (window->mate_seen_pick_seq_ <= s->pick_seq_before) {
      if (s->ticks < 30) return false;
      ++s->corner; s->sub = 1; s->ticks = 10; return false;
    }
    const auto picks = window->mate_controller_->evidence().value("picks").toArray();
    const auto last = picks.last().toObject().value("resolution").toObject();
    if (last.value("reason_code").toString() != "NO_ASSET") { ++s->corner; s->sub = 1; s->ticks = 10; return false; }
    require(window->mate_active() && window->mate_controller_->step() == workcell_builder::MateToolController::Step::Ready,
      "a missed click cancelled Mate or dropped a pick");
    require(window->mate_controller_->status().contains("missed every part"), "the missed click is not explained");
    s->evidence["mate_missed_click"] = window->mate_controller_->status();
    window->mate_controller_->cancel();
    return true;
  });
  if (!airpick_scene.isEmpty()) {
    add("Switching cells with unsaved edits asks first; Stay keeps everything", [=]() {
      if (s->sub == 0) {
        const auto b = pose(bin);
        require(window->commit_mounting_request({{"id", bin}, {"world_pose", QJsonObject{{"xyz", QJsonArray{b[0] + 0.01, b[1], b[2]}},
          {"rpy", QJsonArray{b[3], b[4], b[5]}}}}}), "the edit before switching failed");
        require(window->layout_dirty_, "the edit did not mark the layout dirty");
        s->expect_switch_dialog = true; s->switch_choice = QStringLiteral("Stay");
        QStringList blockers; QJsonObject diagnostics;
        const bool opened = window->load_scene_for_scene3d_smoke(airpick_scene,
          window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/" + airpick_scene, &blockers, &diagnostics);
        require(!opened && window->layout_dirty_ && !window->undo_stack_.empty(), "Stay did not keep the cell and its edits");
        require(window->selected_scene_state_.path == s->scene, "Stay opened the other cell");
        s->evidence["switch_stay"] = QJsonObject{{"kept_cell", true}, {"undo_steps", static_cast<int>(window->undo_stack_.size())}};
        return true;
      }
      return true;
    });
    add("Discard opens AirPick with a clean session (no undo, no selection, not dirty)", [=]() {
      s->expect_switch_dialog = true; s->switch_choice = QStringLiteral("Discard edits and open");
      QStringList blockers; QJsonObject diagnostics;
      s->scene = window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/" + airpick_scene;
      s->env_hash = file_hash(s->scene + "/environment.yaml");
      s->layout_hash = file_hash(s->scene + "/layout/workcell_studio_layout.yaml");
      require(window->load_scene_for_scene3d_smoke(airpick_scene, s->scene, &blockers, &diagnostics), "AirPick did not open: " + blockers.join("; "));
      require(window->undo_stack_.empty() && window->redo_stack_.empty() && window->selection_set_.isEmpty() && !window->layout_dirty_,
        "the previous cell's undo history, selection or dirty state leaked into AirPick");
      return true;
    });
    add_web("AirPick: read-only preview, Save disabled everywhere", [=]() {
      auto * preview = window->scene_preview_widget_;
      if (!preview->property("diagnostic_preview_active").toBool()) { require(s->ticks < 800, "AirPick did not reach the read-only preview"); return false; }
      if (s->ticks < 60) return false;
      require(!window->save_layout_button_->isEnabled(), "Save Layout is enabled in the read-only preview");
      window->save_layout_changes();  // Ctrl+S
      window->scene_builder_action(QStringLiteral("layout.save"))->trigger();  // menu entry
      QCoreApplication::processEvents();
      require(file_hash(s->scene + "/environment.yaml") == s->env_hash &&
        file_hash(s->scene + "/layout/workcell_studio_layout.yaml") == s->layout_hash, "a save path wrote AirPick's files");
      QJsonObject statuses;
      for (const auto & step : window->scene_workflow_steps()) statuses[step.label] = window->scene_workflow_status_text(step.status);
      s->evidence["airpick"] = QJsonObject{{"read_only", true}, {"save_enabled", window->save_layout_button_->isEnabled()},
        {"status_message", window->statusBar()->currentMessage()}, {"checks", statuses}};
      shot("05_airpick_read_only");
      return true;
    });
    add_web("AirPick settled: viewer shows AirPick, Save still disabled, no leaked viewer edits", [=]() {
      if (s->sub == 2) {
        // Same view 5 s later: screen pixels (X11) and the web view's own frame, to measure display lag.
        if (s->ticks < 50) return false;
        shot("07_airpick_settled_plus5s");
        if (auto * view = web()) view->grab().save(output + "/07_airpick_settled_plus5s_webview.png");
        return true;
      }
      if (s->js_pending) return false;
      if (s->sub == 0) {
        js("(() => { const st = window.__WORKCELL_EDITOR_API_V1__ ? window.__WORKCELL_EDITOR_API_V1__.getState() : {};"
           " const vs = window.__WORKCELL_VIEWER_STATUS__ || {};"
           " return {ready: Boolean(st.ready), sceneId: String(st.sceneId || ''), dirty: Boolean(st.dirty), dirtyCount: Number(st.dirtyCount || 0),"
           " terminal: Boolean(vs.terminal), lifecycle: String(vs.lifecycle_state || ''), statusSceneId: String(vs.scene_id || ''),"
           " statusSequence: Number(vs.status_sequence || 0), pending: (vs.pending_required_loads || []).map(String),"
           " pill: String((document.querySelector('.scene-health') || {}).textContent || '').trim().slice(0, 160)}; })()");
        s->sub = 1; return false;
      }
      const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
      s->sub = 0;
      // "ready" only means the scene JSON is parsed; wait for the viewer's terminal lifecycle state (drawn or failed).
      const bool settled = state.value("ready").toBool() && state.value("terminal").toBool() &&
        state.value("sceneId").toString() == airpick_scene && state.value("pending").toArray().isEmpty();
      if (!settled && s->ticks < 1500) return false;  // bounded: record the state either way
      s->evidence["airpick_settled_within_limit"] = settled;
      QJsonArray header;
      for (auto * label : window->findChildren<QLabel *>()) {
        if (!label->isVisible()) continue;
        const QPoint top = label->mapTo(window, QPoint(0, 0));
        if (top.y() < 40 && !label->text().trimmed().isEmpty()) header.append(label->text().left(80));
      }
      // What the user sees, not just isEnabled(): a disabled Save must not be painted as the active (accent) button.
      const QImage save_pixels = window->save_layout_button_->grab().toImage();
      double r = 0, g = 0, b = 0; int n = 0;
      for (int y = 2; y < save_pixels.height() - 2; y += 2) for (int x = 2; x < save_pixels.width() - 2; x += 2) {
        const QColor c = save_pixels.pixelColor(x, y); r += c.red(); g += c.green(); b += c.blue(); ++n;
      }
      if (n) { r /= n; g /= n; b /= n; }
      s->evidence["airpick_settled"] = QJsonObject{{"viewer", state}, {"save_enabled", window->save_layout_button_->isEnabled()},
        {"save_layout_dirty_property", window->save_layout_button_->property("layoutDirty").toBool()},
        {"save_mean_rgb", QJsonArray{r, g, b}}, {"layout_dirty", window->layout_dirty_}, {"header_labels", header}};
      shot("06_airpick_settled");
      require(!window->save_layout_button_->isEnabled(), "Save Layout became enabled after the AirPick view loaded");
      require(b - r < 60.0, "Save Layout is disabled but painted as the active accent button");
      const auto title = window->findChild<QLabel *>(QStringLiteral("studioContextSceneTitle"));
      require(!title || title->toolTip() == airpick_scene, "the header still names the previous cell: " + (title ? title->text() : QString()));
      require(state.value("dirtyCount").toInt() == 0, "the Product View carried edits from the previous cell into AirPick");
      s->sub = 2; s->ticks = 0;
      return false;
    });
  }
  add("finish", [=]() { s->evidence["result"] = "PASS"; write(); return true; });

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
      if (window->arrange_controller_) s->evidence["arrange_state"] = window->arrange_controller_->evidence();
      s->evidence["selection"] = QJsonObject{{"members", QJsonArray::fromStringList(window->selection_set_.members())},
        {"reference", window->selection_set_.reference()}};
      shot("failure"); write(); timer->stop(); app->exit(1);
    }
  });
  timer->start();
}
