#pragma once
#include <QGroupBox>
#include <QJsonObject>
#include <QMap>
#include <functional>
#include "task_intent_model.hpp"
class QFormLayout;
class QLabel;
class QComboBox;
class QSpinBox;
class QTableWidget;
class QTimer;
class QProcess;

class TaskIntentEditor : public QGroupBox
{
  Q_OBJECT
public:
  explicit TaskIntentEditor(QWidget * parent = nullptr);
  ~TaskIntentEditor() override;
  bool confirm_scene_change(const QString & scene);
  bool load_scene(const QString & scene, const QString & helper);
  bool save(QString * error = nullptr);
  void validate_now();
  void bind_pick(const QString & id);
  void bind_destination(const QString & id);
  bool dirty() const { return dirty_; }
  QString scene() const { return scene_; }
  QString blocker(bool require_input = true) const;
  const workcell_builder::TaskIntentModel & model() const { return model_; }
  // The validator's resolved destination for the saved environment and current draft (empty when blocked).
  QJsonObject resolved_destination() const { return report_["resolved_destination"].toObject(); }
  // Physical-destination role of a scene item (semantic or layout ID) from the saved environment.yaml:
  // {kind: target|region, target, target_layout, regions[]}; empty when the item is neither.
  QJsonObject physical_destination(const QString & id) const;
  QString destination_summary() const;
  // Product View has moved geometry that is not saved yet: the resolved destination is the saved one.
  void set_geometry_unsaved(bool unsaved);
signals:
  void edited();
  void saved();
  void destination_changed();
  void show_target_requested(const QString & target_layout_id);
private:
  void start_plan_check(bool sequence = false);
  void on_plan_check_output();
  void finish_plan_check(int exit_code, bool normal_exit);
  bool stop_plan_check(int wait_ms);
  void set_field(const QString & path, const QJsonValue & value);
  void refresh_fields();
  QJsonObject request(bool draft);
  void apply_report(const QJsonObject & report);
  void text_field(QFormLayout *, const QString &, const QString &, const QString &);
  void number_field(QFormLayout *, const QString &, const QString &, const QString &, const QString &, bool nullable = false);
  QComboBox * choice_field(QFormLayout *, const QString &, const QString &, const QString &, const QStringList &);
  void vector_field(QFormLayout *, const QString &, const QString &, const QString &, int size = 3);
  // Object input guidance: existing recordings for the selected mode, and the saved input's identity,
  // freshness and planning eligibility as the authoring helper (planning_input) reports it.
  void refresh_recordings();
  void refresh_input_status();
  void refresh_destination_summary();
  void follow_mode_freshness(const QJsonObject & report);
  QString stored_input_path(const QString & path) const;
  QString producer_blocker(const QJsonObject & report) const;
  bool age_follows_mode_{false}, clamping_age_{false}, geometry_unsaved_{false};
  // physical_destination() index of the saved environment.yaml, rebuilt when the file changes.
  mutable QString destinations_key_;
  mutable QMap<QString, QJsonObject> destinations_;
  QString input_note_;
  workcell_builder::TaskIntentModel model_;
  QString scene_, helper_, loaded_bytes_, error_, input_blocker_;
  bool loading_{false}, dirty_{false};
  QJsonObject report_;
  QMap<QString, std::function<void(const QJsonValue &)>> readers_;
  QMap<QString, QString> input_errors_;
  QLabel * status_;
  QTimer * validation_timer_;
  QComboBox * strategies_;
  QComboBox * assets_;
  QComboBox * regions_;
  QComboBox * pick_zones_;
  // R2.4 class routing table (routing.mode class): class, destination region, enabled, priority.
  QTableWidget * routing_table_{nullptr};
  QComboBox * unrouted_policy_{nullptr};
  QSpinBox * max_attempts_{nullptr};
  QStringList region_ids_, vocabulary_;
  bool routing_loading_{false};
  void add_routing_row(const QJsonObject & rule);
  void fill_routing(const QJsonObject & routing);
  void write_routing();
  // Feature 4B: owned plan-only batch check (fake hardware, never executes).
  QProcess * plan_check_{nullptr};
  QString plan_check_scene_, plan_check_task_bytes_;
  QByteArray plan_check_buffer_;
  QJsonObject plan_check_report_;
  QStringList plan_check_lines_;
  int plan_check_total_{0};
  bool plan_check_sequence_{false};
  void clear_sequence_results(const QString & why);
  // Feature 4E: the pick sequence saved in the task (sequence block), edited in the Task tab.
  void rebuild_sequence(const QJsonObject & report);
  void apply_sequence_review(const QJsonObject & review);
  void refresh_sequence_texts();
  void show_selected_place();
  void write_sequence();
  void update_dirty_ui();
public:
  void revert();
private:
  QString sequence_source_sha_;
  bool sequence_loading_{false}, sequence_rebind_pending_{false};
};
