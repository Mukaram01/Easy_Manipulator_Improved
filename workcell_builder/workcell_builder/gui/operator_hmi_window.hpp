#pragma once

// Workcell Operator HMI (preview). A separate process that consumes the
// engineering configuration read-only. It may run the offline, no-motion checks
// the operator role permits; it never edits the scene and never commands motion.

#include <QJsonArray>
#include <QJsonObject>
#include <QMainWindow>
#include <QPointer>
#include <QString>
#include <QTemporaryDir>

#include <memory>
#include <optional>

#include "operator_hmi_model.hpp"
#include "operator_preview_request.hpp"

class QFileSystemWatcher;
class QFrame;
class QLabel;
class QPlainTextEdit;
class QProcess;
class QPushButton;
class QTableWidget;
class QTimer;

namespace workcell_builder
{

class CellMapWidget;

struct OperatorHmiOptions
{
  QString request_path;  // managed preview: the Builder's request file
  QString scene_dir;     // standalone: open this scene's saved revision
  QString scripts_dir;   // overrides script discovery
  QString status_out;    // optional machine-readable status for acceptance runs
  QString acceptance_commands;  // test-only: JSON command list that clicks real HMI controls
  QString capture_dir;   // test-only: where "capture" commands save window grabs
};

class OperatorHmiWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit OperatorHmiWindow(const OperatorHmiOptions & options, QWidget * parent = nullptr);
  ~OperatorHmiWindow() override;

  void start();
  void refresh_preview(bool evaluate_input);

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  void build_ui();
  QFrame * build_header();
  QWidget * build_overview();
  QWidget * build_monitoring();
  QWidget * build_controls();
  QWidget * build_events();
  void apply_style();

  void check_request_file();
  void apply_request(const OperatorPreviewRequest & request);
  QString resolve_script(const QString & name) const;

  void projection_finished(int generation, int exit_code);
  void render();
  void render_badges();
  void render_overview();
  void render_monitoring();
  void render_controls();
  void render_selected_object();
  void run_gated_dry_run();
  void dry_run_finished(int exit_code);
  void stop_checks();
  void set_busy(const QString & activity);
  void log_event(const QString & level, const QString & code, const QString & message);
  void write_status() const;
  void poll_acceptance_commands();

  OperatorHmiOptions options_;
  std::optional<OperatorPreviewRequest> request_;
  int pending_revision_{-1};
  QString scene_dir_;
  OperatorPreviewView view_;
  bool view_loaded_{false};
  bool replay_shown_{false};
  QJsonObject projection_raw_;
  GatedDryRunResult dry_run_;
  QJsonObject dry_run_raw_;
  QString busy_;

  QProcess * projection_process_{nullptr};
  QProcess * dry_run_process_{nullptr};
  int projection_generation_{0};
  bool projection_wants_input_{false};
  QTimer * projection_timeout_{nullptr};
  std::unique_ptr<QTemporaryDir> dry_run_dir_;
  QFileSystemWatcher * request_watcher_{nullptr};
  QTimer * request_poll_{nullptr};
  QTimer * command_poll_{nullptr};
  int last_command_{0};
  QJsonArray command_results_;

  // Header
  QLabel * title_{nullptr};
  QLabel * subtitle_{nullptr};
  QLabel * revision_badge_{nullptr};
  QLabel * approval_badge_{nullptr};
  QLabel * mode_badge_{nullptr};
  QLabel * readiness_badge_{nullptr};
  QLabel * production_badge_{nullptr};
  QPushButton * refresh_button_{nullptr};
  QFrame * update_banner_{nullptr};
  QLabel * update_banner_text_{nullptr};
  QLabel * draft_notice_{nullptr};

  // Overview
  QLabel * cell_values_{nullptr};
  QLabel * task_values_{nullptr};
  QLabel * validation_values_{nullptr};

  // Monitoring
  CellMapWidget * map_{nullptr};
  QLabel * map_caption_{nullptr};
  QTableWidget * objects_{nullptr};
  QLabel * object_detail_{nullptr};
  QLabel * robot_status_{nullptr};
  QLabel * cycle_status_{nullptr};

  // Controls
  QLabel * recipe_summary_{nullptr};
  QPushButton * check_button_{nullptr};
  QPushButton * replay_button_{nullptr};
  QLabel * replay_note_{nullptr};
  QPushButton * dry_run_button_{nullptr};
  QLabel * dry_run_note_{nullptr};
  QLabel * monitor_status_{nullptr};
  QPushButton * stop_button_{nullptr};
  QLabel * results_summary_{nullptr};
  QPushButton * start_button_{nullptr};
  QLabel * production_blockers_{nullptr};

  // Events
  QTableWidget * events_{nullptr};
  QPlainTextEdit * details_{nullptr};
};

}  // namespace workcell_builder
