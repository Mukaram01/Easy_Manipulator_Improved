#pragma once

// Owns the Operator HMI child process for Workcell Builder. The HMI receives a
// request file in a private temporary directory (never inside the scene), is
// started with an argument list (no shell), and is stopped with the Builder.

#include <QObject>
#include <QJsonObject>
#include <QString>
#include <QTemporaryDir>

#include <memory>

#include "operator_preview_request.hpp"

class QProcess;

namespace workcell_builder
{

class OperatorPreviewLauncher : public QObject
{
  Q_OBJECT

public:
  explicit OperatorPreviewLauncher(QObject * parent = nullptr);
  ~OperatorPreviewLauncher() override;

  // Opens the HMI, or refreshes and raises a running one.
  bool open_or_refresh(OperatorPreviewRequest request, QString * error);
  // Publishes newer engineering state to a running HMI without raising it.
  void publish_update(OperatorPreviewRequest request);
  bool running() const;
  void shutdown(int timeout_ms = 4000);

  QString request_path() const;
  QString hmi_program() const;
  void set_hmi_program(const QString & program) { hmi_program_override_ = program; }
  void set_extra_arguments(const QStringList & arguments) { extra_arguments_ = arguments; }
  qint64 process_id() const;
  // Simulation link (operator_simulation_link.hpp): the token every request carries, the latest request revision,
  // and the Builder-owned status the HMI shows. Commands arrive only on this child's stdout.
  QString control_token() const { return control_token_; }
  int latest_revision() const { return revision_; }
  void publish_simulation_status(const QJsonObject & status);

signals:
  void state_changed(const QString & state, const QString & detail);
  void simulation_command(const QJsonObject & command);

private:
  bool write_request(OperatorPreviewRequest & request, bool activate, QString * error);

  void read_hmi_output();

  QProcess * process_{nullptr};
  QString control_token_;
  QByteArray output_buffer_;
  QByteArray last_status_;
  std::unique_ptr<QTemporaryDir> directory_;
  int revision_{0};
  QString hmi_program_override_;
  QStringList extra_arguments_;
};

}  // namespace workcell_builder
