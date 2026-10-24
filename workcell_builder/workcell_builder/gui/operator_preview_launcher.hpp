#pragma once

// Owns the Operator HMI child process for Workcell Builder. The HMI receives a
// request file in a private temporary directory (never inside the scene), is
// started with an argument list (no shell), and is stopped with the Builder.

#include <QObject>
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

signals:
  void state_changed(const QString & state, const QString & detail);

private:
  bool write_request(OperatorPreviewRequest & request, bool activate, QString * error);

  QProcess * process_{nullptr};
  std::unique_ptr<QTemporaryDir> directory_;
  int revision_{0};
  QString hmi_program_override_;
  QStringList extra_arguments_;
};

}  // namespace workcell_builder
