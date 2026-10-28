#pragma once

// Operator HMI <-> Workcell Builder simulation link (R2.3).
//
// Workcell Builder owns every simulation: it authorises, starts, supervises and cancels the existing qualified
// physics runner. The managed Operator HMI only asks:
//
//   HMI -> Builder  one line on the HMI's stdout, "WORKCELL_OPERATOR_COMMAND <json>". The HMI is the Builder's own
//                   child process, so only the Builder reads that pipe; nothing else can inject a command.
//   Builder -> HMI  operator_simulation_status.json, written atomically beside the private preview request.
//
// A command carries the control token of the request it was made from (a fresh random token per Builder launcher),
// the cell, the saved task's normalized intent SHA-256 that the HMI showed, the request revision it showed and, to
// cancel, the session id. The Builder accepts it only if all of them still match its own current state; a
// standalone HMI has no token and can never start anything.

#include <QByteArray>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

namespace workcell_builder
{

inline const char * operator_command_prefix() { return "WORKCELL_OPERATOR_COMMAND "; }
inline const char * operator_command_schema() { return "workcell_operator_simulation_command/v1"; }
inline const char * operator_status_schema() { return "workcell_operator_simulation_status/v1"; }
inline QString operator_status_file(const QString & request_path)
{
  return QFileInfo(request_path).dir().filePath("operator_simulation_status.json");
}

inline QJsonObject make_operator_command(int id, const QString & action, const QString & token, const QString & scene_dir,
  const QString & intent_sha256, int request_revision, const QString & session_id = {})
{
  return QJsonObject{{"schema", operator_command_schema()}, {"id", id}, {"action", action}, {"control_token", token},
    {"scene_dir", scene_dir}, {"intent_sha256", intent_sha256}, {"request_revision", request_revision},
    {"session_id", session_id}};
}

inline QByteArray operator_command_line(const QJsonObject & command)
{
  return QByteArray(operator_command_prefix()) + QJsonDocument(command).toJson(QJsonDocument::Compact) + "\n";
}

// A stdout line of the HMI: the command it carries, or false for any other output.
inline bool parse_operator_command_line(const QByteArray & line, QJsonObject * command)
{
  const QByteArray prefix(operator_command_prefix());
  if (!line.startsWith(prefix)) return false;
  const auto document = QJsonDocument::fromJson(line.mid(prefix.size()).trimmed());
  if (!document.isObject() || document.object().value("schema").toString() != operator_command_schema()) return false;
  *command = document.object();
  return true;
}

// The Builder's own state a command is checked against.
struct OperatorLinkState
{
  QString control_token;      // the launcher's token (never empty while a managed HMI runs)
  QString scene_dir;          // canonical path of the cell selected in the Builder
  QString intent_sha256;      // normalized intent SHA-256 of the saved task the cell was generated from
  int request_revision{-1};   // latest request the Builder published to the HMI
  bool session_active{false}; // a Builder-owned planning or simulation session is running
  QString session_id;         // id of the running physics or sorting session, if any
  bool cancel_requested{false};
  bool sorting_session{false}; // the running session is an R2.4 sorting session
};

struct OperatorCommandCheck
{
  bool accepted{false};
  QString reason;             // why it was refused (shown to the operator)
};

inline OperatorCommandCheck check_operator_command(const QJsonObject & command, const OperatorLinkState & state)
{
  const auto refuse = [](const QString & reason) { return OperatorCommandCheck{false, reason}; };
  const QString action = command.value("action").toString();
  if (state.control_token.isEmpty() || command.value("control_token").toString() != state.control_token)
    return refuse("not from the Operator HMI this Workcell Builder opened");
  const auto canonical = [](const QString & path) {
    const QString c = QFileInfo(path).canonicalFilePath();
    return c.isEmpty() ? QDir::cleanPath(path) : c;
  };
  if (state.scene_dir.isEmpty() || canonical(command.value("scene_dir").toString()) != canonical(state.scene_dir))
    return refuse("the HMI shows another cell than the one selected in Workcell Builder; Refresh the HMI");
  if (command.value("request_revision").toInt(-2) != state.request_revision)
    return refuse("Workcell Builder has newer engineering state; Refresh the HMI before starting");
  if (action == "run_simulation" || action == "start_sorting") {
    if (state.session_active) return refuse("a session is already running; one simulation at a time");
    if (state.intent_sha256.isEmpty() || command.value("intent_sha256").toString() != state.intent_sha256)
      return refuse("the saved task shown in the HMI is not the one the cell was generated from; Save, Generate and Refresh");
    return {true, {}};
  }
  if (action == "cancel_simulation") {
    if (!state.session_active || state.session_id.isEmpty()) return refuse("no simulation is running");
    if (command.value("session_id").toString() != state.session_id) return refuse("that simulation session is no longer running");
    if (state.cancel_requested) return refuse("cancellation was already requested");
    return {true, {}};
  }
  if (action == "pause_sorting" || action == "resume_sorting" || action == "stop_sorting") {
    // Applied by the session at its next cycle boundary; a stop is a software stop, never an emergency stop.
    if (!state.session_active || state.session_id.isEmpty() || !state.sorting_session) return refuse("no sorting session is running");
    if (command.value("session_id").toString() != state.session_id) return refuse("that sorting session is no longer running");
    return {true, {}};
  }
  return refuse("unknown action " + action);
}

}  // namespace workcell_builder
