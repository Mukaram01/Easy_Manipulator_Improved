#include "operator_preview_request.hpp"

#include <QFile>
#include <QJsonDocument>
#include <QSaveFile>

namespace workcell_builder
{

namespace
{
constexpr auto kSchema = "workcell_operator_preview_request/v1";
}

QJsonObject OperatorPreviewRequest::to_json() const
{
  QJsonObject json{
    {"schema", kSchema}, {"revision", revision}, {"scene_dir", scene_dir},
    {"scene_name", scene_name}, {"workspace_root", workspace_root}, {"scripts_dir", scripts_dir},
    {"task_dirty", task_dirty}, {"layout_dirty", layout_dirty}, {"activate", activate},
    {"builder_pid", static_cast<double>(builder_pid)}, {"requested_at", requested_at}};
  if (task_dirty) json.insert("task_draft", task_draft);
  return json;
}

std::optional<OperatorPreviewRequest> OperatorPreviewRequest::from_json(
  const QJsonObject & json, QString * error)
{
  auto fail = [error](const QString & message) -> std::optional<OperatorPreviewRequest> {
      if (error) *error = message;
      return std::nullopt;
    };
  if (json.value("schema").toString() != kSchema) return fail("Unsupported preview request schema.");
  OperatorPreviewRequest request;
  request.revision = json.value("revision").toInt(-1);
  request.scene_dir = json.value("scene_dir").toString();
  request.scene_name = json.value("scene_name").toString();
  request.workspace_root = json.value("workspace_root").toString();
  request.scripts_dir = json.value("scripts_dir").toString();
  request.task_dirty = json.value("task_dirty").toBool();
  request.layout_dirty = json.value("layout_dirty").toBool();
  request.activate = json.value("activate").toBool();
  request.builder_pid = static_cast<qint64>(json.value("builder_pid").toDouble());
  request.requested_at = json.value("requested_at").toString();
  if (request.revision < 0) return fail("Preview request has no revision.");
  if (request.scene_dir.isEmpty()) return fail("Preview request names no scene.");
  if (request.task_dirty) {
    if (!json.value("task_draft").isObject() || json.value("task_draft").toObject().isEmpty()) {
      return fail("Preview request reports unsaved task edits but carries no draft.");
    }
    request.task_draft = json.value("task_draft").toObject();
  }
  return request;
}

bool write_operator_preview_request(
  const QString & path, const OperatorPreviewRequest & request, QString * error)
{
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = file.errorString();
    return false;
  }
  file.write(QJsonDocument(request.to_json()).toJson(QJsonDocument::Indented));
  if (!file.commit()) {
    if (error) *error = file.errorString();
    return false;
  }
  return true;
}

std::optional<OperatorPreviewRequest> read_operator_preview_request(
  const QString & path, QString * error)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QString("Cannot read preview request %1: %2").arg(path, file.errorString());
    return std::nullopt;
  }
  QJsonParseError parse{};
  const auto document = QJsonDocument::fromJson(file.readAll(), &parse);
  if (parse.error != QJsonParseError::NoError || !document.isObject()) {
    if (error) *error = "Preview request is not valid JSON: " + parse.errorString();
    return std::nullopt;
  }
  return OperatorPreviewRequest::from_json(document.object(), error);
}

}  // namespace workcell_builder
