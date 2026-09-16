#pragma once
#include <QGroupBox>
#include <QJsonObject>
#include <QMap>
#include <functional>
#include "task_intent_model.hpp"
class QFormLayout;
class QLabel;
class QComboBox;
class QTimer;

class TaskIntentEditor : public QGroupBox
{
  Q_OBJECT
public:
  explicit TaskIntentEditor(QWidget * parent = nullptr);
  bool confirm_scene_change(const QString & scene);
  bool load_scene(const QString & scene, const QString & helper);
  bool save(QString * error = nullptr);
  void validate_now();
  void bind_pick(const QString & id);
  void bind_destination(const QString & id);
  bool dirty() const { return dirty_; }
  QString scene() const { return scene_; }
  QString blocker() const;
  const workcell_builder::TaskIntentModel & model() const { return model_; }
signals:
  void edited();
  void saved();
private:
  void set_field(const QString & path, const QJsonValue & value);
  void refresh_fields();
  QJsonObject request(bool draft);
  void apply_report(const QJsonObject & report);
  void text_field(QFormLayout *, const QString &, const QString &, const QString &);
  void number_field(QFormLayout *, const QString &, const QString &, const QString &, const QString &, bool nullable = false);
  QComboBox * choice_field(QFormLayout *, const QString &, const QString &, const QString &, const QStringList &);
  void vector_field(QFormLayout *, const QString &, const QString &, const QString &, int size = 3);
  workcell_builder::TaskIntentModel model_;
  QString scene_, helper_, loaded_bytes_, error_;
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
};
