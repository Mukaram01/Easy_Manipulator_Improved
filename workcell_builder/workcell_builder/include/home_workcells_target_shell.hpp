#pragma once

// Target Workcell Studio shell for the production Home page.
//
// This deliberately reuses the existing scene table, selection lifecycle and
// QAction wiring.  It changes presentation and page composition only.  There
// are no startup functions, event filters, timer-driven mutations, hidden 3D
// renderers, or per-cell widget overlays.

#include <QAbstractItemView>
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMainWindow>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QPointer>
#include <QPushButton>
#include <QSettings>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QStatusBar>
#include <QStyle>
#include <QSignalBlocker>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <vector>

#include "home_workcells_panel.hpp"

namespace workcell_builder
{
namespace home_workcells
{

constexpr int kPinnedRole = Qt::UserRole + 37;
constexpr int kDisplayNameRole = Qt::UserRole + 38;
constexpr int kTaskRole = Qt::UserRole + 39;
constexpr int kReadinessReasonsRole = Qt::UserRole + 40;
constexpr int kFakeHardwareReadyRole = Qt::UserRole + 41;
// MainWindow owns UserRole + 42 for the canonical scene path.
constexpr int kModifiedRole = Qt::UserRole + 43;

inline QString scene_id_at(QTableWidget * table, int row)
{
  if (!table || row < 0 || row >= table->rowCount()) return QString();
  QTableWidgetItem * item = table->item(row, 0);
  if (!item) return QString();
  return item->toolTip().trimmed().isEmpty() ? item->text().trimmed() : item->toolTip().trimmed();
}

inline QString scene_dir_for_id(const QString & workspace_root, const QString & scene_id)
{
  if (workspace_root.trimmed().isEmpty() || scene_id.trimmed().isEmpty()) return QString();
  const QStringList roots = {
    QDir(workspace_root).filePath(QStringLiteral("src/easy_manipulation_deployment/scenes")),
    QDir(workspace_root).filePath(QStringLiteral("src/scenes"))};
  for (const QString & root : roots) {
    const QString candidate = QDir(root).filePath(scene_id);
    if (QFileInfo(candidate).isDir()) return QDir::cleanPath(candidate);
  }
  return QString();
}

inline QDateTime scene_last_updated(const QString & workspace_root, const QString & scene_id)
{
  const QString scene_dir = scene_dir_for_id(workspace_root, scene_id);
  if (scene_dir.isEmpty()) return QDateTime();
  QDateTime latest;
  const QStringList evidence = {
    QStringLiteral("environment.yaml"),
    QStringLiteral("cell_definition.yaml"),
    QStringLiteral("scene_manifest.yaml"),
    QStringLiteral("environment_layout.yaml"),
    QStringLiteral("layout/workcell_studio_layout.yaml"),
    QStringLiteral("config/workcell_builder_task_intent.yaml"),
    QStringLiteral("config/task_recipe.yaml"),
    QStringLiteral("task/task_intent.yaml"),
    QStringLiteral("task/task_recipe.yaml")};
  for (const QString & relative : evidence) {
    const QFileInfo info(QDir(scene_dir).filePath(relative));
    if (info.exists() && info.lastModified().isValid() && (!latest.isValid() || info.lastModified() > latest))
      latest = info.lastModified();
  }
  return latest;
}

inline QString scene_content_fingerprint(const QString & workspace_root, const QString & scene_id)
{
  const QString scene_dir = scene_dir_for_id(workspace_root, scene_id);
  if (scene_dir.isEmpty()) return QStringLiteral("missing");
  const QStringList evidence = {
    QStringLiteral("environment.yaml"), QStringLiteral("cell_definition.yaml"),
    QStringLiteral("scene_manifest.yaml"), QStringLiteral("environment_layout.yaml"),
    QStringLiteral("layout/workcell_studio_layout.yaml"),
    QStringLiteral("config/workcell_builder_task_intent.yaml"),
    QStringLiteral("config/task_recipe.yaml"), QStringLiteral("task/task_intent.yaml"),
    QStringLiteral("task/task_recipe.yaml"),
    QStringLiteral("generated/scene_visual_mesh_index.json")};
  QCryptographicHash hash(QCryptographicHash::Sha256);
  for (const QString & relative : evidence) {
    QFile file(QDir(scene_dir).filePath(relative));
    hash.addData(relative.toUtf8());
    if (file.open(QIODevice::ReadOnly)) hash.addData(file.readAll());
  }
  return QString::fromLatin1(hash.result().toHex().left(16));
}

inline QString home_preview_cache_directory()
{
  QString root = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
  if (root.trimmed().isEmpty()) root = QDir(QDir::tempPath()).filePath(QStringLiteral("workcell_studio"));
  return QDir(root).filePath(QStringLiteral("home_previews"));
}

inline QString safe_home_preview_component(QString value)
{
  for (int i = 0; i < value.size(); ++i) {
    const QChar c = value.at(i);
    if (!c.isLetterOrNumber() && c != QLatin1Char('_') && c != QLatin1Char('-')) value[i] = QLatin1Char('_');
  }
  return value.isEmpty() ? QStringLiteral("workcell") : value;
}

inline bool completed_home_preview_contract(
  const QString & image_path, const QString & scene_id, const QString & fingerprint = QString())
{
  QFile contract_file(image_path + QStringLiteral(".json"));
  if (!QFileInfo(image_path).isFile() || !contract_file.open(QIODevice::ReadOnly)) return false;
  QJsonParseError error;
  const QJsonDocument document = QJsonDocument::fromJson(contract_file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) return false;
  const QJsonObject contract = document.object();
  return contract.value(QStringLiteral("scene_id")).toString() == scene_id &&
    (fingerprint.isEmpty() || contract.value(QStringLiteral("scene_fingerprint")).toString() == fingerprint) &&
    contract.value(QStringLiteral("lifecycle_state")).toString() == QStringLiteral("scene_ready") &&
    contract.value(QStringLiteral("terminal")).toBool() &&
    contract.value(QStringLiteral("rendered_physical_item_count")).toInt() > 0;
}

inline QString current_valid_home_preview_path(const QString & workspace_root, const QString & scene_id)
{
  const QString fingerprint = scene_content_fingerprint(workspace_root, scene_id);
  const QString path = QDir(home_preview_cache_directory()).filePath(QStringLiteral("%1-%2.png")
    .arg(safe_home_preview_component(scene_id), fingerprint));
  return completed_home_preview_contract(path, scene_id, fingerprint) ? path : QString();
}

inline QString relative_time(const QDateTime & timestamp)
{
  if (!timestamp.isValid()) return QStringLiteral("—");
  const qint64 seconds = timestamp.secsTo(QDateTime::currentDateTime());
  if (seconds < 0) return QStringLiteral("just now");
  if (seconds < 60) return QStringLiteral("just now");
  if (seconds < 3600) return QStringLiteral("%1 min ago").arg(qMax<qint64>(1, seconds / 60));
  if (seconds < 86400) {
    const qint64 hours = qMax<qint64>(1, seconds / 3600);
    return QStringLiteral("%1 %2 ago").arg(hours).arg(hours == 1 ? QStringLiteral("hour") : QStringLiteral("hours"));
  }
  const qint64 days = qMax<qint64>(1, seconds / 86400);
  return QStringLiteral("%1 %2 ago").arg(days).arg(days == 1 ? QStringLiteral("day") : QStringLiteral("days"));
}

inline QString scene_status_at(QTableWidget * table, int row)
{
  return table && row >= 0 && row < table->rowCount() && table->item(row, 1)
    ? clean_status(table->item(row, 1)->text()) : QStringLiteral("Unknown");
}

inline bool scene_is_pinned(QTableWidget * table, int row)
{
  if (!table || row < 0 || row >= table->rowCount() || !table->item(row, 0)) return false;
  return table->item(row, 0)->data(kPinnedRole).toBool();
}

inline void restore_pinned_state(QTableWidget * table)
{
  if (!table) return;
  QSettings settings;
  for (int row = 0; row < table->rowCount(); ++row) {
    QTableWidgetItem * item = table->item(row, 0);
    if (!item) continue;
    const QString id = scene_id_at(table, row);
    item->setData(kPinnedRole, settings.value(QStringLiteral("home/pinned/%1").arg(id), false).toBool());
  }
}

class TargetWorkcellDelegate final : public QStyledItemDelegate
{
public:
  TargetWorkcellDelegate(QString workspace_root, QObject * parent = nullptr)
  : QStyledItemDelegate(parent), workspace_root_(std::move(workspace_root)) {}

  void paint(QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index) const override
  {
    painter->save();
    QStyleOptionViewItem base(option);
    initStyleOption(&base, index);
    base.text.clear();
    const QWidget * widget = option.widget;
    QStyle * style = widget ? widget->style() : QApplication::style();
    style->drawControl(QStyle::CE_ItemViewItem, &base, painter, widget);

    const QRect r = option.rect;
    const bool selected = option.state & QStyle::State_Selected;
    const QColor primary = selected ? QColor(QStringLiteral("#123F74")) : QColor(QStringLiteral("#102B4E"));
    const QColor muted = selected ? QColor(QStringLiteral("#45698D")) : QColor(QStringLiteral("#70849A"));
    const QAbstractItemModel * model = index.model();
    const QModelIndex first = model->index(index.row(), 0);
    const QString raw_id = first.data(Qt::ToolTipRole).toString().trimmed().isEmpty()
      ? first.data(Qt::DisplayRole).toString() : first.data(Qt::ToolTipRole).toString();
    const QString display_name = first.data(kDisplayNameRole).toString().trimmed();

    if (index.column() == 0) {
      const QRect thumb(r.left() + 10, r.top() + 7, 70, r.height() - 14);
      painter->setPen(QPen(QColor(QStringLiteral("#DCE5EF")), 1));
      painter->setBrush(QColor(QStringLiteral("#F7F9FC")));
      painter->drawRoundedRect(thumb, 5, 5);
      const QString preview_path = current_valid_home_preview_path(workspace_root_, raw_id);
      QPixmap pixmap(preview_path);
      if (!pixmap.isNull()) {
        QPainterPath clip;
        clip.addRoundedRect(thumb.adjusted(1, 1, -1, -1), 4, 4);
        painter->setClipPath(clip);
        painter->drawPixmap(thumb.adjusted(2, 2, -2, -2), pixmap.scaled(
          thumb.size() - QSize(4, 4), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        painter->setClipping(false);
      } else {
        painter->setPen(QColor(QStringLiteral("#7B91A8")));
        painter->drawText(thumb, Qt::AlignCenter, QStringLiteral("◇"));
      }
      const QRect text_rect(thumb.right() + 12, r.top() + 9, r.right() - thumb.right() - 20, r.height() - 18);
      QFont title_font = option.font;
      title_font.setBold(true);
      painter->setFont(title_font);
      painter->setPen(primary);
      painter->drawText(text_rect.adjusted(0, 1, 0, -20), Qt::AlignLeft | Qt::AlignVCenter,
        display_name.isEmpty() || display_name == raw_id ? friendly_workcell_name(raw_id) : display_name);
      QFont id_font = option.font;
      id_font.setPointSizeF(qMax(8.0, option.font.pointSizeF() - 1.0));
      painter->setFont(id_font);
      painter->setPen(muted);
      painter->drawText(text_rect.adjusted(0, 23, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, raw_id);
    } else if (index.column() == 1) {
      const QString status = clean_status(index.data(Qt::DisplayRole).toString());
      QColor accent(QStringLiteral("#D97706"));
      QString icon = QStringLiteral("△");
      if (status == QStringLiteral("Ready")) {
        accent = QColor(QStringLiteral("#14804A")); icon = QStringLiteral("●");
      } else if (status == QStringLiteral("Blocked")) {
        accent = QColor(QStringLiteral("#C52B2B")); icon = QStringLiteral("×");
      }
      QFont strong = option.font; strong.setBold(true);
      painter->setFont(strong); painter->setPen(accent);
      painter->drawText(r.adjusted(10, 0, -5, 0), Qt::AlignLeft | Qt::AlignVCenter,
        icon + QStringLiteral("  ") + status);
    } else if (index.column() == 2) {
      painter->setPen(primary);
      painter->drawText(r.adjusted(10, 0, -5, 0), Qt::AlignLeft | Qt::AlignVCenter,
        clean_robot(index.data(Qt::DisplayRole).toString()));
    } else if (index.column() == 3) {
      painter->setPen(primary);
      painter->drawText(r.adjusted(10, 0, -5, 0), Qt::AlignLeft | Qt::AlignVCenter,
        clean_tool(index.data(Qt::DisplayRole).toString()));
    } else if (index.column() == 4) {
      painter->setPen(primary);
      painter->drawText(r.adjusted(10, 0, -5, 0), Qt::AlignLeft | Qt::AlignVCenter,
        relative_time(scene_last_updated(workspace_root_, raw_id)));
    } else if (index.column() == 5) {
      const bool pinned = first.data(kPinnedRole).toBool();
      QFont star = option.font; star.setPixelSize(18); star.setBold(true);
      painter->setFont(star);
      painter->setPen(pinned ? QColor(QStringLiteral("#123F7A")) : QColor(QStringLiteral("#9DB0C4")));
      painter->drawText(r, Qt::AlignCenter, pinned ? QStringLiteral("★") : QStringLiteral("☆"));
    }
    painter->restore();
  }

private:
  QString workspace_root_;
};

inline QLayout * find_layout_containing(QLayout * layout, QWidget * target)
{
  if (!layout || !target) return nullptr;
  for (int i = 0; i < layout->count(); ++i) {
    QLayoutItem * item = layout->itemAt(i);
    if (!item) continue;
    if (item->widget() == target) return layout;
    if (item->layout()) {
      if (QLayout * found = find_layout_containing(item->layout(), target)) return found;
    }
  }
  return nullptr;
}

inline QPushButton * sidebar_button(const QString & icon, const QString & text, QWidget * parent)
{
  auto * button = new QPushButton(icon + QStringLiteral("   ") + text, parent);
  button->setObjectName(QStringLiteral("studioTargetNavButton"));
  button->setProperty("active", false);
  button->setCursor(Qt::PointingHandCursor);
  button->setMinimumHeight(48);
  return button;
}

inline QAction * action_with_text(QObject * root, const QStringList & candidates)
{
  if (!root) return nullptr;
  for (QAction * action : root->findChildren<QAction *>()) {
    if (!action) continue;
    const QString text = action->text().remove('&').trimmed();
    for (const QString & candidate : candidates) {
      if (text.compare(candidate, Qt::CaseInsensitive) == 0 || text.contains(candidate, Qt::CaseInsensitive))
        return action;
    }
  }
  return nullptr;
}

inline void set_nav_active(const QList<QPushButton *> & buttons, int active_index)
{
  for (int i = 0; i < buttons.size(); ++i) {
    buttons[i]->setProperty("active", i == active_index);
    buttons[i]->style()->unpolish(buttons[i]);
    buttons[i]->style()->polish(buttons[i]);
  }
}

inline void build_application_shell(QMainWindow * window, QStackedWidget * pages)
{
  if (!window || !pages || window->findChild<QWidget *>(QStringLiteral("studioTargetShell"))) return;
  QWidget * central = window->centralWidget();
  if (!central || !central->layout()) return;
  QLayout * host = find_layout_containing(central->layout(), pages);
  auto * body = qobject_cast<QHBoxLayout *>(host);
  if (!body) return;

  auto * shell_marker = new QWidget(central);
  shell_marker->setObjectName(QStringLiteral("studioTargetShell"));
  shell_marker->hide();

  body->removeWidget(pages);
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(0);

  auto * sidebar = new QFrame(central);
  sidebar->setObjectName(QStringLiteral("studioTargetSidebar"));
  sidebar->setFixedWidth(194);
  auto * side = new QVBoxLayout(sidebar);
  side->setContentsMargins(14, 14, 14, 18);
  side->setSpacing(9);

  auto * brand = new QLabel(QStringLiteral("◈   WORKCELL\n      STUDIO"), sidebar);
  brand->setObjectName(QStringLiteral("studioTargetBrand"));
  brand->setMinimumHeight(62);
  side->addWidget(brand);

  auto * new_cell = new QPushButton(QStringLiteral("＋   New Cell"), sidebar);
  new_cell->setObjectName(QStringLiteral("studioTargetNewCell"));
  new_cell->setMinimumHeight(46);
  side->addWidget(new_cell);
  side->addSpacing(8);

  QPushButton * home = sidebar_button(QStringLiteral("⌂"), QStringLiteral("Home"), sidebar);
  QPushButton * product = sidebar_button(QStringLiteral("◇"), QStringLiteral("Product View"), sidebar);
  QPushButton * simulate = sidebar_button(QStringLiteral("▷"), QStringLiteral("Simulate"), sidebar);
  QPushButton * validation = sidebar_button(QStringLiteral("♢"), QStringLiteral("Validation"), sidebar);
  QPushButton * export_btn = sidebar_button(QStringLiteral("⇧"), QStringLiteral("Export"), sidebar);
  const QList<QPushButton *> nav = {home, product, simulate, validation, export_btn};
  for (QPushButton * button : nav) side->addWidget(button);
  side->addStretch(1);

  QPushButton * system = sidebar_button(QStringLiteral("⚙"), QStringLiteral("System"), sidebar);
  side->addWidget(system);
  side->addSpacing(12);
  auto * online = new QLabel(QStringLiteral("●  System online\n     ROS 2 %1")
    .arg(QString::fromLocal8Bit(qgetenv("ROS_DISTRO")).trimmed().isEmpty() ? QStringLiteral("Humble") :
      QString::fromLocal8Bit(qgetenv("ROS_DISTRO")).trimmed()), sidebar);
  online->setObjectName(QStringLiteral("studioTargetOnline"));
  side->addWidget(online);
  side->addSpacing(10);
  auto * version = new QLabel(QStringLiteral("WS   v0.1.0"), sidebar);
  version->setObjectName(QStringLiteral("studioTargetVersion"));
  side->addWidget(version);

  auto * right = new QWidget(central);
  right->setObjectName(QStringLiteral("studioTargetRightShell"));
  auto * right_layout = new QVBoxLayout(right);
  right_layout->setContentsMargins(0, 0, 0, 0);
  right_layout->setSpacing(0);

  auto * topbar = new QFrame(right);
  topbar->setObjectName(QStringLiteral("studioTargetTopbar"));
  topbar->setFixedHeight(60);
  auto * top = new QHBoxLayout(topbar);
  top->setContentsMargins(20, 10, 18, 10);
  top->addStretch(1);
  auto * fake = new QLabel(QStringLiteral("●  Simulation Mode   FAKE HARDWARE"), topbar);
  fake->setObjectName(QStringLiteral("studioTargetFakePill"));
  top->addWidget(fake);
  auto * locked = new QLabel(QStringLiteral("▣  Real hardware   LOCKED"), topbar);
  locked->setObjectName(QStringLiteral("studioTargetLockedPill"));
  top->addWidget(locked);
  auto * help = new QToolButton(topbar); help->setText(QStringLiteral("?")); help->setObjectName(QStringLiteral("studioTargetCircleButton"));
  auto * settings = new QToolButton(topbar); settings->setText(QStringLiteral("⚙")); settings->setObjectName(QStringLiteral("studioTargetCircleButton"));
  auto * avatar = new QLabel(QStringLiteral("WS"), topbar); avatar->setObjectName(QStringLiteral("studioTargetAvatar")); avatar->setAlignment(Qt::AlignCenter); avatar->setFixedSize(38, 38);
  top->addWidget(help); top->addWidget(settings); top->addWidget(avatar);

  pages->setParent(right);
  right_layout->addWidget(topbar);
  right_layout->addWidget(pages, 1);
  body->addWidget(sidebar);
  body->addWidget(right, 1);

  QObject::connect(home, &QPushButton::clicked, pages, [pages]() { pages->setCurrentIndex(0); });
  QObject::connect(product, &QPushButton::clicked, pages, [pages]() { pages->setCurrentIndex(1); });
  QObject::connect(simulate, &QPushButton::clicked, pages, [pages]() { pages->setCurrentIndex(4); });
  QObject::connect(validation, &QPushButton::clicked, pages, [pages]() { pages->setCurrentIndex(6); });
  QObject::connect(export_btn, &QPushButton::clicked, pages, [pages]() { pages->setCurrentIndex(7); });
  QObject::connect(system, &QPushButton::clicked, pages, [pages]() { pages->setCurrentIndex(5); });
  if (QAction * action = action_with_text(window, {QStringLiteral("New Cell")}))
    QObject::connect(new_cell, &QPushButton::clicked, action, &QAction::trigger);

  const auto update_nav = [nav](int index) {
    int active = -1;
    if (index == 0) active = 0;
    else if (index == 1) active = 1;
    else if (index == 4) active = 2;
    else if (index == 6) active = 3;
    else if (index == 7) active = 4;
    set_nav_active(nav, active);
  };
  QObject::connect(pages, &QStackedWidget::currentChanged, pages, update_nav);
  update_nav(pages->currentIndex());

  if (QFrame * log = window->findChild<QFrame *>(QStringLiteral("sceneBuilderLogDrawer"))) log->hide();
  if (window->statusBar()) window->statusBar()->hide();

  central->setStyleSheet(central->styleSheet() + QStringLiteral(
    "QFrame#studioTargetSidebar{background:#0B2341;border:0;border-right:1px solid #163B63;}"
    "QLabel#studioTargetBrand{color:#FFFFFF;font-size:17px;font-weight:900;letter-spacing:.5px;}"
    "QPushButton#studioTargetNewCell{background:#0758D8;color:#FFFFFF;border:1px solid #146AE8;border-radius:6px;font-weight:800;text-align:left;padding-left:14px;}"
    "QPushButton#studioTargetNewCell:hover{background:#0865EE;}"
    "QPushButton#studioTargetNavButton{background:transparent;color:#E2ECF7;border:0;border-radius:6px;text-align:left;padding-left:13px;font-size:12px;font-weight:650;}"
    "QPushButton#studioTargetNavButton:hover{background:#11365D;}"
    "QPushButton#studioTargetNavButton[active=true]{background:#12457A;color:#FFFFFF;}"
    "QLabel#studioTargetOnline{color:#AFC3D8;font-size:11px;line-height:1.5;}"
    "QLabel#studioTargetVersion{color:#7992AA;font-size:10px;}"
    "QWidget#studioTargetRightShell{background:#F5F8FC;}"
    "QFrame#studioTargetTopbar{background:#FFFFFF;border:0;border-bottom:1px solid #DDE6EF;}"
    "QLabel#studioTargetFakePill{background:#F3FBF6;color:#14794A;border:1px solid #D6E9DF;border-radius:17px;padding:7px 15px;font-size:10px;font-weight:750;}"
    "QLabel#studioTargetLockedPill{background:#F8FAFD;color:#183C67;border:1px solid #DCE5EF;border-radius:17px;padding:7px 15px;font-size:10px;font-weight:750;}"
    "QToolButton#studioTargetCircleButton{background:#FFFFFF;color:#173B68;border:1px solid #DCE5EF;border-radius:17px;min-width:34px;max-width:34px;min-height:34px;max-height:34px;padding:0;font-weight:800;}"
    "QLabel#studioTargetAvatar{background:#0C2749;color:#FFFFFF;border-radius:19px;font-size:11px;font-weight:800;}"));
}

inline QFrame * metric_card(const QString & icon, const QString & label, int count, const QString & hint,
  const QString & kind, QWidget * parent)
{
  auto * card = new QFrame(parent);
  card->setObjectName(QStringLiteral("studioTargetMetricCard"));
  card->setProperty("kind", kind);
  auto * row = new QHBoxLayout(card);
  row->setContentsMargins(10, 7, 12, 7);
  row->setSpacing(8);
  auto * icon_label = new QLabel(icon, card);
  icon_label->setObjectName(QStringLiteral("studioTargetMetricIcon"));
  icon_label->setProperty("kind", kind);
  icon_label->setAlignment(Qt::AlignCenter);
  icon_label->setFixedSize(36, 36);
  row->addWidget(icon_label);
  auto * copy = new QVBoxLayout();
  copy->setSpacing(0);
  auto * title = new QLabel(label, card); title->setObjectName(QStringLiteral("studioTargetMetricLabel"));
  auto * value = new QLabel(QString::number(count), card); value->setObjectName(QStringLiteral("studioTargetMetricValue"));
  value->setProperty("kind", kind);
  auto * sub = new QLabel(hint, card); sub->setObjectName(QStringLiteral("studioTargetMetricHint"));
  copy->addWidget(title); copy->addWidget(value); copy->addWidget(sub);
  row->addLayout(copy, 1);
  return card;
}

inline void apply_composed_filters(QTableWidget * table, QLineEdit * search, QComboBox * status,
  QComboBox * robot, QComboBox * tool, QPushButton * pinned, QLabel * footer)
{
  if (!table) return;
  const QString query = search ? search->text().trimmed().toLower() : QString();
  const QString status_filter = status ? status->currentText() : QStringLiteral("Status: All");
  const QString robot_filter = robot ? robot->currentText() : QStringLiteral("Robot: All");
  const QString tool_filter = tool ? tool->currentText() : QStringLiteral("Tool: All");
  const bool pinned_only = pinned && pinned->isChecked();
  int visible = 0;
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString robot_value = table->item(row, 2) ? clean_robot(table->item(row, 2)->text()) : QStringLiteral("—");
    const QString tool_value = table->item(row, 3) ? clean_tool(table->item(row, 3)->text()) : QStringLiteral("—");
    const QString scene_id = scene_id_at(table, row);
    const QString display_name = table->item(row, 0) ? table->item(row, 0)->data(kDisplayNameRole).toString() : QString();
    const QString searchable = QStringLiteral("%1 %2 %3 %4")
      .arg(display_name, scene_id, robot_value, tool_value).toLower();
    const QString row_status = scene_status_at(table, row);
    const bool search_matches = query.isEmpty() || searchable.contains(query);
    const bool status_matches = !status || status->currentIndex() == 0 ||
      (status_filter.contains(QStringLiteral("Ready"), Qt::CaseInsensitive) && row_status == QStringLiteral("Ready")) ||
      (status_filter.contains(QStringLiteral("Warning"), Qt::CaseInsensitive) && row_status == QStringLiteral("Needs Attention")) ||
      (status_filter.contains(QStringLiteral("Attention"), Qt::CaseInsensitive) && row_status == QStringLiteral("Needs Attention")) ||
      (status_filter.contains(QStringLiteral("Blocked"), Qt::CaseInsensitive) && row_status == QStringLiteral("Blocked"));
    const bool robot_matches = !robot || robot->currentIndex() == 0 || robot_filter == robot_value;
    const bool tool_matches = !tool || tool->currentIndex() == 0 || tool_filter == tool_value;
    const bool pinned_matches = !pinned_only || scene_is_pinned(table, row);
    const bool show = search_matches && status_matches && robot_matches && tool_matches && pinned_matches;
    table->setRowHidden(row, !show);
    if (show) ++visible;
  }
  const int selected_row = table->currentRow();
  if (selected_row >= 0 && table->isRowHidden(selected_row)) {
    table->clearSelection();
    table->setCurrentCell(-1, -1);
  }
  if (footer) {
    const int total = table->property("studioTargetTotalWorkcells").toInt();
    footer->setText(QStringLiteral("Showing %1 of %2 workcells").arg(visible).arg(total > 0 ? total : table->rowCount()));
  }
}

inline void refresh_filter_options(QTableWidget * table, QComboBox * robot, QComboBox * tool)
{
  if (!table) return;
  const QString current_robot = robot ? robot->currentText() : QString();
  const QString current_tool = tool ? tool->currentText() : QString();
  QSet<QString> robots, tools;
  for (int row = 0; row < table->rowCount(); ++row) {
    if (table->item(row, 2)) robots.insert(clean_robot(table->item(row, 2)->text()));
    if (table->item(row, 3)) tools.insert(clean_tool(table->item(row, 3)->text()));
  }
  const auto repopulate = [](QComboBox * combo, const QString & all, const QSet<QString> & values, const QString & current) {
    if (!combo) return;
    const QSignalBlocker blocker(combo);
    combo->clear(); combo->addItem(all);
    QStringList sorted = values.values(); sorted.removeAll(QStringLiteral("—"));
    sorted.sort(Qt::CaseInsensitive);
    combo->addItems(sorted);
    const int index = combo->findText(current);
    combo->setCurrentIndex(index >= 0 ? index : 0);
  };
  repopulate(robot, QStringLiteral("Robot: All"), robots, current_robot);
  repopulate(tool, QStringLiteral("Tool: All"), tools, current_tool);
}

inline void refresh_metrics(QMainWindow * window, QTableWidget * table)
{
  if (!window || !table) return;
  int ready = 0, attention = 0, blocked = 0;
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString status = scene_status_at(table, row);
    if (status == QStringLiteral("Ready")) ++ready;
    else if (status == QStringLiteral("Needs Attention")) ++attention;
    else ++blocked;
  }
  for (QLabel * value : window->findChildren<QLabel *>(QStringLiteral("studioTargetMetricValue"))) {
    const QString kind = value->property("kind").toString();
    value->setText(QString::number(kind == QStringLiteral("total") ? table->rowCount() :
      kind == QStringLiteral("ready") ? ready : kind == QStringLiteral("attention") ? attention : blocked));
  }
  table->setProperty("studioTargetTotalWorkcells", table->rowCount());
}

inline void sort_target_rows(QTableWidget * table, QComboBox * sort)
{
  if (!table || !sort || table->rowCount() < 2) return;
  const QString selected_id = scene_id_at(table, table->currentRow());
  std::vector<int> order(static_cast<std::size_t>(table->rowCount()));
  for (int row = 0; row < table->rowCount(); ++row) {
    order[static_cast<std::size_t>(row)] = row;
    if (table->item(row, 0)) table->item(row, 0)->setData(
      kModifiedRole, scene_last_updated(table->property("studioTargetWorkspaceRoot").toString(), scene_id_at(table, row)));
  }
  const int mode = sort->currentIndex();
  const auto less = [table, mode](int a, int b) {
    const QString aid = scene_id_at(table, a), bid = scene_id_at(table, b);
    const QString aname = table->item(a, 0)->data(kDisplayNameRole).toString().isEmpty() ? aid : table->item(a, 0)->data(kDisplayNameRole).toString();
    const QString bname = table->item(b, 0)->data(kDisplayNameRole).toString().isEmpty() ? bid : table->item(b, 0)->data(kDisplayNameRole).toString();
    if (mode == 0) {
      const QDateTime ad = table->item(a, 0)->data(kModifiedRole).toDateTime();
      const QDateTime bd = table->item(b, 0)->data(kModifiedRole).toDateTime();
      if (ad != bd) return ad > bd;
    } else if (mode == 1) {
      const int name_cmp = QString::compare(aname, bname, Qt::CaseInsensitive);
      if (name_cmp != 0) return name_cmp < 0;
    } else if (mode == 2) {
      const auto rank = [table](int row) {
        const QString value = scene_status_at(table, row);
        return value == QStringLiteral("Ready") ? 0 : value == QStringLiteral("Needs Attention") ? 1 : 2;
      };
      if (rank(a) != rank(b)) return rank(a) < rank(b);
    } else if (mode == 3 && scene_is_pinned(table, a) != scene_is_pinned(table, b)) {
      return scene_is_pinned(table, a);
    }
    return QString::compare(aid, bid, Qt::CaseInsensitive) < 0;
  };
  std::stable_sort(order.begin(), order.end(), less);
  const QSignalBlocker blocker(table);
  std::vector<std::array<QTableWidgetItem *, 6>> rows(order.size());
  for (int old_row = 0; old_row < table->rowCount(); ++old_row)
    for (int column = 0; column < 6; ++column)
      rows[static_cast<std::size_t>(old_row)][static_cast<std::size_t>(column)] = table->takeItem(old_row, column);
  for (int new_row = 0; new_row < table->rowCount(); ++new_row) {
    const int old_row = order[static_cast<std::size_t>(new_row)];
    for (int column = 0; column < 6; ++column)
      table->setItem(new_row, column, rows[static_cast<std::size_t>(old_row)][static_cast<std::size_t>(column)]);
    if (scene_id_at(table, new_row) == selected_id) table->setCurrentCell(new_row, 0);
  }
}

inline void refresh_target_details(QMainWindow * window, QTableWidget * table, const QString & workspace_root)
{
  if (!window || !table) return;
  const int row = table->currentRow();
  const bool selected = row >= 0 && row < table->rowCount() && table->item(row, 0);
  const QString scene_id = selected ? scene_id_at(table, row) : QString();
  const QString status = selected ? scene_status_at(table, row) : QStringLiteral("No selection");
  const QString robot = selected && table->item(row, 2) ? clean_robot(table->item(row, 2)->text()) : QStringLiteral("—");
  const QString tool = selected && table->item(row, 3) ? clean_tool(table->item(row, 3)->text()) : QStringLiteral("—");
  const QString task = selected && table->item(row, 4) ? clean_task(table->item(row, 4)->text()) : QStringLiteral("—");
  const QString launch = selected && table->item(row, 5) ? clean_launch(table->item(row, 5)->text()) : QStringLiteral("—");
  const QDateTime updated = selected ? scene_last_updated(workspace_root, scene_id) : QDateTime();

  const QString display_name = selected ? table->item(row, 0)->data(kDisplayNameRole).toString().trimmed() : QString();
  set_label(window, QStringLiteral("studioTargetSelectedTitle"), selected ?
    (display_name.isEmpty() || display_name == scene_id ? friendly_workcell_name(scene_id) : display_name) :
    QStringLiteral("Select a workcell"));
  set_label(window, QStringLiteral("studioTargetSelectedId"), scene_id);
  set_label(window, QStringLiteral("studioTargetStatus"), status);
  set_label(window, QStringLiteral("studioTargetMetaRobot"), robot);
  set_label(window, QStringLiteral("studioTargetMetaTool"), tool);
  set_label(window, QStringLiteral("studioTargetMetaTask"), task);
  set_label(window, QStringLiteral("studioTargetMetaLaunch"), launch);
  set_label(window, QStringLiteral("studioTargetMetaUpdated"), updated.isValid()
    ? relative_time(updated) + QStringLiteral("\n") + updated.toString(QStringLiteral("yyyy-MM-dd HH:mm")) : QStringLiteral("—"));

  if (auto * status_label = window->findChild<QLabel *>(QStringLiteral("studioTargetStatus"))) {
    const QString kind = status == QStringLiteral("Ready") ? QStringLiteral("ready") :
      status == QStringLiteral("Needs Attention") ? QStringLiteral("attention") :
      status == QStringLiteral("Blocked") ? QStringLiteral("blocked") : QStringLiteral("neutral");
    status_label->setProperty("kind", kind);
    status_label->style()->unpolish(status_label); status_label->style()->polish(status_label);
  }

  if (auto * preview = window->findChild<QLabel *>(QStringLiteral("studioTargetPreview"))) {
    preview->setPixmap(QPixmap());
    const QString path = selected ? current_valid_home_preview_path(workspace_root, scene_id) : QString();
    const QPixmap pixmap(path);
    if (!pixmap.isNull()) {
      preview->setText(QString());
      preview->setPixmap(pixmap.scaled(QSize(qMax(320, preview->width() - 8), 250), Qt::KeepAspectRatio, Qt::SmoothTransformation));
      preview->setToolTip(path);
    } else {
      preview->setText(selected ? QStringLiteral("Preview unavailable") :
        QStringLiteral("Select a workcell\nto preview its scene"));
      preview->setToolTip(QString());
    }
  }

  if (auto * readiness = window->findChild<QLabel *>(QStringLiteral("studioTargetReadiness"))) {
    if (!selected) readiness->setText(QStringLiteral("Select a workcell to review readiness."));
    else if (status == QStringLiteral("Ready")) readiness->setText(QStringLiteral("Ready for fake-hardware simulation"));
    else {
      QStringList reasons = table->item(row, 0)->data(kReadinessReasonsRole).toStringList();
      if (reasons.isEmpty()) reasons << (status == QStringLiteral("Blocked") ?
        QStringLiteral("Required scene evidence is incomplete") :
        QStringLiteral("Review validation before simulation"));
      const QString primary = reasons.isEmpty() ?
        (status == QStringLiteral("Blocked") ? QStringLiteral("Required scene evidence is incomplete") :
          QStringLiteral("Review validation before simulation")) : reasons.first();
      QString text = status == QStringLiteral("Blocked") ? QStringLiteral("Blocked · %1").arg(primary) :
        QStringLiteral("%1 warning%2").arg(qMax(1, reasons.size())).arg(reasons.size() == 1 ? QString() : QStringLiteral("s"));
      const int detail_count = qMin(3, reasons.size());
      for (int i = 0; i < detail_count; ++i) text += QStringLiteral("\n• %1").arg(reasons.at(i));
      readiness->setText(text);
    }
  }

  const bool fake_ready = selected && table->item(row, 0)->data(kFakeHardwareReadyRole).toBool();
  Q_UNUSED(fake_ready);
  if (auto * validation = window->findChild<QPushButton *>(QStringLiteral("studioTargetViewValidation")))
    validation->setVisible(selected && status != QStringLiteral("Ready"));

  if (auto * pin = window->findChild<QToolButton *>(QStringLiteral("studioTargetInspectorPin")))
    pin->setText(selected && scene_is_pinned(table, row) ? QStringLiteral("★") : QStringLiteral("☆"));
}

inline QLabel * target_value(QWidget * parent, const QString & object_name)
{
  auto * label = new QLabel(QStringLiteral("—"), parent);
  label->setObjectName(object_name);
  label->setWordWrap(true);
  return label;
}

inline QFrame * build_inspector(QMainWindow * window, QTableWidget * table, const QString & workspace_root, QWidget * parent)
{
  auto * inspector = new QFrame(parent);
  inspector->setObjectName(QStringLiteral("studioTargetInspector"));
  inspector->setMinimumWidth(380);
  inspector->setMaximumWidth(440);
  auto * layout = new QVBoxLayout(inspector);
  layout->setContentsMargins(16, 14, 16, 16);
  layout->setSpacing(8);

  auto * header = new QHBoxLayout();
  auto * title = new QLabel(QStringLiteral("Selected Workcell"), inspector);
  title->setObjectName(QStringLiteral("studioTargetInspectorEyebrow"));
  header->addWidget(title); header->addStretch(1);
  auto * pin = new QToolButton(inspector); pin->setObjectName(QStringLiteral("studioTargetInspectorPin")); pin->setText(QStringLiteral("☆"));
  auto * more = new QToolButton(inspector); more->setObjectName(QStringLiteral("studioTargetInspectorMore")); more->setText(QStringLiteral("More ▾"));
  QMenu * source_menu = nullptr;
  if (auto * old_actions = window->findChild<QToolButton *>(QStringLiteral("studioHomeSecondaryButton"))) {
    source_menu = old_actions->menu();
    old_actions->hide();
  }
  auto * more_menu = new QMenu(more);
  more->setMenu(more_menu);
  more->setPopupMode(QToolButton::InstantPopup);
  header->addWidget(pin);
  layout->addLayout(header);

  auto * selected_title = new QLabel(QStringLiteral("Select a workcell"), inspector);
  selected_title->setObjectName(QStringLiteral("studioTargetSelectedTitle"));
  layout->addWidget(selected_title);
  auto * id = new QLabel(inspector); id->setObjectName(QStringLiteral("studioTargetSelectedId")); layout->addWidget(id);
  auto * status = new QLabel(QStringLiteral("No selection"), inspector); status->setObjectName(QStringLiteral("studioTargetStatus")); status->setProperty("kind", QStringLiteral("neutral")); status->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed); layout->addWidget(status, 0, Qt::AlignLeft);

  auto * preview = new QLabel(QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here"), inspector);
  preview->setObjectName(QStringLiteral("studioTargetPreview"));
  preview->setAlignment(Qt::AlignCenter);
  preview->setMinimumHeight(220);
  preview->setMaximumHeight(250);
  preview->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  layout->addWidget(preview);

  auto * meta = new QWidget(inspector); meta->setObjectName(QStringLiteral("studioTargetMetadata"));
  auto * grid = new QGridLayout(meta); grid->setContentsMargins(0, 4, 0, 4); grid->setHorizontalSpacing(16); grid->setVerticalSpacing(7);
  const QStringList keys = {QStringLiteral("♙   Robot"), QStringLiteral("⌘   Tool / Gripper"), QStringLiteral("▣   Task"), QStringLiteral("◉   Fake Hardware"), QStringLiteral("◷   Modified")};
  const QStringList names = {QStringLiteral("studioTargetMetaRobot"), QStringLiteral("studioTargetMetaTool"), QStringLiteral("studioTargetMetaTask"), QStringLiteral("studioTargetMetaLaunch"), QStringLiteral("studioTargetMetaUpdated")};
  for (int i = 0; i < keys.size(); ++i) {
    auto * key = new QLabel(keys[i], meta); key->setObjectName(QStringLiteral("studioTargetMetaKey"));
    grid->addWidget(key, i, 0); grid->addWidget(target_value(meta, names[i]), i, 1);
  }
  grid->setColumnStretch(1, 1);
  layout->addWidget(meta);

  auto * readiness_card = new QFrame(inspector); readiness_card->setObjectName(QStringLiteral("studioTargetReadinessCard"));
  auto * readiness_layout = new QVBoxLayout(readiness_card); readiness_layout->setContentsMargins(12, 10, 12, 10);
  auto * readiness = new QLabel(QStringLiteral("Select a workcell to review readiness."), readiness_card); readiness->setObjectName(QStringLiteral("studioTargetReadiness")); readiness->setWordWrap(true);
  readiness_layout->addWidget(readiness);
  auto * view_validation = new QPushButton(QStringLiteral("View validation"), readiness_card);
  view_validation->setObjectName(QStringLiteral("studioTargetViewValidation"));
  readiness_layout->addWidget(view_validation, 0, Qt::AlignLeft);
  layout->addWidget(readiness_card);

  auto * action_row = new QHBoxLayout();
  action_row->addStretch(1); action_row->addWidget(more); layout->addLayout(action_row);

  const auto source_action = [source_menu](const QString & text) -> QAction * {
    if (!source_menu) return nullptr;
    for (QAction * action : source_menu->actions())
      if (action && action->text().contains(text, Qt::CaseInsensitive)) return action;
    return nullptr;
  };
  const auto add_backed_more_action = [more, more_menu](QAction * backing, const QString & label) {
    if (!backing) return;
    QAction * proxy = more_menu->addAction(label);
    QObject::connect(proxy, &QAction::triggered, backing, &QAction::trigger);
    const auto sync = [more, more_menu, proxy, backing]() {
      proxy->setVisible(backing->isEnabled());
      bool any_visible = false;
      for (QAction * action : more_menu->actions()) any_visible = any_visible || action->isVisible();
      more->setVisible(any_visible);
    };
    QObject::connect(backing, &QAction::changed, more, sync);
    sync();
  };
  if (QAction * generate = action_with_text(window,
      {QStringLiteral("Generate Scene Package"), QStringLiteral("Generate Workcell Package")})) {
    add_backed_more_action(generate, QStringLiteral("Generate Package"));
  }
  if (QAction * remove = source_action(QStringLiteral("Delete Scene"))) {
    add_backed_more_action(remove, remove->text());
  }
  if (more_menu->actions().isEmpty()) more->hide();
  QObject::connect(view_validation, &QPushButton::clicked, window, [window]() {
    QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
    if (auto * pages = dashboard ? qobject_cast<QStackedWidget *>(dashboard->parentWidget()) : nullptr)
      pages->setCurrentIndex(6);
  });
  QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
  view_validation->setVisible(dashboard && qobject_cast<QStackedWidget *>(dashboard->parentWidget()));

  QObject::connect(pin, &QToolButton::clicked, window, [window, table, pin]() {
    const int row = table ? table->currentRow() : -1;
    if (!table || row < 0 || row >= table->rowCount() || !table->item(row, 0)) return;
    const bool next = !scene_is_pinned(table, row);
    table->item(row, 0)->setData(kPinnedRole, next);
    QSettings().setValue(QStringLiteral("home/pinned/%1").arg(scene_id_at(table, row)), next);
    pin->setText(next ? QStringLiteral("★") : QStringLiteral("☆"));
    table->viewport()->update();
    refresh_target_details(window, table, table->property("studioTargetWorkspaceRoot").toString());
  });

  refresh_target_details(window, table, workspace_root);
  return inspector;
}

inline void build_home_page(QMainWindow * window, QWidget * dashboard, QTableWidget * table, const QString & workspace_root)
{
  if (!window || !dashboard || !table || dashboard->findChild<QWidget *>(QStringLiteral("studioTargetHomeContent"))) return;
  auto * root = qobject_cast<QVBoxLayout *>(dashboard->layout());
  if (!root) return;

  for (QWidget * child : dashboard->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) child->hide();
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  const int total = table->rowCount();
  int ready_count = 0, warning_count = 0, blocked_count = 0;
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString status = scene_status_at(table, row);
    if (status == QStringLiteral("Ready")) ++ready_count;
    else if (status == QStringLiteral("Needs Attention")) ++warning_count;
    else ++blocked_count;
  }
  table->setProperty("studioTargetTotalWorkcells", total);
  restore_pinned_state(table);

  auto * home = new QWidget(dashboard); home->setObjectName(QStringLiteral("studioTargetHomeContent"));
  auto * page = new QHBoxLayout(home); page->setContentsMargins(28, 20, 18, 20); page->setSpacing(18);

  auto * left = new QWidget(home); left->setObjectName(QStringLiteral("studioTargetHomeMain"));
  auto * main = new QVBoxLayout(left); main->setContentsMargins(0, 0, 0, 0); main->setSpacing(14);
  auto * heading = new QLabel(QStringLiteral("Your workcells"), left); heading->setObjectName(QStringLiteral("studioTargetHeading")); main->addWidget(heading);
  auto * subheading = new QLabel(QStringLiteral("Select a workcell or start a new robotic cell."), left); subheading->setObjectName(QStringLiteral("studioTargetSubheading")); main->addWidget(subheading);

  auto * metrics = new QHBoxLayout(); metrics->setSpacing(12);
  metrics->addWidget(metric_card(QStringLiteral("◇"), QStringLiteral("Total Workcells"), total, QStringLiteral("Across all folders"), QStringLiteral("total"), left));
  metrics->addWidget(metric_card(QStringLiteral("✓"), QStringLiteral("Ready"), ready_count, QStringLiteral("Ready to simulate"), QStringLiteral("ready"), left));
  metrics->addWidget(metric_card(QStringLiteral("△"), QStringLiteral("Needs Attention"), warning_count, QStringLiteral("Warnings to review"), QStringLiteral("attention"), left));
  metrics->addWidget(metric_card(QStringLiteral("×"), QStringLiteral("Blocked"), blocked_count, QStringLiteral("Cannot run"), QStringLiteral("blocked"), left));
  main->addLayout(metrics);

  auto * tools = new QGridLayout(); tools->setHorizontalSpacing(9); tools->setVerticalSpacing(7);
  auto * search = window->findChild<QLineEdit *>(QStringLiteral("studioHomeSearchBox"));
  auto * status = window->findChild<QComboBox *>(QStringLiteral("studioHomeStatusFilter"));
  if (search) {
    if (search->parentWidget() && search->parentWidget()->layout()) search->parentWidget()->layout()->removeWidget(search);
    search->setParent(left); search->setPlaceholderText(QStringLiteral("⌕  Search workcells...")); search->setClearButtonEnabled(true); search->setMinimumWidth(230); search->setObjectName(QStringLiteral("studioTargetSearch"));
    tools->addWidget(search, 0, 0, 1, 3);
  }
  if (status) {
    if (status->parentWidget() && status->parentWidget()->layout()) status->parentWidget()->layout()->removeWidget(status);
    status->setParent(left); if (status->count() > 0) status->setItemText(0, QStringLiteral("Status: All")); status->setObjectName(QStringLiteral("studioTargetFilter")); tools->addWidget(status, 0, 3);
  }
  auto * robot = new QComboBox(left); robot->setObjectName(QStringLiteral("studioTargetFilter")); robot->addItem(QStringLiteral("Robot: All"));
  auto * tool = new QComboBox(left); tool->setObjectName(QStringLiteral("studioTargetFilter")); tool->addItem(QStringLiteral("Tool: All"));
  refresh_filter_options(table, robot, tool);
  tools->addWidget(robot, 1, 0); tools->addWidget(tool, 1, 1);
  auto * pinned = new QPushButton(QStringLiteral("☆  Pinned"), left); pinned->setObjectName(QStringLiteral("studioTargetPinnedFilter")); pinned->setCheckable(true); tools->addWidget(pinned, 1, 2);
  auto * sort = new QComboBox(left); sort->setObjectName(QStringLiteral("studioTargetSort"));
  sort->addItems({QStringLiteral("Recently modified"), QStringLiteral("Name A–Z"),
    QStringLiteral("Status"), QStringLiteral("Pinned first")});
  tools->addWidget(sort, 1, 3);
  tools->setColumnStretch(0, 2); tools->setColumnStretch(1, 1);
  main->addLayout(tools);

  auto * table_card = new QFrame(left); table_card->setObjectName(QStringLiteral("studioTargetTableCard"));
  auto * table_layout = new QVBoxLayout(table_card); table_layout->setContentsMargins(0, 0, 0, 0); table_layout->setSpacing(0);
  if (table->parentWidget() && table->parentWidget()->layout()) table->parentWidget()->layout()->removeWidget(table);
  table->setParent(table_card);
  table->setProperty("studioTargetWorkspaceRoot", workspace_root);
  table->setItemDelegate(new TargetWorkcellDelegate(workspace_root, table));
  table->setHorizontalHeaderLabels({QStringLiteral("Workcell"), QStringLiteral("Status"), QStringLiteral("Robot"), QStringLiteral("Tool / Gripper"), QStringLiteral("Modified"), QStringLiteral("★")});
  for (int row = 0; row < table->rowCount(); ++row) {
    for (int column = 0; column < table->columnCount(); ++column) {
      if (QTableWidgetItem * item = table->item(row, column)) item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    }
  }
  table->setShowGrid(false); table->setAlternatingRowColors(false); table->setWordWrap(false); table->setSelectionBehavior(QAbstractItemView::SelectRows); table->setSelectionMode(QAbstractItemView::SingleSelection); table->setEditTriggers(QAbstractItemView::NoEditTriggers); table->verticalHeader()->hide(); table->verticalHeader()->setDefaultSectionSize(62); table->horizontalHeader()->setMinimumHeight(38); table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch); table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Fixed); table->setColumnWidth(1, 145); table->setColumnWidth(2, 82); table->setColumnWidth(3, 135); table->setColumnWidth(4, 105); table->setColumnWidth(5, 46);
  table_layout->addWidget(table, 1);
  auto * footer = new QLabel(QStringLiteral("Showing %1 of %1 workcells").arg(total), table_card); footer->setObjectName(QStringLiteral("studioTargetTableFooter")); footer->setContentsMargins(12, 6, 12, 6); table_layout->addWidget(footer);
  main->addWidget(table_card, 1);

  auto * inspector = build_inspector(window, table, workspace_root, home);
  page->addWidget(left, 1); page->addWidget(inspector);
  root->addWidget(home, 1);

  const auto refresh_home = [window, table, search, status, robot, tool, pinned, sort, footer, workspace_root]() {
    restore_pinned_state(table);
    refresh_filter_options(table, robot, tool);
    sort_target_rows(table, sort);
    apply_composed_filters(table, search, status, robot, tool, pinned, footer);
    refresh_metrics(window, table);
    refresh_target_details(window, table, workspace_root);
    table->viewport()->update();
  };
  const auto apply_filters = [table, search, status, robot, tool, pinned, footer]() {
    apply_composed_filters(table, search, status, robot, tool, pinned, footer);
  };
  QObject::connect(robot, qOverload<int>(&QComboBox::currentIndexChanged), table, [apply_filters](int) { apply_filters(); });
  QObject::connect(tool, qOverload<int>(&QComboBox::currentIndexChanged), table, [apply_filters](int) { apply_filters(); });
  QObject::connect(pinned, &QPushButton::toggled, table, [apply_filters, pinned](bool on) { pinned->setText(on ? QStringLiteral("★  Pinned") : QStringLiteral("☆  Pinned")); apply_filters(); });
  if (search) QObject::connect(search, &QLineEdit::textChanged, table, [apply_filters](const QString &) { apply_filters(); });
  if (status) QObject::connect(status, qOverload<int>(&QComboBox::currentIndexChanged), table, [apply_filters](int) { apply_filters(); });
  QObject::connect(sort, qOverload<int>(&QComboBox::currentIndexChanged), table, [refresh_home](int) { refresh_home(); });
  if (auto * pages = qobject_cast<QStackedWidget *>(dashboard->parentWidget())) {
    QObject::connect(pages, &QStackedWidget::currentChanged, home,
      [refresh_home](int index) { if (index == 0) refresh_home(); });
  }
  QObject::connect(table, &QTableWidget::cellClicked, window, [window, table, workspace_root, refresh_home](int row, int column) {
    if (column == 5 && table->item(row, 0)) {
      const bool next = !scene_is_pinned(table, row);
      table->item(row, 0)->setData(kPinnedRole, next);
      QSettings().setValue(QStringLiteral("home/pinned/%1").arg(scene_id_at(table, row)), next);
      table->viewport()->update();
      refresh_home();
    }
    refresh_target_details(window, table, workspace_root);
  });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, window,
    [window, table, workspace_root]() { refresh_target_details(window, table, workspace_root); });

  home->setStyleSheet(QStringLiteral(
    "QWidget#studioTargetHomeContent{background:#F6F9FC;color:#102B4E;}"
    "QLabel#studioTargetHeading{color:#102B4E;font-size:23px;font-weight:900;}"
    "QLabel#studioTargetSubheading{color:#617A94;font-size:11px;}"
    "QFrame#studioTargetMetricCard{background:#FFFFFF;border:1px solid #DDE6EF;border-radius:8px;}"
    "QLabel#studioTargetMetricIcon{border-radius:18px;font-size:17px;font-weight:900;}"
    "QLabel#studioTargetMetricIcon[kind=total]{background:#EDF4FF;color:#1260E8;}"
    "QLabel#studioTargetMetricIcon[kind=ready]{background:#ECF8F1;color:#13804A;}"
    "QLabel#studioTargetMetricIcon[kind=attention]{background:#FFF5E8;color:#E46F08;}"
    "QLabel#studioTargetMetricIcon[kind=blocked]{background:#FDEEEF;color:#D62E3A;}"
    "QLabel#studioTargetMetricLabel{color:#173656;font-size:11px;font-weight:750;}"
    "QLabel#studioTargetMetricValue{color:#0D2F59;font-size:18px;font-weight:900;}"
    "QLabel#studioTargetMetricHint{color:#68809A;font-size:9px;}"
    "QLineEdit#studioTargetSearch,QComboBox#studioTargetFilter{background:#FFFFFF;color:#173656;border:1px solid #D4E0EB;border-radius:6px;min-height:38px;padding:0 11px;}"
    "QPushButton#studioTargetPinnedFilter,QComboBox#studioTargetSort{background:#FFFFFF;color:#173656;border:1px solid #D4E0EB;border-radius:6px;min-height:38px;padding:0 13px;font-weight:650;}"
    "QPushButton#studioTargetPinnedFilter:checked{background:#EEF4FF;color:#0C55C8;border-color:#AFC9EE;}"
    "QFrame#studioTargetTableCard{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:8px;}"
    "QTableWidget#studioHomeSceneTable{background:#FFFFFF;color:#173656;border:0;outline:0;}"
    "QTableWidget#studioHomeSceneTable::item{border:0;border-bottom:1px solid #E7EDF4;padding:0;}"
    "QTableWidget#studioHomeSceneTable::item:selected{background:#EDF4FD;color:#123F74;}"
    "QHeaderView::section{background:#F9FBFD;color:#35516F;border:0;border-bottom:1px solid #DCE5EF;padding:7px 9px;font-size:9px;font-weight:800;}"
    "QLabel#studioTargetTableFooter{color:#70849A;background:#FFFFFF;border:0;border-top:1px solid #E7EDF4;font-size:9px;}"
    "QFrame#studioTargetInspector{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:8px;}"
    "QLabel#studioTargetInspectorEyebrow{color:#173656;font-size:11px;font-weight:800;}"
    "QToolButton#studioTargetInspectorPin{background:transparent;color:#143B68;border:0;min-width:28px;max-width:28px;min-height:28px;max-height:28px;padding:0;font-size:17px;}"
    "QToolButton#studioTargetInspectorMore{background:#FFFFFF;color:#143B68;border:1px solid #C8D8EA;border-radius:5px;min-height:34px;padding:0 10px;font-size:10px;font-weight:750;}"
    "QLabel#studioTargetSelectedTitle{color:#102B4E;font-size:20px;font-weight:900;}"
    "QLabel#studioTargetSelectedId{color:#70849A;font-size:10px;}"
    "QLabel#studioTargetStatus{border-radius:8px;padding:3px 8px;font-size:9px;font-weight:800;}"
    "QLabel#studioTargetStatus[kind=neutral]{background:#EFF3F7;color:#66798C;border:1px solid #DCE4EB;}"
    "QLabel#studioTargetStatus[kind=ready]{background:#ECF8F1;color:#147A49;border:1px solid #C8E5D3;}"
    "QLabel#studioTargetStatus[kind=attention]{background:#FFF5E8;color:#C66708;border:1px solid #F1D1A6;}"
    "QLabel#studioTargetStatus[kind=blocked]{background:#FDEEEF;color:#BC2B34;border:1px solid #EFBEC2;}"
    "QLabel#studioTargetPreview{background:#121B24;color:#C4D0DC;border:1px solid #263847;border-radius:7px;font-size:10px;font-weight:700;}"
    "QWidget#studioTargetMetadata{background:transparent;}"
    "QLabel#studioTargetMetaKey{color:#536C86;font-size:10px;}"
    "QLabel#studioTargetMetaRobot,QLabel#studioTargetMetaTool,QLabel#studioTargetMetaTask,QLabel#studioTargetMetaLaunch,QLabel#studioTargetMetaUpdated{color:#173656;font-size:10px;font-weight:700;}"
    "QFrame#studioTargetReadinessCard{background:#FFF8EE;border:1px solid #F0DFC6;border-radius:7px;}"
    "QLabel#studioTargetReadiness{color:#364E66;font-size:10px;font-weight:600;}"
    "QPushButton#studioTargetViewValidation{background:transparent;color:#124A9D;border:0;padding:0;font-size:9px;font-weight:750;}"));

  refresh_home();
  if (total > 0) {
    table->setCurrentCell(0, 0);
  }
}

inline void configure_target_shell(QMainWindow * window, const QString & workspace_root)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (window->property("studioTargetShellApplied").toBool()) return;
  QTableWidget * table = scene_table(window);
  QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
  auto * pages = dashboard ? qobject_cast<QStackedWidget *>(dashboard->parentWidget()) : nullptr;
  if (!table || !dashboard || !pages) return;
  window->setProperty("studioTargetShellApplied", true);
  window->setMinimumSize(1180, 760);
  window->setWindowState(window->windowState() | Qt::WindowMaximized);
  build_application_shell(window, pages);
  build_home_page(window, dashboard, table, workspace_root);
}

}  // namespace home_workcells
}  // namespace workcell_builder
