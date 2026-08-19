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
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
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
#include <QStatusBar>
#include <QStyle>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include "home_workcells_panel.hpp"

namespace workcell_builder
{
namespace home_workcells
{

constexpr int kPinnedRole = Qt::UserRole + 37;

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
  QDateTime latest = QFileInfo(scene_dir).lastModified();
  const QStringList evidence = {
    QStringLiteral("layout/workcell_studio_layout.yaml"),
    QStringLiteral("generated/scene_preview_metadata.json"),
    QStringLiteral("generated/scene_visual_mesh_index.json"),
    QStringLiteral("generated/scene_manifest.yaml"),
    QStringLiteral("config/moveit_collision_objects.yaml"),
    QStringLiteral("task/task_intent.yaml"),
    QStringLiteral("task/task_recipe.yaml"),
    QStringLiteral("smoke/scene3d_gui_smoke.png"),
    QStringLiteral("acceptance/scene3d_gui_smoke.png")};
  for (const QString & relative : evidence) {
    const QFileInfo info(QDir(scene_dir).filePath(relative));
    if (info.exists() && info.lastModified().isValid() && (!latest.isValid() || info.lastModified() > latest))
      latest = info.lastModified();
  }
  return latest;
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

    if (index.column() == 0) {
      const QRect thumb(r.left() + 10, r.top() + 7, 70, r.height() - 14);
      painter->setPen(QPen(QColor(QStringLiteral("#DCE5EF")), 1));
      painter->setBrush(QColor(QStringLiteral("#F7F9FC")));
      painter->drawRoundedRect(thumb, 5, 5);
      const QString preview_path = find_preview_path(workspace_root_, raw_id);
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
        friendly_workcell_name(raw_id));
      QFont id_font = option.font;
      id_font.setPointSizeF(qMax(8.0, option.font.pointSizeF() - 1.0));
      painter->setFont(id_font);
      painter->setPen(muted);
      painter->drawText(text_rect.adjusted(0, 23, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, raw_id);
    } else if (index.column() == 1) {
      const QString status = clean_status(index.data(Qt::DisplayRole).toString());
      QColor accent(QStringLiteral("#D97706"));
      QString icon = QStringLiteral("△");
      QString detail = QStringLiteral("Review required");
      if (status == QStringLiteral("Ready")) {
        accent = QColor(QStringLiteral("#14804A")); icon = QStringLiteral("✓"); detail = QStringLiteral("Ready to simulate");
      } else if (status == QStringLiteral("Blocked")) {
        accent = QColor(QStringLiteral("#C52B2B")); icon = QStringLiteral("×"); detail = QStringLiteral("Action required");
      }
      QFont strong = option.font; strong.setBold(true);
      painter->setFont(strong); painter->setPen(accent);
      painter->drawText(r.adjusted(10, 7, -5, -24), Qt::AlignLeft | Qt::AlignVCenter, icon + QStringLiteral("  ") + status);
      QFont small = option.font; small.setPointSizeF(qMax(8.0, option.font.pointSizeF() - 1.0));
      painter->setFont(small); painter->setPen(muted);
      painter->drawText(r.adjusted(30, 28, -5, -4), Qt::AlignLeft | Qt::AlignVCenter, detail);
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
      QFont star = option.font; star.setPointSizeF(option.font.pointSizeF() + 5.0); star.setBold(true);
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
  row->setContentsMargins(12, 10, 14, 10);
  row->setSpacing(10);
  auto * icon_label = new QLabel(icon, card);
  icon_label->setObjectName(QStringLiteral("studioTargetMetricIcon"));
  icon_label->setProperty("kind", kind);
  icon_label->setAlignment(Qt::AlignCenter);
  icon_label->setFixedSize(48, 48);
  row->addWidget(icon_label);
  auto * copy = new QVBoxLayout();
  copy->setSpacing(0);
  auto * title = new QLabel(label, card); title->setObjectName(QStringLiteral("studioTargetMetricLabel"));
  auto * value = new QLabel(QString::number(count), card); value->setObjectName(QStringLiteral("studioTargetMetricValue"));
  auto * sub = new QLabel(hint, card); sub->setObjectName(QStringLiteral("studioTargetMetricHint"));
  copy->addWidget(title); copy->addWidget(value); copy->addWidget(sub);
  row->addLayout(copy, 1);
  return card;
}

inline void apply_extra_filters(QTableWidget * table, QComboBox * robot, QComboBox * tool,
  QPushButton * pinned, QLabel * footer)
{
  if (!table) return;
  const QString robot_filter = robot ? robot->currentText() : QStringLiteral("Robot: All");
  const QString tool_filter = tool ? tool->currentText() : QStringLiteral("Tool: All");
  const bool pinned_only = pinned && pinned->isChecked();
  int visible = 0;
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString robot_value = table->item(row, 2) ? clean_robot(table->item(row, 2)->text()) : QStringLiteral("—");
    const QString tool_value = table->item(row, 3) ? clean_tool(table->item(row, 3)->text()) : QStringLiteral("—");
    bool show = true;
    if (robot && robot->currentIndex() > 0 && robot_filter != robot_value) show = false;
    if (tool && tool->currentIndex() > 0 && tool_filter != tool_value) show = false;
    if (pinned_only && !scene_is_pinned(table, row)) show = false;
    table->setRowHidden(row, !show);
    if (show) ++visible;
  }
  if (footer) {
    const int total = table->property("studioTargetTotalWorkcells").toInt();
    footer->setText(QStringLiteral("Showing %1 of %2 workcells").arg(visible).arg(total > 0 ? total : table->rowCount()));
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

  set_label(window, QStringLiteral("studioTargetSelectedTitle"), selected ? friendly_workcell_name(scene_id) : QStringLiteral("Select a workcell"));
  set_label(window, QStringLiteral("studioTargetSelectedId"), scene_id);
  set_label(window, QStringLiteral("studioTargetStatus"), status);
  set_label(window, QStringLiteral("studioTargetMetaRobot"), robot);
  set_label(window, QStringLiteral("studioTargetMetaTool"), tool);
  set_label(window, QStringLiteral("studioTargetMetaTask"), task == QStringLiteral("Configured") ? QStringLiteral("Pick & Place") : task);
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
    const QString path = selected ? find_preview_path(workspace_root, scene_id) : QString();
    const QPixmap pixmap(path);
    if (!pixmap.isNull()) {
      preview->setText(QString());
      preview->setPixmap(pixmap.scaled(QSize(qMax(320, preview->width() - 8), 250), Qt::KeepAspectRatio, Qt::SmoothTransformation));
      preview->setToolTip(path);
    } else {
      preview->setText(selected ? QStringLiteral("NO PREVIEW IMAGE\nOpen Product View to render this workcell") :
        QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here"));
      preview->setToolTip(QString());
    }
  }

  if (auto * readiness = window->findChild<QLabel *>(QStringLiteral("studioTargetReadiness"))) {
    if (!selected) readiness->setText(QStringLiteral("Select a workcell to review readiness."));
    else if (status == QStringLiteral("Ready")) readiness->setText(QStringLiteral("Ready for fake-hardware simulation.\nView details →"));
    else if (status == QStringLiteral("Needs Attention")) readiness->setText(QStringLiteral("Readiness checks need attention.\nView details →"));
    else readiness->setText(QStringLiteral("A blocker prevents the next safe workflow step.\nView details →"));
  }

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
  inspector->setMinimumWidth(355);
  inspector->setMaximumWidth(405);
  auto * layout = new QVBoxLayout(inspector);
  layout->setContentsMargins(16, 14, 16, 16);
  layout->setSpacing(8);

  auto * header = new QHBoxLayout();
  auto * title = new QLabel(QStringLiteral("Selected Workcell"), inspector);
  title->setObjectName(QStringLiteral("studioTargetInspectorEyebrow"));
  header->addWidget(title); header->addStretch(1);
  auto * pin = new QToolButton(inspector); pin->setObjectName(QStringLiteral("studioTargetInspectorPin")); pin->setText(QStringLiteral("☆"));
  auto * more = new QToolButton(inspector); more->setObjectName(QStringLiteral("studioTargetInspectorMore")); more->setText(QStringLiteral("⋮"));
  if (auto * old_actions = window->findChild<QToolButton *>(QStringLiteral("studioHomeSecondaryButton"))) {
    if (old_actions->menu()) more->setMenu(old_actions->menu());
    more->setPopupMode(QToolButton::InstantPopup);
    old_actions->hide();
  }
  header->addWidget(pin); header->addWidget(more);
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
  layout->addWidget(preview);

  auto * meta = new QWidget(inspector); meta->setObjectName(QStringLiteral("studioTargetMetadata"));
  auto * grid = new QGridLayout(meta); grid->setContentsMargins(0, 4, 0, 4); grid->setHorizontalSpacing(16); grid->setVerticalSpacing(7);
  const QStringList keys = {QStringLiteral("♙   Robot"), QStringLiteral("⌘   Tool / Gripper"), QStringLiteral("▣   Task"), QStringLiteral("◉   Launch (Fake Hardware)"), QStringLiteral("◷   Last Updated")};
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
  readiness_layout->addWidget(readiness); layout->addWidget(readiness_card);

  auto * open = new QPushButton(QStringLiteral("▣  Open in Scene Builder"), inspector); open->setObjectName(QStringLiteral("studioTargetPrimaryAction"));
  layout->addWidget(open);
  auto * secondary = new QHBoxLayout();
  auto * validate = new QPushButton(QStringLiteral("♢  Validate"), inspector); validate->setObjectName(QStringLiteral("studioTargetSecondaryAction"));
  auto * generate = new QPushButton(QStringLiteral("◇  Generate Package"), inspector); generate->setObjectName(QStringLiteral("studioTargetSecondaryAction"));
  secondary->addWidget(validate); secondary->addWidget(generate); layout->addLayout(secondary);
  auto * simulate = new QPushButton(QStringLiteral("▷  Simulate · Fake Hardware"), inspector); simulate->setObjectName(QStringLiteral("studioTargetSimulateAction"));
  layout->addWidget(simulate);

  const auto menu_action = [more](const QString & text) -> QAction * {
    if (!more || !more->menu()) return nullptr;
    for (QAction * action : more->menu()->actions())
      if (action && action->text().contains(text, Qt::CaseInsensitive)) return action;
    return nullptr;
  };
  if (QAction * a = menu_action(QStringLiteral("Open in Scene Builder"))) QObject::connect(open, &QPushButton::clicked, a, &QAction::trigger);
  else open->setEnabled(false);
  if (QAction * a = menu_action(QStringLiteral("Validate"))) QObject::connect(validate, &QPushButton::clicked, a, &QAction::trigger);
  else validate->setEnabled(false);
  if (QAction * a = menu_action(QStringLiteral("Plan / Simulate"))) QObject::connect(simulate, &QPushButton::clicked, a, &QAction::trigger);
  else simulate->setEnabled(false);
  if (QAction * a = action_with_text(window, {QStringLiteral("Generate Scene Package"), QStringLiteral("Generate Workcell Package")}))
    QObject::connect(generate, &QPushButton::clicked, a, &QAction::trigger);
  else generate->setEnabled(false);

  QObject::connect(pin, &QToolButton::clicked, window, [window, table, pin]() {
    const int row = table ? table->currentRow() : -1;
    if (!table || row < 0 || row >= table->rowCount() || !table->item(row, 0)) return;
    const bool next = !scene_is_pinned(table, row);
    table->item(row, 0)->setData(kPinnedRole, next);
    QSettings().setValue(QStringLiteral("home/pinned/%1").arg(scene_id_at(table, row)), next);
    pin->setText(next ? QStringLiteral("★") : QStringLiteral("☆"));
    table->viewport()->update();
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

  auto * tools = new QHBoxLayout(); tools->setSpacing(9);
  auto * search = window->findChild<QLineEdit *>(QStringLiteral("studioHomeSearchBox"));
  auto * status = window->findChild<QComboBox *>(QStringLiteral("studioHomeStatusFilter"));
  if (search) {
    if (search->parentWidget() && search->parentWidget()->layout()) search->parentWidget()->layout()->removeWidget(search);
    search->setParent(left); search->setPlaceholderText(QStringLiteral("⌕  Search workcells...")); search->setClearButtonEnabled(true); search->setMinimumWidth(230); search->setObjectName(QStringLiteral("studioTargetSearch"));
    tools->addWidget(search, 2);
  }
  if (status) {
    if (status->parentWidget() && status->parentWidget()->layout()) status->parentWidget()->layout()->removeWidget(status);
    status->setParent(left); if (status->count() > 0) status->setItemText(0, QStringLiteral("Status: All")); status->setObjectName(QStringLiteral("studioTargetFilter")); tools->addWidget(status);
  }
  auto * robot = new QComboBox(left); robot->setObjectName(QStringLiteral("studioTargetFilter")); robot->addItem(QStringLiteral("Robot: All"));
  auto * tool = new QComboBox(left); tool->setObjectName(QStringLiteral("studioTargetFilter")); tool->addItem(QStringLiteral("Tool: All"));
  QSet<QString> robots, tools_set;
  for (int row = 0; row < table->rowCount(); ++row) {
    if (table->item(row, 2)) robots.insert(clean_robot(table->item(row, 2)->text()));
    if (table->item(row, 3)) tools_set.insert(clean_tool(table->item(row, 3)->text()));
  }
  for (const QString & value : robots) if (value != QStringLiteral("—")) robot->addItem(value);
  for (const QString & value : tools_set) if (value != QStringLiteral("—")) tool->addItem(value);
  tools->addWidget(robot); tools->addWidget(tool);
  auto * pinned = new QPushButton(QStringLiteral("☆  Pinned"), left); pinned->setObjectName(QStringLiteral("studioTargetPinnedFilter")); pinned->setCheckable(true); tools->addWidget(pinned);
  tools->addStretch(1);
  auto * sort = new QPushButton(QStringLiteral("Sort: Recently updated   ≡"), left); sort->setObjectName(QStringLiteral("studioTargetSort")); tools->addWidget(sort);
  main->addLayout(tools);

  auto * table_card = new QFrame(left); table_card->setObjectName(QStringLiteral("studioTargetTableCard"));
  auto * table_layout = new QVBoxLayout(table_card); table_layout->setContentsMargins(0, 0, 0, 0); table_layout->setSpacing(0);
  if (table->parentWidget() && table->parentWidget()->layout()) table->parentWidget()->layout()->removeWidget(table);
  table->setParent(table_card);
  table->setItemDelegate(new TargetWorkcellDelegate(workspace_root, table));
  table->setHorizontalHeaderLabels({QStringLiteral("Workcell"), QStringLiteral("Status"), QStringLiteral("Robot"), QStringLiteral("Tool / Gripper"), QStringLiteral("Updated"), QStringLiteral("Pinned")});
  table->setShowGrid(false); table->setAlternatingRowColors(false); table->setWordWrap(false); table->setSelectionBehavior(QAbstractItemView::SelectRows); table->setSelectionMode(QAbstractItemView::SingleSelection); table->verticalHeader()->hide(); table->verticalHeader()->setDefaultSectionSize(62); table->horizontalHeader()->setMinimumHeight(38); table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch); table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed); table->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Fixed); table->setColumnWidth(1, 165); table->setColumnWidth(2, 95); table->setColumnWidth(3, 155); table->setColumnWidth(4, 125); table->setColumnWidth(5, 70);
  table_layout->addWidget(table, 1);
  auto * footer = new QLabel(QStringLiteral("Showing %1 of %1 workcells").arg(total), table_card); footer->setObjectName(QStringLiteral("studioTargetTableFooter")); footer->setContentsMargins(12, 6, 12, 6); table_layout->addWidget(footer);
  main->addWidget(table_card, 1);

  auto * inspector = build_inspector(window, table, workspace_root, home);
  page->addWidget(left, 1); page->addWidget(inspector);
  root->addWidget(home, 1);

  const auto apply_filters = [table, robot, tool, pinned, footer]() { apply_extra_filters(table, robot, tool, pinned, footer); };
  QObject::connect(robot, qOverload<int>(&QComboBox::currentIndexChanged), table, [apply_filters](int) { apply_filters(); });
  QObject::connect(tool, qOverload<int>(&QComboBox::currentIndexChanged), table, [apply_filters](int) { apply_filters(); });
  QObject::connect(pinned, &QPushButton::toggled, table, [apply_filters, pinned](bool on) { pinned->setText(on ? QStringLiteral("★  Pinned") : QStringLiteral("☆  Pinned")); apply_filters(); });
  if (search) QObject::connect(search, &QLineEdit::textChanged, table, [table, apply_filters](const QString &) { restore_pinned_state(table); apply_filters(); table->viewport()->update(); });
  if (status) QObject::connect(status, qOverload<int>(&QComboBox::currentIndexChanged), table, [table, apply_filters](int) { restore_pinned_state(table); apply_filters(); table->viewport()->update(); });
  QObject::connect(table, &QTableWidget::cellClicked, window, [window, table, workspace_root, apply_filters](int row, int column) {
    if (column == 5 && table->item(row, 0)) {
      const bool next = !scene_is_pinned(table, row);
      table->item(row, 0)->setData(kPinnedRole, next);
      QSettings().setValue(QStringLiteral("home/pinned/%1").arg(scene_id_at(table, row)), next);
      table->viewport()->update();
      apply_filters();
    }
    refresh_target_details(window, table, workspace_root);
  });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, window, [window, table, workspace_root]() { refresh_target_details(window, table, workspace_root); });

  home->setStyleSheet(QStringLiteral(
    "QWidget#studioTargetHomeContent{background:#F6F9FC;color:#102B4E;}"
    "QLabel#studioTargetHeading{color:#102B4E;font-size:23px;font-weight:900;}"
    "QLabel#studioTargetSubheading{color:#617A94;font-size:11px;}"
    "QFrame#studioTargetMetricCard{background:#FFFFFF;border:1px solid #DDE6EF;border-radius:8px;}"
    "QLabel#studioTargetMetricIcon{border-radius:24px;font-size:22px;font-weight:900;}"
    "QLabel#studioTargetMetricIcon[kind=total]{background:#EDF4FF;color:#1260E8;}"
    "QLabel#studioTargetMetricIcon[kind=ready]{background:#ECF8F1;color:#13804A;}"
    "QLabel#studioTargetMetricIcon[kind=attention]{background:#FFF5E8;color:#E46F08;}"
    "QLabel#studioTargetMetricIcon[kind=blocked]{background:#FDEEEF;color:#D62E3A;}"
    "QLabel#studioTargetMetricLabel{color:#173656;font-size:11px;font-weight:750;}"
    "QLabel#studioTargetMetricValue{color:#0D2F59;font-size:23px;font-weight:900;}"
    "QLabel#studioTargetMetricHint{color:#68809A;font-size:9px;}"
    "QLineEdit#studioTargetSearch,QComboBox#studioTargetFilter{background:#FFFFFF;color:#173656;border:1px solid #D4E0EB;border-radius:6px;min-height:38px;padding:0 11px;}"
    "QPushButton#studioTargetPinnedFilter,QPushButton#studioTargetSort{background:#FFFFFF;color:#173656;border:1px solid #D4E0EB;border-radius:6px;min-height:38px;padding:0 13px;font-weight:650;}"
    "QPushButton#studioTargetPinnedFilter:checked{background:#EEF4FF;color:#0C55C8;border-color:#AFC9EE;}"
    "QFrame#studioTargetTableCard{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:8px;}"
    "QTableWidget#studioHomeSceneTable{background:#FFFFFF;color:#173656;border:0;outline:0;}"
    "QTableWidget#studioHomeSceneTable::item{border:0;border-bottom:1px solid #E7EDF4;padding:0;}"
    "QTableWidget#studioHomeSceneTable::item:selected{background:#EDF4FD;color:#123F74;}"
    "QHeaderView::section{background:#F9FBFD;color:#35516F;border:0;border-bottom:1px solid #DCE5EF;padding:7px 9px;font-size:9px;font-weight:800;}"
    "QLabel#studioTargetTableFooter{color:#70849A;background:#FFFFFF;border:0;border-top:1px solid #E7EDF4;font-size:9px;}"
    "QFrame#studioTargetInspector{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:8px;}"
    "QLabel#studioTargetInspectorEyebrow{color:#173656;font-size:11px;font-weight:800;}"
    "QToolButton#studioTargetInspectorPin,QToolButton#studioTargetInspectorMore{background:transparent;color:#143B68;border:0;min-width:28px;max-width:28px;min-height:28px;max-height:28px;padding:0;font-size:17px;}"
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
    "QPushButton#studioTargetPrimaryAction{background:#0B4698;color:#FFFFFF;border:1px solid #0B4698;border-radius:5px;min-height:38px;font-weight:800;}"
    "QPushButton#studioTargetPrimaryAction:hover{background:#0C55B8;}"
    "QPushButton#studioTargetSecondaryAction{background:#FFFFFF;color:#124A9D;border:1px solid #C8D8EA;border-radius:5px;min-height:36px;font-weight:750;}"
    "QPushButton#studioTargetSimulateAction{background:#FFFFFF;color:#16804D;border:1px solid #A8D7BE;border-radius:5px;min-height:38px;font-weight:800;}"));

  if (total > 0) {
    table->setCurrentCell(0, 0);
    QMetaObject::invokeMethod(table, "cellClicked", Qt::DirectConnection, Q_ARG(int, 0), Q_ARG(int, 0));
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
