#pragma once

// Production Home workcell-library presentation.
//
// This is intentionally a one-shot composition pass called explicitly from
// main.cpp after MainWindow construction and before QWidget::show().  It does
// not install an application event filter, startup function, timer, hidden 3D
// renderer, or overlapping per-cell widgets.

#include <QAbstractItemView>
#include <QApplication>
#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QDir>
#include <QFileInfo>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPainter>
#include <QPointer>
#include <QSplitter>
#include <QStyle>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

namespace workcell_builder
{
namespace home_workcells
{

inline QString friendly_workcell_name(QString id)
{
  id = id.trimmed();
  if (id.isEmpty()) return QStringLiteral("Workcell");
  QStringList words;
  for (QString token : id.split('_', Qt::SkipEmptyParts)) {
    const QString lower = token.toLower();
    if (QRegularExpression(QStringLiteral("^ur\\d+$"), QRegularExpression::CaseInsensitiveOption)
          .match(lower).hasMatch() ||
        lower == QStringLiteral("2f") || lower == QStringLiteral("3f")) {
      token = lower.toUpper();
    } else if (lower == QStringLiteral("airpick4")) {
      token = QStringLiteral("AirPick4");
    } else {
      token = lower;
      if (!token.isEmpty()) token[0] = token[0].toUpper();
    }
    words << token;
  }
  return words.join(' ');
}

inline QString clean_robot(QString robot)
{
  robot = robot.trimmed();
  if (robot.isEmpty() || robot.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0)
    return QStringLiteral("—");
  if (QRegularExpression(QStringLiteral("^ur\\d+$"), QRegularExpression::CaseInsensitiveOption)
        .match(robot).hasMatch()) return robot.toUpper();
  return robot;
}

inline QString clean_tool(QString tool)
{
  const QString lower = tool.trimmed().toLower();
  if (lower.isEmpty() || lower == QStringLiteral("unknown")) return QStringLiteral("—");
  if (lower == QStringLiteral("robotiq_85") || lower == QStringLiteral("robotiq_85_gripper") ||
      lower == QStringLiteral("robotiq_2f_85") || lower == QStringLiteral("robotiq_2f_85_gripper"))
    return QStringLiteral("Robotiq 2F-85");
  if (lower == QStringLiteral("single_suction")) return QStringLiteral("Single Suction");
  if (lower == QStringLiteral("airpick4") || lower == QStringLiteral("onrobot_airpick4"))
    return QStringLiteral("OnRobot AirPick4");
  tool = tool.trimmed();
  tool.replace('_', ' ');
  return tool;
}

inline QString clean_status(const QString & raw)
{
  const QString status = raw.trimmed().toUpper();
  if (status.contains(QStringLiteral("READY")) || status.contains(QStringLiteral("VALID")))
    return QStringLiteral("Ready");
  if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION")))
    return QStringLiteral("Needs Attention");
  if (status.contains(QStringLiteral("BLOCK")) || status.contains(QStringLiteral("FAIL")) ||
      status.contains(QStringLiteral("ERROR"))) return QStringLiteral("Blocked");
  return raw.trimmed().isEmpty() ? QStringLiteral("Unknown") : raw.trimmed();
}

inline QString clean_task(const QString & raw)
{
  const QString value = raw.trimmed().toLower();
  if (value == QStringLiteral("present") || value == QStringLiteral("ready")) return QStringLiteral("Configured");
  if (value == QStringLiteral("missing")) return QStringLiteral("Missing");
  return raw.trimmed().isEmpty() ? QStringLiteral("—") : raw.trimmed();
}

inline QString clean_launch(const QString & raw)
{
  const QString value = raw.trimmed().toLower();
  if (value == QStringLiteral("ready")) return QStringLiteral("Ready");
  if (value == QStringLiteral("blocked")) return QStringLiteral("Blocked");
  return raw.trimmed().isEmpty() ? QStringLiteral("—") : raw.trimmed();
}

class WorkcellTableDelegate final : public QStyledItemDelegate
{
public:
  explicit WorkcellTableDelegate(QObject * parent = nullptr) : QStyledItemDelegate(parent) {}

  void paint(QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index) const override
  {
    QStyleOptionViewItem opt(option);
    initStyleOption(&opt, index);
    const QString raw = index.data(Qt::DisplayRole).toString();
    switch (index.column()) {
      case 0: opt.text = friendly_workcell_name(raw); break;
      case 1: opt.text = clean_status(raw); break;
      case 2: opt.text = clean_robot(raw); break;
      case 3: opt.text = clean_tool(raw); break;
      case 4: opt.text = clean_task(raw); break;
      case 5: opt.text = clean_launch(raw); break;
      default: break;
    }

    if (index.column() == 0) {
      QFont font = opt.font;
      font.setBold(true);
      opt.font = font;
    }
    if (index.column() == 1 && !(opt.state & QStyle::State_Selected)) {
      const QString status = clean_status(raw);
      if (status == QStringLiteral("Ready")) {
        opt.palette.setColor(QPalette::Text, QColor(QStringLiteral("#137A46")));
        opt.backgroundBrush = QBrush(QColor(QStringLiteral("#ECF8F1")));
      } else if (status == QStringLiteral("Needs Attention")) {
        opt.palette.setColor(QPalette::Text, QColor(QStringLiteral("#A65B00")));
        opt.backgroundBrush = QBrush(QColor(QStringLiteral("#FFF7EA")));
      } else if (status == QStringLiteral("Blocked")) {
        opt.palette.setColor(QPalette::Text, QColor(QStringLiteral("#B42318")));
        opt.backgroundBrush = QBrush(QColor(QStringLiteral("#FFF0F0")));
      }
      QFont font = opt.font;
      font.setBold(true);
      opt.font = font;
    }

    const QWidget * widget = option.widget;
    QStyle * style = widget ? widget->style() : QApplication::style();
    style->drawControl(QStyle::CE_ItemViewItem, &opt, painter, widget);
  }
};

inline QTableWidget * scene_table(QMainWindow * window)
{
  return window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
}

inline QFrame * workcells_card(QTableWidget * table)
{
  return table ? qobject_cast<QFrame *>(table->parentWidget()) : nullptr;
}

inline QFrame * legacy_library_card(QMainWindow * window)
{
  if (!window) return nullptr;
  for (QLineEdit * edit : window->findChildren<QLineEdit *>()) {
    if (edit && edit->placeholderText() == QStringLiteral("Search library..."))
      return qobject_cast<QFrame *>(edit->parentWidget());
  }
  return nullptr;
}

inline QFrame * details_card(QMainWindow * window)
{
  if (!window) return nullptr;
  for (QFrame * frame : window->findChildren<QFrame *>(QStringLiteral("studioHomeDetailsCard"))) {
    if (frame) return frame;
  }
  return nullptr;
}

inline QString find_preview_path(const QString & workspace_root, const QString & scene_id)
{
  if (workspace_root.trimmed().isEmpty() || scene_id.trimmed().isEmpty()) return QString();
  QStringList scene_roots;
  const auto add_root = [&scene_roots](const QString & path) {
    QFileInfo info(path);
    if (!info.exists() || !info.isDir()) return;
    QString resolved = info.canonicalFilePath();
    if (resolved.isEmpty()) resolved = info.absoluteFilePath();
    resolved = QDir::cleanPath(resolved);
    if (!scene_roots.contains(resolved)) scene_roots.append(resolved);
  };
  add_root(QDir(workspace_root).filePath(QStringLiteral("src/easy_manipulation_deployment/scenes")));
  add_root(QDir(workspace_root).filePath(QStringLiteral("src/scenes")));

  const QStringList exact = {
    QStringLiteral("smoke/scene3d_gui_smoke.png"),
    QStringLiteral("acceptance/scene3d_gui_smoke.png"),
    QStringLiteral("smoke/scene3d_smoke.png"),
    QStringLiteral("acceptance/scene3d_smoke.png"),
    QStringLiteral("preview/scene3d_preview.png"),
    QStringLiteral("preview/workcell_studio_canvas_snapshot.png"),
    QStringLiteral("preview/static_preview.png"),
    QStringLiteral("preview_launch/product_view.png"),
    QStringLiteral("generated/scene3d_preview.png")};

  for (const QString & root : scene_roots) {
    const QDir scene_dir(QDir(root).filePath(scene_id));
    if (!scene_dir.exists()) continue;
    for (const QString & relative : exact) {
      const QString candidate = scene_dir.filePath(relative);
      if (QFileInfo(candidate).isFile()) return candidate;
    }
    for (const QString & subdir : {QStringLiteral("smoke"), QStringLiteral("acceptance"),
         QStringLiteral("preview"), QStringLiteral("preview_launch"), QStringLiteral("generated")}) {
      QDir image_dir(scene_dir.filePath(subdir));
      if (!image_dir.exists()) continue;
      const QStringList matches = image_dir.entryList(
        {QStringLiteral("*scene3d*.png"), QStringLiteral("*product*.png"),
         QStringLiteral("*preview*.png"), QStringLiteral("*snapshot*.png")},
        QDir::Files, QDir::Time);
      if (!matches.isEmpty()) return image_dir.filePath(matches.first());
    }
  }
  return QString();
}

inline void set_label(QMainWindow * window, const QString & object_name, const QString & text)
{
  if (auto * label = window ? window->findChild<QLabel *>(object_name) : nullptr) label->setText(text);
}

inline void refresh_details(QMainWindow * window, const QString & workspace_root)
{
  QTableWidget * table = scene_table(window);
  if (!table) return;
  const int row = table->currentRow();
  const bool selected = row >= 0 && row < table->rowCount() && table->item(row, 0);
  auto * preview = window->findChild<QLabel *>(QStringLiteral("studioHomeScenePreview"));
  auto * readiness = window->findChild<QLabel *>(QStringLiteral("studioHomeReadinessSummary"));
  auto * actions = window->findChild<QToolButton *>(QStringLiteral("studioHomeSecondaryButton"));

  if (!selected) {
    set_label(window, QStringLiteral("studioHomeSelectedTitle"), QStringLiteral("Select a workcell"));
    set_label(window, QStringLiteral("studioHomeSelectedId"), QString());
    set_label(window, QStringLiteral("studioHomeSelectedStatus"), QStringLiteral("No selection"));
    set_label(window, QStringLiteral("studioHomeMetaRobot"), QStringLiteral("—"));
    set_label(window, QStringLiteral("studioHomeMetaTool"), QStringLiteral("—"));
    set_label(window, QStringLiteral("studioHomeMetaTask"), QStringLiteral("—"));
    set_label(window, QStringLiteral("studioHomeMetaLaunch"), QStringLiteral("—"));
    if (preview) {
      preview->setPixmap(QPixmap());
      preview->setText(QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here"));
    }
    if (readiness) readiness->setText(QStringLiteral("Select a workcell to review its readiness and available workflow actions."));
    return;
  }

  const auto text_at = [table, row](int column) {
    return table->item(row, column) ? table->item(row, column)->text().trimmed() : QString();
  };
  const QString scene_id = text_at(0);
  const QString status = clean_status(text_at(1));
  const QString robot = clean_robot(text_at(2));
  const QString tool = clean_tool(text_at(3));
  const QString task = clean_task(text_at(4));
  const QString launch = clean_launch(text_at(5));

  set_label(window, QStringLiteral("studioHomeSelectedTitle"), friendly_workcell_name(scene_id));
  set_label(window, QStringLiteral("studioHomeSelectedId"), scene_id);
  set_label(window, QStringLiteral("studioHomeSelectedStatus"), status);
  set_label(window, QStringLiteral("studioHomeMetaRobot"), robot);
  set_label(window, QStringLiteral("studioHomeMetaTool"), tool);
  set_label(window, QStringLiteral("studioHomeMetaTask"), task);
  set_label(window, QStringLiteral("studioHomeMetaLaunch"), launch);

  if (auto * status_label = window->findChild<QLabel *>(QStringLiteral("studioHomeSelectedStatus"))) {
    status_label->setProperty("kind", status == QStringLiteral("Ready") ? QStringLiteral("ready") :
      status == QStringLiteral("Needs Attention") ? QStringLiteral("attention") : QStringLiteral("blocked"));
    status_label->style()->unpolish(status_label);
    status_label->style()->polish(status_label);
  }

  if (preview) {
    preview->setPixmap(QPixmap());
    const QString preview_path = find_preview_path(workspace_root, scene_id);
    const QPixmap pixmap(preview_path);
    if (!pixmap.isNull()) {
      preview->setText(QString());
      const QSize target(qMax(300, preview->width() - 8), qMax(200, preview->height() - 8));
      preview->setPixmap(pixmap.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
      preview->setToolTip(preview_path);
    } else {
      preview->setText(QStringLiteral("NO PREVIEW IMAGE\nOpen Product View to render this workcell"));
      preview->setToolTip(QStringLiteral("No smoke, acceptance, preview, or generated workcell image was found."));
    }
  }

  if (readiness) {
    if (status == QStringLiteral("Ready"))
      readiness->setText(QStringLiteral("Ready for the next fake-hardware workflow step."));
    else if (status == QStringLiteral("Needs Attention"))
      readiness->setText(QStringLiteral("Readiness checks need attention before simulation."));
    else
      readiness->setText(QStringLiteral("A blocker prevents the next safe workflow step."));
  }
  if (actions) actions->setToolTip(QStringLiteral("Workflow actions for the selected workcell"));
}

inline QLabel * metadata_value(QWidget * parent, const QString & object_name)
{
  auto * value = new QLabel(QStringLiteral("—"), parent);
  value->setObjectName(object_name);
  value->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return value;
}

inline void build_details_panel(QMainWindow * window, QFrame * card, const QString & workspace_root)
{
  if (!window || !card || card->findChild<QWidget *>(QStringLiteral("studioHomeProductDetails"))) return;
  auto * layout = qobject_cast<QVBoxLayout *>(card->layout());
  if (!layout) return;
  layout->setContentsMargins(18, 16, 18, 16);
  layout->setSpacing(9);

  for (QLabel * label : card->findChildren<QLabel *>(QString(), Qt::FindDirectChildrenOnly)) {
    if (label) label->hide();
  }

  auto * content = new QWidget(card);
  content->setObjectName(QStringLiteral("studioHomeProductDetails"));
  auto * body = new QVBoxLayout(content);
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(8);

  auto * top = new QHBoxLayout();
  auto * eyebrow = new QLabel(QStringLiteral("WORKCELL DETAILS"), content);
  eyebrow->setObjectName(QStringLiteral("studioHomeDetailsEyebrow"));
  top->addWidget(eyebrow);
  top->addStretch(1);
  QToolButton * actions = window->findChild<QToolButton *>(QStringLiteral("studioHomeSecondaryButton"));
  if (actions) {
    if (QLayout * old = actions->parentWidget() ? actions->parentWidget()->layout() : nullptr) old->removeWidget(actions);
    actions->setText(QStringLiteral("⋯"));
    actions->setToolTip(QStringLiteral("Workcell actions"));
    actions->setFixedSize(34, 30);
    top->addWidget(actions);
  }
  body->addLayout(top);

  auto * title = new QLabel(QStringLiteral("Select a workcell"), content);
  title->setObjectName(QStringLiteral("studioHomeSelectedTitle"));
  title->setWordWrap(true);
  body->addWidget(title);
  auto * id = new QLabel(content);
  id->setObjectName(QStringLiteral("studioHomeSelectedId"));
  body->addWidget(id);
  auto * status = new QLabel(QStringLiteral("No selection"), content);
  status->setObjectName(QStringLiteral("studioHomeSelectedStatus"));
  status->setProperty("kind", QStringLiteral("neutral"));
  status->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
  body->addWidget(status, 0, Qt::AlignLeft);

  auto * preview = new QLabel(QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here"), content);
  preview->setObjectName(QStringLiteral("studioHomeScenePreview"));
  preview->setAlignment(Qt::AlignCenter);
  preview->setMinimumHeight(220);
  preview->setMaximumHeight(260);
  preview->setScaledContents(false);
  body->addWidget(preview);

  auto * metadata = new QWidget(content);
  metadata->setObjectName(QStringLiteral("studioHomeMetadata"));
  auto * grid = new QGridLayout(metadata);
  grid->setContentsMargins(2, 4, 2, 4);
  grid->setHorizontalSpacing(12);
  grid->setVerticalSpacing(7);
  const QStringList keys = {QStringLiteral("Robot"), QStringLiteral("Tool / Gripper"),
    QStringLiteral("Task"), QStringLiteral("Launch (Fake Hardware)")};
  const QStringList objects = {QStringLiteral("studioHomeMetaRobot"), QStringLiteral("studioHomeMetaTool"),
    QStringLiteral("studioHomeMetaTask"), QStringLiteral("studioHomeMetaLaunch")};
  for (int row = 0; row < keys.size(); ++row) {
    auto * key = new QLabel(keys[row], metadata);
    key->setObjectName(QStringLiteral("studioHomeMetaKey"));
    grid->addWidget(key, row, 0);
    grid->addWidget(metadata_value(metadata, objects[row]), row, 1);
  }
  grid->setColumnStretch(1, 1);
  body->addWidget(metadata);

  auto * readiness_card = new QFrame(content);
  readiness_card->setObjectName(QStringLiteral("studioHomeReadinessCard"));
  auto * readiness_layout = new QVBoxLayout(readiness_card);
  readiness_layout->setContentsMargins(12, 10, 12, 10);
  auto * readiness = new QLabel(QStringLiteral("Select a workcell to review its readiness and available workflow actions."), readiness_card);
  readiness->setObjectName(QStringLiteral("studioHomeReadinessSummary"));
  readiness->setWordWrap(true);
  readiness_layout->addWidget(readiness);
  body->addWidget(readiness_card);
  body->addStretch(1);

  layout->insertWidget(0, content, 1);
  refresh_details(window, workspace_root);
}

inline void configure(QMainWindow * window, const QString & workspace_root)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (window->property("homeWorkcellsProductionLayoutApplied").toBool()) return;
  QTableWidget * table = scene_table(window);
  QFrame * center = workcells_card(table);
  QFrame * details = details_card(window);
  if (!table || !center || !details) return;
  window->setProperty("homeWorkcellsProductionLayoutApplied", true);

  if (QFrame * legacy = legacy_library_card(window)) legacy->hide();
  if (auto * source = window->findChild<QLabel *>(QStringLiteral("studioHomeDetailsCard"))) source->hide();

  center->setObjectName(QStringLiteral("studioHomeWorkcellsCard"));
  if (auto * layout = qobject_cast<QVBoxLayout *>(center->layout())) {
    layout->setContentsMargins(14, 12, 14, 14);
    layout->setSpacing(9);
  }
  for (QLabel * label : center->findChildren<QLabel *>(QString(), Qt::FindDirectChildrenOnly)) {
    if (label && label->text().contains(QStringLiteral("Scenes"), Qt::CaseInsensitive)) {
      label->setText(QStringLiteral("<b>Workcells</b>"));
      label->setObjectName(QStringLiteral("studioHomeWorkcellsHeading"));
    }
  }
  if (auto * search = window->findChild<QLineEdit *>(QStringLiteral("studioHomeSearchBox"))) {
    search->setPlaceholderText(QStringLiteral("Search workcells..."));
    search->setClearButtonEnabled(true);
  }
  if (auto * filter = window->findChild<QComboBox *>(QStringLiteral("studioHomeStatusFilter"))) {
    if (filter->count() > 0) filter->setItemText(0, QStringLiteral("Status: All"));
  }

  table->setItemDelegate(new WorkcellTableDelegate(table));
  table->setHorizontalHeaderLabels({QStringLiteral("Workcell"), QStringLiteral("Status"), QStringLiteral("Robot"),
    QStringLiteral("Tool / Gripper"), QStringLiteral("Task"), QStringLiteral("Launch")});
  table->setShowGrid(false);
  table->setAlternatingRowColors(false);
  table->setWordWrap(false);
  table->setTextElideMode(Qt::ElideRight);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->verticalHeader()->hide();
  table->verticalHeader()->setDefaultSectionSize(46);
  table->horizontalHeader()->setMinimumHeight(38);
  table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Fixed);
  table->setColumnWidth(1, 145);
  table->setColumnWidth(2, 90);
  table->setColumnWidth(3, 160);
  table->setColumnWidth(4, 105);
  table->setColumnWidth(5, 92);

  if (auto * splitter = qobject_cast<QSplitter *>(center->parentWidget())) {
    splitter->setCollapsible(0, true);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 7);
    splitter->setStretchFactor(2, 3);
    splitter->setSizes({0, 1040, 380});
    details->setMinimumWidth(340);
    details->setMaximumWidth(420);
  }

  build_details_panel(window, details, workspace_root);

  center->setStyleSheet(QStringLiteral(
    "QFrame#studioHomeWorkcellsCard{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:10px;}"
    "QLabel#studioHomeWorkcellsHeading{color:#102B46;font-size:15px;font-weight:800;border:0;}"
    "QTableWidget#studioHomeSceneTable{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:7px;outline:0;}"
    "QTableWidget#studioHomeSceneTable::item{border-bottom:1px solid #EDF2F6;padding:6px 9px;color:#263C52;}"
    "QTableWidget#studioHomeSceneTable::item:selected{background:#EAF2FB;color:#153D62;}"
    "QHeaderView::section{background:#F7F9FC;color:#465B70;border:0;border-bottom:1px solid #DCE5EF;padding:7px 8px;font-weight:700;}"));
  details->setStyleSheet(QStringLiteral(
    "QFrame#studioHomeDetailsCard{background:#FFFFFF;border:1px solid #DCE5EF;border-radius:10px;}"
    "QLabel#studioHomeDetailsEyebrow{color:#6C7E91;font-size:10px;font-weight:800;letter-spacing:1px;}"
    "QLabel#studioHomeSelectedTitle{color:#102B46;font-size:20px;font-weight:800;}"
    "QLabel#studioHomeSelectedId{color:#71869A;font-size:11px;}"
    "QLabel#studioHomeSelectedStatus{border-radius:9px;padding:3px 8px;font-size:10px;font-weight:800;}"
    "QLabel#studioHomeSelectedStatus[kind=neutral]{background:#EEF2F7;color:#64748B;border:1px solid #D9E1E9;}"
    "QLabel#studioHomeSelectedStatus[kind=ready]{background:#EAF7F0;color:#137A46;border:1px solid #BBDDC9;}"
    "QLabel#studioHomeSelectedStatus[kind=attention]{background:#FFF4E5;color:#A45500;border:1px solid #EDB879;}"
    "QLabel#studioHomeSelectedStatus[kind=blocked]{background:#FDECEC;color:#B42318;border:1px solid #E6B3AF;}"
    "QLabel#studioHomeScenePreview{background:#121D28;color:#C7D3DF;border:1px solid #243747;border-radius:8px;font-size:11px;font-weight:700;padding:4px;}"
    "QWidget#studioHomeMetadata{background:transparent;border:0;}"
    "QLabel#studioHomeMetaKey{color:#64788D;font-size:11px;}"
    "QLabel#studioHomeMetaRobot,QLabel#studioHomeMetaTool,QLabel#studioHomeMetaTask,QLabel#studioHomeMetaLaunch{color:#17324C;font-size:11px;font-weight:700;}"
    "QFrame#studioHomeReadinessCard{background:#FFF8EE;border:1px solid #F0DFC6;border-radius:8px;}"
    "QLabel#studioHomeReadinessSummary{color:#3D5267;font-size:11px;background:transparent;border:0;}"
    "QToolButton#studioHomeSecondaryButton{background:#F6F8FB;color:#173B5D;border:1px solid #D8E2EC;border-radius:6px;padding:2px;font-size:18px;}"
    "QToolButton#studioHomeSecondaryButton:hover{background:#EAF1F7;}"));

  const QPointer<QMainWindow> safe_window(window);
  QObject::connect(table, &QTableWidget::cellClicked, window,
    [safe_window, workspace_root](int, int) {
      if (safe_window) refresh_details(safe_window, workspace_root);
    });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, window,
    [safe_window, workspace_root]() {
      if (safe_window) refresh_details(safe_window, workspace_root);
    });
}

}  // namespace home_workcells
}  // namespace workcell_builder
