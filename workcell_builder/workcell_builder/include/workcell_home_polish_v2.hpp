#pragma once

#include <QAbstractItemModel>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QPixmap>
#include <QPushButton>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

namespace workcell_builder
{
namespace home_polish_v2
{
constexpr int kSceneColumn = 0;
constexpr int kStatusColumn = 1;
constexpr int kRobotColumn = 2;
constexpr int kToolColumn = 3;
constexpr int kTaskColumn = 4;
constexpr int kLaunchColumn = 5;
constexpr int kUpdatedColumn = 6;
constexpr int kPinColumn = 7;

inline QLayout * findLayoutContaining(QLayout * root, QWidget * target)
{
  if (!root || !target) return nullptr;
  for (int i = 0; i < root->count(); ++i) {
    QLayoutItem * item = root->itemAt(i);
    if (!item) continue;
    if (item->widget() == target) return root;
    if (QLayout * child = item->layout()) {
      if (QLayout * found = findLayoutContaining(child, target)) return found;
    }
  }
  return nullptr;
}

inline QString cleanStatusText(QString text)
{
  text.replace(QStringLiteral("▲"), QString());
  text.replace(QStringLiteral("●"), QString());
  return text.simplified();
}

inline QString sceneNameAt(QTableWidget * table, int row)
{
  if (!table || row < 0 || row >= table->rowCount()) return QString();
  QTableWidgetItem * item = table->item(row, kSceneColumn);
  if (!item) return QString();
  const QString tooltip = item->toolTip().trimmed();
  return tooltip.isEmpty() ? item->text().trimmed() : tooltip;
}

inline QString sceneRoot(QMainWindow * window)
{
  if (!window) return QString();
  if (auto * summary = window->findChild<QLabel *>(QStringLiteral("dashboardSummaryLabel"))) {
    const QRegularExpression re(QStringLiteral("Source:\\s*([^|\\n]+)"));
    const auto match = re.match(summary->text());
    if (match.hasMatch()) return match.captured(1).trimmed();
  }
  if (auto * footer = window->findChild<QLabel *>(QStringLiteral("studioHomeFooter"))) {
    const QRegularExpression re(QStringLiteral("Source:\\s*([^·\\n]+)"));
    const auto match = re.match(footer->text());
    if (match.hasMatch()) return match.captured(1).trimmed();
  }
  return QString();
}

inline void triggerAction(QMainWindow * window, const QStringList & candidates)
{
  if (!window) return;
  for (QAction * action : window->findChildren<QAction *>()) {
    if (!action) continue;
    QString text = action->text();
    text.remove('&');
    for (const QString & candidate : candidates) {
      if (text.compare(candidate, Qt::CaseInsensitive) == 0 && action->isEnabled()) {
        action->trigger();
        return;
      }
    }
  }
}

inline QLabel * makeKpiValue(QWidget * parent, const QString & object_name, const QString & color)
{
  auto * value = new QLabel(QStringLiteral("0"), parent);
  value->setObjectName(object_name);
  value->setStyleSheet(QStringLiteral("font-size:26px;font-weight:800;color:%1;background:transparent;border:0;").arg(color));
  return value;
}

inline QFrame * makeKpiCard(
  QWidget * parent, const QString & object_name, const QString & title,
  const QString & value_object_name, const QString & detail, const QString & color)
{
  auto * card = new QFrame(parent);
  card->setObjectName(object_name);
  card->setMinimumHeight(86);
  auto * layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 11, 14, 10);
  layout->setSpacing(1);
  auto * title_label = new QLabel(title, card);
  title_label->setObjectName(QStringLiteral("homeV2KpiTitle"));
  auto * value = makeKpiValue(card, value_object_name, color);
  auto * detail_label = new QLabel(detail, card);
  detail_label->setObjectName(QStringLiteral("homeV2KpiDetail"));
  layout->addWidget(title_label);
  layout->addWidget(value);
  layout->addWidget(detail_label);
  return card;
}

inline void updateKpis(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (!table) return;
  int ready = 0;
  int attention = 0;
  int blocked = 0;
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString status = table->item(row, kStatusColumn)
      ? table->item(row, kStatusColumn)->text().toUpper()
      : QString();
    if (status.contains(QStringLiteral("READY"))) ++ready;
    else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) ++attention;
    else ++blocked;
  }
  const struct { const char * name; int value; } values[] = {
    {"homeV2TotalValue", table->rowCount()},
    {"homeV2ReadyValue", ready},
    {"homeV2AttentionValue", attention},
    {"homeV2BlockedValue", blocked}};
  for (const auto & entry : values) {
    if (auto * label = window->findChild<QLabel *>(QString::fromLatin1(entry.name))) {
      label->setText(QString::number(entry.value));
    }
  }
}

inline QLabel * ensureStatusPill(QTableWidget * table, int row)
{
  if (!table || row < 0 || row >= table->rowCount()) return nullptr;
  if (auto * existing = qobject_cast<QLabel *>(table->cellWidget(row, kStatusColumn))) return existing;
  auto * pill = new QLabel(table);
  pill->setObjectName(QStringLiteral("homeV2StatusPill"));
  pill->setAlignment(Qt::AlignCenter);
  pill->setMinimumWidth(126);
  pill->setMaximumHeight(26);
  table->setCellWidget(row, kStatusColumn, pill);
  return pill;
}

inline void polishTable(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table) return;
  table->setShowGrid(false);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setColumnHidden(kTaskColumn, true);
  table->setColumnHidden(kLaunchColumn, true);
  table->verticalHeader()->hide();
  table->verticalHeader()->setDefaultSectionSize(46);
  table->horizontalHeader()->setMinimumHeight(38);
  table->horizontalHeader()->setSectionResizeMode(kSceneColumn, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(kStatusColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kRobotColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kToolColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kUpdatedColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kPinColumn, QHeaderView::Fixed);
  table->setColumnWidth(kStatusColumn, 158);
  table->setColumnWidth(kUpdatedColumn, 132);
  table->setColumnWidth(kPinColumn, 42);

  for (int row = 0; row < table->rowCount(); ++row) {
    const QString raw_status = table->item(row, kStatusColumn)
      ? table->item(row, kStatusColumn)->text()
      : QStringLiteral("BLOCKED");
    const QString status = cleanStatusText(raw_status).toUpper();
    QLabel * pill = ensureStatusPill(table, row);
    if (!pill) continue;
    QString background = QStringLiteral("#FFF4E5");
    QString foreground = QStringLiteral("#A45500");
    QString display = QStringLiteral("NEEDS ATTENTION");
    if (status.contains(QStringLiteral("READY"))) {
      background = QStringLiteral("#E8F6EE");
      foreground = QStringLiteral("#137A46");
      display = QStringLiteral("READY");
    } else if (!(status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION")))) {
      background = QStringLiteral("#FDECEC");
      foreground = QStringLiteral("#B42318");
      display = QStringLiteral("BLOCKED");
    }
    pill->setText(display);
    pill->setStyleSheet(QStringLiteral(
      "QLabel{background:%1;color:%2;border:1px solid %2;border-radius:11px;"
      "font-size:10px;font-weight:800;padding:2px 8px;}").arg(background, foreground));

    if (auto * robot = table->item(row, kRobotColumn)) {
      robot->setForeground(robot->text().compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0
        ? QColor(QStringLiteral("#94A3B8")) : QColor(QStringLiteral("#334155")));
    }
    if (auto * tool = table->item(row, kToolColumn)) {
      tool->setForeground(tool->text().compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0
        ? QColor(QStringLiteral("#94A3B8")) : QColor(QStringLiteral("#334155")));
    }
  }
}

inline void updatePreviewLabel(QMainWindow * window, QLabel * preview, const QString & scene)
{
  if (!preview) return;
  preview->setPixmap(QPixmap());
  const QString root = sceneRoot(window);
  QString preview_path;
  if (!root.isEmpty() && !scene.isEmpty()) {
    const QDir scene_dir(QDir(root).filePath(scene));
    const QStringList candidates = {
      QStringLiteral("preview/workcell_studio_canvas_snapshot.png"),
      QStringLiteral("preview/scene3d_preview.png"),
      QStringLiteral("preview/static_preview.png"),
      QStringLiteral("smoke/scene3d_smoke.png"),
      QStringLiteral("acceptance/scene3d_smoke.png")};
    for (const QString & relative : candidates) {
      const QString candidate = scene_dir.filePath(relative);
      if (QFileInfo::exists(candidate)) {
        preview_path = candidate;
        break;
      }
    }
  }
  const QPixmap pixmap(preview_path);
  if (!pixmap.isNull()) {
    preview->setText(QString());
    preview->setPixmap(pixmap.scaled(330, 182, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    preview->setToolTip(preview_path);
  } else {
    preview->setText(scene.isEmpty()
      ? QStringLiteral("SELECT A SCENE\nPreview and readiness will appear here")
      : QStringLiteral("3D PREVIEW\nOpen Product View for the live scene"));
    preview->setToolTip(QStringLiteral("No cached preview image is available for this scene."));
  }
}

inline void updateInspector(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  auto * scene_label = window->findChild<QLabel *>(QStringLiteral("homeV2InspectorSceneName"));
  auto * status_label = window->findChild<QLabel *>(QStringLiteral("homeV2InspectorStatus"));
  auto * meta_label = window->findChild<QLabel *>(QStringLiteral("homeV2InspectorMeta"));
  auto * readiness = window->findChild<QLabel *>(QStringLiteral("homeV2InspectorReadiness"));
  auto * preview = window->findChild<QLabel *>(QStringLiteral("homeV2InspectorPreview"));
  const int row = table ? table->currentRow() : -1;
  const bool has_scene = table && row >= 0 && row < table->rowCount();
  const QString scene = has_scene ? sceneNameAt(table, row) : QString();

  for (const QString & button_name : {
      QStringLiteral("homeV2OpenButton"), QStringLiteral("homeV2ValidateButton"),
      QStringLiteral("homeV2GenerateButton"), QStringLiteral("homeV2SimulateButton")}) {
    if (auto * button = window->findChild<QPushButton *>(button_name)) button->setEnabled(has_scene);
  }

  if (!has_scene) {
    if (scene_label) scene_label->setText(QStringLiteral("No scene selected"));
    if (status_label) {
      status_label->setText(QStringLiteral("SELECT A SCENE"));
      status_label->setStyleSheet(QStringLiteral("background:#EEF2F7;color:#64748B;border-radius:10px;padding:3px 8px;font-weight:700;"));
    }
    if (meta_label) meta_label->setText(QStringLiteral("Robot  —\nTool  —\nTask  —\nLaunch  —"));
    if (readiness) readiness->setText(QStringLiteral("Select a scene to inspect its readiness and next safe action."));
    updatePreviewLabel(window, preview, QString());
    return;
  }

  const QString status = cleanStatusText(table->item(row, kStatusColumn) ? table->item(row, kStatusColumn)->text() : QStringLiteral("BLOCKED")).toUpper();
  const QString robot = table->item(row, kRobotColumn) ? table->item(row, kRobotColumn)->text() : QStringLiteral("—");
  const QString tool = table->item(row, kToolColumn) ? table->item(row, kToolColumn)->text() : QStringLiteral("—");
  const QString task = table->item(row, kTaskColumn) ? table->item(row, kTaskColumn)->text() : QStringLiteral("—");
  const QString launch = table->item(row, kLaunchColumn) ? table->item(row, kLaunchColumn)->text() : QStringLiteral("—");
  if (scene_label) scene_label->setText(scene);
  if (status_label) {
    QString background = QStringLiteral("#FFF4E5");
    QString foreground = QStringLiteral("#A45500");
    QString display = QStringLiteral("NEEDS ATTENTION");
    if (status.contains(QStringLiteral("READY"))) {
      background = QStringLiteral("#E8F6EE"); foreground = QStringLiteral("#137A46"); display = QStringLiteral("READY");
    } else if (!(status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION")))) {
      background = QStringLiteral("#FDECEC"); foreground = QStringLiteral("#B42318"); display = QStringLiteral("BLOCKED");
    }
    status_label->setText(display);
    status_label->setStyleSheet(QStringLiteral(
      "background:%1;color:%2;border:1px solid %2;border-radius:10px;padding:3px 9px;font-size:10px;font-weight:800;")
      .arg(background, foreground));
  }
  if (meta_label) {
    meta_label->setText(QStringLiteral(
      "<table cellspacing='3'>"
      "<tr><td style='color:#64748B'>Robot</td><td><b>%1</b></td></tr>"
      "<tr><td style='color:#64748B'>Tool</td><td><b>%2</b></td></tr>"
      "<tr><td style='color:#64748B'>Task recipe</td><td><b>%3</b></td></tr>"
      "<tr><td style='color:#64748B'>Fake-hardware launch</td><td><b>%4</b></td></tr>"
      "</table>").arg(robot.toHtmlEscaped(), tool.toHtmlEscaped(), task.toHtmlEscaped(), launch.toHtmlEscaped()));
  }
  if (readiness) {
    readiness->setText(status.contains(QStringLiteral("READY"))
      ? QStringLiteral("Ready for the next fake-hardware workflow step.")
      : QStringLiteral("Review warnings before simulation. Real hardware remains locked."));
  }
  updatePreviewLabel(window, preview, scene);
}

inline void buildInspector(QMainWindow * window)
{
  if (!window || window->findChild<QWidget *>(QStringLiteral("homeV2InspectorContent"))) return;
  QWidget * details_panel = nullptr;
  for (QWidget * candidate : window->findChildren<QWidget *>(QStringLiteral("studioHomeDetailsCard"))) {
    if (candidate && candidate->layout()) {
      details_panel = candidate;
      break;
    }
  }
  if (!details_panel || !details_panel->layout()) return;
  details_panel->setMinimumWidth(322);
  details_panel->setMaximumWidth(356);

  for (QWidget * child : details_panel->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) {
    if (child) child->hide();
  }

  auto * content = new QWidget(details_panel);
  content->setObjectName(QStringLiteral("homeV2InspectorContent"));
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(14, 14, 14, 14);
  layout->setSpacing(10);

  auto * eyebrow = new QLabel(QStringLiteral("SELECTED SCENE"), content);
  eyebrow->setObjectName(QStringLiteral("homeV2InspectorEyebrow"));
  auto * scene_name = new QLabel(QStringLiteral("No scene selected"), content);
  scene_name->setObjectName(QStringLiteral("homeV2InspectorSceneName"));
  scene_name->setWordWrap(true);
  auto * status = new QLabel(QStringLiteral("SELECT A SCENE"), content);
  status->setObjectName(QStringLiteral("homeV2InspectorStatus"));
  status->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
  auto * preview = new QLabel(content);
  preview->setObjectName(QStringLiteral("homeV2InspectorPreview"));
  preview->setAlignment(Qt::AlignCenter);
  preview->setMinimumHeight(182);
  preview->setMaximumHeight(182);
  preview->setScaledContents(false);
  auto * meta = new QLabel(content);
  meta->setObjectName(QStringLiteral("homeV2InspectorMeta"));
  meta->setTextFormat(Qt::RichText);
  meta->setWordWrap(true);
  auto * readiness = new QLabel(content);
  readiness->setObjectName(QStringLiteral("homeV2InspectorReadiness"));
  readiness->setWordWrap(true);

  auto * open = new QPushButton(QStringLiteral("Open in Scene Builder"), content);
  open->setObjectName(QStringLiteral("homeV2OpenButton"));
  auto * secondary_row = new QHBoxLayout();
  secondary_row->setSpacing(8);
  auto * validate = new QPushButton(QStringLiteral("Validate"), content);
  validate->setObjectName(QStringLiteral("homeV2ValidateButton"));
  auto * generate = new QPushButton(QStringLiteral("Generate Package"), content);
  generate->setObjectName(QStringLiteral("homeV2GenerateButton"));
  secondary_row->addWidget(validate);
  secondary_row->addWidget(generate);
  auto * simulate = new QPushButton(QStringLiteral("Simulate · Fake Hardware"), content);
  simulate->setObjectName(QStringLiteral("homeV2SimulateButton"));
  simulate->setToolTip(QStringLiteral("Open the guarded fake-hardware simulation workflow. Real hardware remains locked."));

  layout->addWidget(eyebrow);
  layout->addWidget(scene_name);
  layout->addWidget(status, 0, Qt::AlignLeft);
  layout->addWidget(preview);
  layout->addWidget(meta);
  layout->addWidget(readiness);
  layout->addStretch(1);
  layout->addWidget(open);
  layout->addLayout(secondary_row);
  layout->addWidget(simulate);
  details_panel->layout()->addWidget(content);

  QObject::connect(open, &QPushButton::clicked, window, [window]() {
    triggerAction(window, {QStringLiteral("Open Scene Builder"), QStringLiteral("Open in Scene Builder")});
  });
  QObject::connect(validate, &QPushButton::clicked, window, [window]() {
    triggerAction(window, {QStringLiteral("Validate")});
  });
  QObject::connect(generate, &QPushButton::clicked, window, [window]() {
    triggerAction(window, {QStringLiteral("Generate Scene Package")});
  });
  QObject::connect(simulate, &QPushButton::clicked, window, [window]() {
    triggerAction(window, {QStringLiteral("Open RViz Truth Preview"), QStringLiteral("Plan/Simulate Preview"), QStringLiteral("Plan / Simulate")});
  });
}

inline void unifyFilters(QMainWindow * window)
{
  if (!window || window->findChild<QFrame *>(QStringLiteral("homeV2UnifiedFilterBar"))) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (!table) return;
  QWidget * card = table->parentWidget();
  auto * card_layout = card ? qobject_cast<QVBoxLayout *>(card->layout()) : nullptr;
  if (!card_layout) return;

  auto * bar = new QFrame(card);
  bar->setObjectName(QStringLiteral("homeV2UnifiedFilterBar"));
  auto * layout = new QHBoxLayout(bar);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(8);

  QWidget * widgets[] = {
    window->findChild<QLineEdit *>(QStringLiteral("studioHomeSearchBox")),
    window->findChild<QComboBox *>(QStringLiteral("studioHomeStatusFilter")),
    window->findChild<QComboBox *>(QStringLiteral("studioHomeRobotFilter")),
    window->findChild<QComboBox *>(QStringLiteral("studioHomeToolFilter")),
    window->findChild<QCheckBox *>(QStringLiteral("studioHomeFavoritesOnly")),
    window->findChild<QComboBox *>(QStringLiteral("studioHomeSort"))};

  if (auto * search = qobject_cast<QLineEdit *>(widgets[0])) {
    search->setPlaceholderText(QStringLiteral("Search scenes"));
    search->setMinimumWidth(220);
    search->setMaximumWidth(360);
    search->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  if (auto * status = qobject_cast<QComboBox *>(widgets[1])) {
    status->setMinimumWidth(112); status->setMaximumWidth(128);
  }
  if (auto * robot = qobject_cast<QComboBox *>(widgets[2])) {
    robot->setMinimumWidth(112); robot->setMaximumWidth(128);
  }
  if (auto * tool = qobject_cast<QComboBox *>(widgets[3])) {
    tool->setMinimumWidth(132); tool->setMaximumWidth(156);
  }
  if (auto * favorites = qobject_cast<QCheckBox *>(widgets[4])) favorites->setText(QStringLiteral("Favorites"));
  if (auto * sort = qobject_cast<QComboBox *>(widgets[5])) {
    sort->setMinimumWidth(154); sort->setMaximumWidth(178);
  }

  QLayout * dashboard_layout = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))
    ? window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))->layout()
    : nullptr;
  for (QWidget * widget : widgets) {
    if (!widget) continue;
    if (QLayout * old = findLayoutContaining(dashboard_layout, widget)) old->removeWidget(widget);
    widget->setParent(bar);
    layout->addWidget(widget);
  }
  if (widgets[0]) layout->setStretchFactor(widgets[0], 1);

  if (auto * old = window->findChild<QFrame *>(QStringLiteral("studioHomeFilterStrip"))) old->hide();
  const int table_index = card_layout->indexOf(table);
  card_layout->insertWidget(table_index < 0 ? 1 : table_index, bar);
}

inline void buildKpiRow(QMainWindow * window)
{
  if (!window || window->findChild<QFrame *>(QStringLiteral("homeV2KpiRow"))) return;
  auto * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
  auto * root = dashboard ? qobject_cast<QVBoxLayout *>(dashboard->layout()) : nullptr;
  if (!dashboard || !root) return;

  for (QLabel * legacy : dashboard->findChildren<QLabel *>(QStringLiteral("studioHomeSummaryCard"))) {
    if (legacy) legacy->hide();
  }
  for (QWidget * legacy : dashboard->findChildren<QWidget *>(QStringLiteral("studioHomeDetailsCard"))) {
    if (legacy && !legacy->layout()) legacy->hide();
  }

  auto * row = new QFrame(dashboard);
  row->setObjectName(QStringLiteral("homeV2KpiRow"));
  auto * layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(12);
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV2KpiTotal"), QStringLiteral("TOTAL SCENES"), QStringLiteral("homeV2TotalValue"), QStringLiteral("Across scene library"), QStringLiteral("#1D5DA8")));
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV2KpiReady"), QStringLiteral("READY"), QStringLiteral("homeV2ReadyValue"), QStringLiteral("Ready to simulate"), QStringLiteral("#15803D")));
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV2KpiAttention"), QStringLiteral("NEEDS ATTENTION"), QStringLiteral("homeV2AttentionValue"), QStringLiteral("Warnings to review"), QStringLiteral("#C56A00")));
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV2KpiBlocked"), QStringLiteral("BLOCKED"), QStringLiteral("homeV2BlockedValue"), QStringLiteral("Cannot run yet"), QStringLiteral("#C53232")));
  for (int i = 0; i < 4; ++i) layout->setStretch(i, 1);

  QWidget * hero = window->findChild<QWidget *>(QStringLiteral("studioHomeHeroCard"));
  const int hero_index = hero ? root->indexOf(hero) : -1;
  root->insertWidget(hero_index >= 0 ? hero_index + 1 : 0, row);
}

inline void polishShell(QMainWindow * window)
{
  if (!window) return;
  if (auto * top_bar = window->findChild<QToolBar *>(QStringLiteral("studioTopBar"))) {
    top_bar->setMinimumHeight(48);
    top_bar->setMaximumHeight(48);
  }
  if (auto * brand = window->findChild<QLabel *>(QStringLiteral("studioTopBrand"))) brand->hide();
  if (auto * status = window->findChild<QLabel *>(QStringLiteral("studioTopStatusChip"))) status->setText(QStringLiteral("●  Ready"));

  if (auto * sidebar = window->findChild<QFrame *>(QStringLiteral("studioHomeSidebar"))) {
    sidebar->setMinimumWidth(190);
    sidebar->setMaximumWidth(198);
  }
  if (auto * nav = window->findChild<QListWidget *>(QStringLiteral("studioHomeSidebarNav"))) {
    nav->setFrameShape(QFrame::NoFrame);
    nav->setAutoFillBackground(false);
    if (nav->viewport()) {
      nav->viewport()->setAutoFillBackground(false);
      nav->viewport()->setStyleSheet(QStringLiteral("background:#082A4A;border:0;"));
    }
    nav->setStyleSheet(QStringLiteral(
      "QListWidget{background:#082A4A;border:0;color:#DCEAF5;outline:0;}"
      "QListWidget::item{background:transparent;color:#DCEAF5;border-radius:6px;padding:8px 10px;margin:1px 0;}"
      "QListWidget::item:hover{background:#103C64;color:white;}"
      "QListWidget::item:selected{background:#14558E;color:white;font-weight:700;}"));
    for (int i = 0; i < nav->count(); ++i) nav->item(i)->setSizeHint(QSize(0, 36));
  }

  if (auto * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
    if (auto * root = qobject_cast<QVBoxLayout *>(dashboard->layout())) {
      root->setContentsMargins(24, 16, 24, 14);
      root->setSpacing(12);
    }
  }
  if (auto * hero = window->findChild<QWidget *>(QStringLiteral("studioHomeHeroCard"))) {
    hero->setMaximumHeight(82);
    hero->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  }
  if (auto * title = window->findChild<QLabel *>(QStringLiteral("dashboardTitleLabel"))) title->setText(QStringLiteral("Workcell Studio"));
  if (auto * subtitle = window->findChild<QLabel *>(QStringLiteral("dashboardSubtitleLabel"))) {
    subtitle->setText(QStringLiteral("Configure, validate and simulate robotic workcells."));
  }
  for (QPushButton * button : window->findChildren<QPushButton *>(QStringLiteral("studioHomePrimaryButton"))) {
    if (button) button->hide();
  }

  if (QStatusBar * bar = window->statusBar()) {
    QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
    auto * stack = dashboard ? qobject_cast<QStackedWidget *>(dashboard->parentWidget()) : nullptr;
    if (stack) {
      bar->setVisible(stack->currentWidget() != dashboard);
      if (!stack->property("homeV2StatusBarConnected").toBool()) {
        stack->setProperty("homeV2StatusBarConnected", true);
        QObject::connect(stack, &QStackedWidget::currentChanged, bar, [bar, stack, dashboard](int) {
          bar->setVisible(stack->currentWidget() != dashboard);
        });
      }
    }
  }
}

inline void appendPolishStyle(QMainWindow * window)
{
  if (!window || window->property("homeV2PolishStyleApplied").toBool()) return;
  window->setProperty("homeV2PolishStyleApplied", true);
  window->setStyleSheet(window->styleSheet() + QStringLiteral(R"QSS(
QWidget#workcellStudioDashboardPage { background:#F5F7FA; }
QFrame#homeV2KpiTotal, QFrame#homeV2KpiReady, QFrame#homeV2KpiAttention, QFrame#homeV2KpiBlocked {
  background:#FFFFFF; border:1px solid #DCE4EC; border-radius:9px;
}
QLabel#homeV2KpiTitle { color:#64748B; font-size:10px; font-weight:800; letter-spacing:0.5px; border:0; background:transparent; }
QLabel#homeV2KpiDetail { color:#94A3B8; font-size:10px; border:0; background:transparent; }
QFrame#homeV2UnifiedFilterBar { background:transparent; border:0; }
QFrame#studioHomeScenesCard, QWidget#studioHomeDetailsCard { background:#FFFFFF; border:1px solid #DCE4EC; border-radius:9px; }
QLabel#homeV2InspectorEyebrow { color:#64748B; font-size:10px; font-weight:800; letter-spacing:0.6px; }
QLabel#homeV2InspectorSceneName { color:#102B46; font-size:17px; font-weight:800; }
QLabel#homeV2InspectorPreview {
  background:#18232E; color:#C7D3DF; border:1px solid #243747; border-radius:8px;
  font-size:11px; font-weight:700;
}
QLabel#homeV2InspectorMeta { color:#27384A; background:transparent; border:0; font-size:11px; }
QLabel#homeV2InspectorReadiness {
  background:#F7F9FC; color:#465A70; border:1px solid #E1E8EF; border-radius:7px; padding:9px; font-size:11px;
}
QPushButton#homeV2OpenButton {
  background:#0E4F86; color:#FFFFFF; border:1px solid #0A416F; border-radius:6px; min-height:34px; font-weight:800;
}
QPushButton#homeV2OpenButton:hover { background:#12629F; }
QPushButton#homeV2ValidateButton, QPushButton#homeV2GenerateButton, QPushButton#homeV2SimulateButton {
  background:#FFFFFF; color:#174B79; border:1px solid #C7D5E2; border-radius:6px; min-height:32px; font-weight:700;
}
QPushButton#homeV2SimulateButton { color:#137A46; border-color:#AFCFBC; }
QTableWidget#studioHomeSceneTable { border:1px solid #DCE4EC; border-radius:7px; background:#FFFFFF; outline:0; }
QTableWidget#studioHomeSceneTable::item { border-bottom:1px solid #EEF2F6; padding:6px 9px; }
QTableWidget#studioHomeSceneTable::item:selected { background:#EAF2FA; color:#163A5C; }
QTableWidget#studioHomeSceneTable QHeaderView::section { background:#F8FAFC; border:0; border-bottom:1px solid #DCE4EC; padding:8px; color:#465A70; font-weight:800; }
QToolBar#studioTopBar { padding:5px 12px; border-bottom:1px solid #DDE5EE; background:#FFFFFF; }
)QSS"));
}

inline void applyPolish(QMainWindow * window)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (!window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) return;
  if (!window->property("homeV2ScreenshotPolishApplied").toBool()) {
    window->setProperty("homeV2ScreenshotPolishApplied", true);
    appendPolishStyle(window);
    polishShell(window);
    buildKpiRow(window);
    unifyFilters(window);
    buildInspector(window);

    if (auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"))) {
      QObject::connect(table, &QTableWidget::itemSelectionChanged, window, [window]() { updateInspector(window); });
      QObject::connect(table, &QTableWidget::cellClicked, window, [window](int, int) { updateInspector(window); });
      if (table->model()) {
        QObject::connect(table->model(), &QAbstractItemModel::rowsInserted, window, [window]() {
          QTimer::singleShot(0, window, [window]() { polishTable(window); updateKpis(window); updateInspector(window); });
        });
        QObject::connect(table->model(), &QAbstractItemModel::modelReset, window, [window]() {
          QTimer::singleShot(0, window, [window]() { polishTable(window); updateKpis(window); updateInspector(window); });
        });
        QObject::connect(table->model(), &QAbstractItemModel::dataChanged, window, [window]() {
          QTimer::singleShot(0, window, [window]() { polishTable(window); updateKpis(window); updateInspector(window); });
        });
      }
    }
  }
  polishShell(window);
  polishTable(window);
  updateKpis(window);
  updateInspector(window);
}

class HomePolishGuard : public QObject
{
public:
  explicit HomePolishGuard(QObject * parent) : QObject(parent) {}

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    auto * window = qobject_cast<QMainWindow *>(watched);
    if (!window || !window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
      return QObject::eventFilter(watched, event);
    }
    if (event->type() == QEvent::Show) {
      QTimer::singleShot(180, window, [window]() { applyPolish(window); });
      QTimer::singleShot(700, window, [window]() { applyPolish(window); });
    } else if (event->type() == QEvent::Resize) {
      QTimer::singleShot(0, window, [window]() { polishShell(window); });
    }
    return QObject::eventFilter(watched, event);
  }
};

inline void install()
{
  if (!qApp || qApp->property("workcellHomePolishV2Installed").toBool()) return;
  qApp->setProperty("workcellHomePolishV2Installed", true);
  qApp->installEventFilter(new HomePolishGuard(qApp));
}
}  // namespace home_polish_v2
}  // namespace workcell_builder

inline void workcellHomePolishV2Startup()
{
  workcell_builder::home_polish_v2::install();
}

Q_COREAPP_STARTUP_FUNCTION(workcellHomePolishV2Startup)
