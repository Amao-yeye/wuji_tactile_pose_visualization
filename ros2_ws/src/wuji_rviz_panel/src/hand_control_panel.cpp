#include "wuji_rviz_panel/hand_control_panel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>

#include <QColor>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QRect>
#include <QShowEvent>
#include <QSizePolicy>
#include <QThread>
#include <QTimer>
#include <QToolTip>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>

namespace wuji_rviz_panel
{
namespace
{
constexpr double kBaselineCaptureSeconds = 2.0;
constexpr size_t kMinimumBaselineSamples = 5;
constexpr int kStatusTimeoutMs = 1200;
constexpr int kTactileRenderPeriodMs = 20;
constexpr int kRequiredStableLayoutSamples = 2;
constexpr int kMaximumLayoutStabilizationAttempts = 50;
constexpr size_t kPendingTactileCapacity = 32;
constexpr int kDiagnosticsPeriodMs = 5000;
constexpr int kHeartbeatPeriodMs = 200;
constexpr uint32_t kTactileSequenceModulus = 1U << 16;
constexpr double kRawDefaultMaximum = 1.0;
constexpr double kBaselineDefaultMaximum = 0.25;
constexpr double kBaselineDefaultThreshold = 0.10;
constexpr double kTemporalDefaultMaximum = 0.05;
constexpr auto kInvalidColor = "#3f3f46";
constexpr auto kInactiveColor = "#111827";

struct PoseUiEntry
{
  const char * display_name;
  const char * pose_id;
};

constexpr std::array<PoseUiEntry, 11> kPoseUiEntries{{
  {"Relaxed", "relaxed"},
  {"Cup Grasp", "cup_grasp"},
  {"Four Finger 90", "four_finger_90"},
  {"Fist", "fist"},
  {"Thumb–Index Touch", "thumb_index_touch"},
  {"Thumb–Middle Touch", "thumb_middle_touch"},
  {"Thumb–Ring Touch", "thumb_ring_touch"},
  {"Thumb–Pinky Touch", "thumb_pinky_touch"},
  {"Tripod", "tripod"},
  {"Index Point", "index_point"},
  {"Book Flick Ready", "book_flick_ready"},
}};

bool finite(double value)
{
  return std::isfinite(value);
}

uint64_t currentThreadId()
{
  return static_cast<uint64_t>(
    std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

QString formatCell(uint32_t index, uint32_t cols)
{
  return QString("(%1, %2)").arg(index / cols).arg(index % cols);
}

QGroupBox * heatmapGroup(const QString & title, HeatmapWidget * widget, QLabel * stats)
{
  auto * group = new QGroupBox(title);
  auto * layout = new QVBoxLayout(group);
  layout->addWidget(widget, 1);
  stats->setWordWrap(true);
  stats->setMinimumHeight(42);
  stats->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(stats);
  return group;
}

QDockWidget * ancestorDockWidget(QWidget * widget)
{
  for (QWidget * ancestor = widget != nullptr ? widget->parentWidget() : nullptr;
    ancestor != nullptr; ancestor = ancestor->parentWidget())
  {
    if (auto * dock = qobject_cast<QDockWidget *>(ancestor)) {
      return dock;
    }
  }
  return nullptr;
}

QString widgetGeometry(const QWidget * widget)
{
  if (widget == nullptr) {
    return "missing";
  }
  const QRect geometry = widget->geometry();
  return QString("%1,%2 %3x%4 visible=%5")
         .arg(geometry.x()).arg(geometry.y()).arg(geometry.width()).arg(geometry.height())
         .arg(widget->isVisible() ? "yes" : "no");
}

bool hasUsableGeometry(const QWidget * widget)
{
  return widget != nullptr && widget->isVisible() && widget->width() > 0 && widget->height() > 0;
}
}  // namespace

HeatmapWidget::HeatmapWidget(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void HeatmapWidget::setGrid(
  uint32_t rows, uint32_t cols, const std::vector<double> & values,
  const std::vector<uint8_t> & active, double maximum, const QString & overlay)
{
  rows_ = rows;
  cols_ = cols;
  values_ = values;
  overlay_ = overlay;

  const size_t expected_values = static_cast<size_t>(rows_) * cols_;
  if (rows_ == 0 || cols_ == 0 || values_.size() != expected_values) {
    heatmap_image_ = QImage();
    update();
    return;
  }

  const QSize image_size(static_cast<int>(cols_), static_cast<int>(rows_));
  if (heatmap_image_.size() != image_size || heatmap_image_.format() != QImage::Format_RGB32) {
    heatmap_image_ = QImage(image_size, QImage::Format_RGB32);
  }

  const double fixed_maximum = std::max(maximum, 1.0e-9);
  const QRgb invalid_color = QColor(kInvalidColor).rgb();
  const QRgb inactive_color = QColor(kInactiveColor).rgb();
  for (uint32_t row = 0; row < rows_; ++row) {
    auto * pixels = reinterpret_cast<QRgb *>(heatmap_image_.scanLine(static_cast<int>(row)));
    for (uint32_t col = 0; col < cols_; ++col) {
      const size_t index = static_cast<size_t>(row) * cols_ + col;
      QRgb color = invalid_color;
      if (finite(values_[index])) {
        const bool is_active =
          active.empty() || (index < active.size() && active[index] != 0);
        color = is_active ? infernoColor(values_[index], fixed_maximum).rgb() : inactive_color;
      }
      pixels[col] = color;
    }
  }
  update();
}

QSize HeatmapWidget::minimumSizeHint() const
{
  return QSize(250, 260);
}

QColor HeatmapWidget::infernoColor(double value, double maximum)
{
  static const std::vector<std::pair<double, QColor>> stops = {
    {0.00, QColor("#000004")},
    {0.18, QColor("#320a5e")},
    {0.36, QColor("#781c6d")},
    {0.54, QColor("#bb3654")},
    {0.72, QColor("#ed6925")},
    {0.88, QColor("#fbb61a")},
    {1.00, QColor("#fcffa4")},
  };
  const double ratio = std::clamp(value / maximum, 0.0, 1.0);
  for (size_t index = 1; index < stops.size(); ++index) {
    if (ratio <= stops[index].first) {
      const auto & lower = stops[index - 1];
      const auto & upper = stops[index];
      const double span = upper.first - lower.first;
      const double blend = span > 0.0 ? (ratio - lower.first) / span : 0.0;
      return QColor(
        static_cast<int>(std::lround(lower.second.red() + blend * (upper.second.red() - lower.second.red()))),
        static_cast<int>(std::lround(lower.second.green() + blend * (upper.second.green() - lower.second.green()))),
        static_cast<int>(std::lround(lower.second.blue() + blend * (upper.second.blue() - lower.second.blue()))));
    }
  }
  return stops.back().second;
}

void HeatmapWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.fillRect(rect(), QColor("#101010"));
  if (rows_ == 0 || cols_ == 0 || heatmap_image_.isNull()) {
    painter.setPen(QColor("#cbd5e1"));
    painter.drawText(rect(), Qt::AlignCenter, "Waiting for tactile frames...");
    return;
  }

  constexpr double left = 28.0;
  constexpr double top = 22.0;
  constexpr double right = 6.0;
  constexpr double bottom = 6.0;
  const double cell_width = std::max(1.0, (width() - left - right) / cols_);
  const double cell_height = std::max(1.0, (height() - top - bottom) / rows_);
  const QRectF image_bounds(
    left, top, std::max(1.0, width() - left - right),
    std::max(1.0, height() - top - bottom));
  const QRectF image_source(
    0.0, 0.0, static_cast<double>(heatmap_image_.width()),
    static_cast<double>(heatmap_image_.height()));
  painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
  painter.drawImage(image_bounds, heatmap_image_, image_source);

  painter.setPen(QColor("#cbd5e1"));
  QFont axis_font = painter.font();
  axis_font.setPointSize(7);
  painter.setFont(axis_font);
  for (uint32_t col = 0; col < cols_; ++col) {
    if (col % 5 == 0 || col + 1 == cols_) {
      const QRectF label(left + col * cell_width, 0.0, cell_width * 2.0, top);
      painter.drawText(label, Qt::AlignLeft | Qt::AlignVCenter, QString::number(col));
    }
  }
  for (uint32_t row = 0; row < rows_; ++row) {
    if (row % 4 == 0 || row + 1 == rows_) {
      const QRectF label(0.0, top + row * cell_height, left - 3.0, cell_height);
      painter.drawText(label, Qt::AlignRight | Qt::AlignVCenter, QString::number(row));
    }
  }

  if (!overlay_.isEmpty()) {
    QRectF banner = rect().adjusted(35, height() / 2 - 30, -10, -(height() / 2 - 30));
    painter.fillRect(banner, QColor(15, 23, 42, 220));
    painter.setPen(QColor("#e2e8f0"));
    QFont overlay_font = painter.font();
    overlay_font.setBold(true);
    overlay_font.setPointSize(9);
    painter.setFont(overlay_font);
    painter.drawText(banner, Qt::AlignCenter | Qt::TextWordWrap, overlay_);
  }
}

void HeatmapWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (rows_ == 0 || cols_ == 0 || values_.empty()) {
    return;
  }
  constexpr double left = 28.0;
  constexpr double top = 22.0;
  constexpr double right = 6.0;
  constexpr double bottom = 6.0;
  const double cell_width = std::max(1.0, (width() - left - right) / cols_);
  const double cell_height = std::max(1.0, (height() - top - bottom) / rows_);
  const int col = static_cast<int>((event->localPos().x() - left) / cell_width);
  const int row = static_cast<int>((event->localPos().y() - top) / cell_height);
  if (row < 0 || col < 0 || row >= static_cast<int>(rows_) || col >= static_cast<int>(cols_)) {
    QToolTip::hideText();
    return;
  }
  const size_t index = static_cast<size_t>(row) * cols_ + static_cast<size_t>(col);
  const double value = values_[index];
  const QString value_text = finite(value) ? QString::number(value, 'f', 6) : "NaN / invalid";
  QToolTip::showText(event->globalPos(), QString("row %1, col %2: %3").arg(row).arg(col).arg(value_text), this);
}

HandControlPanel::HandControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setMinimumWidth(850);
  buildUi();
  connectUi();
  refreshControlButtons();
}

HandControlPanel::~HandControlPanel()
{
  if (tactile_render_timer_) {
    tactile_render_timer_->stop();
  }
  if (tactile_executor_) {
    tactile_executor_->cancel();
  }
  if (tactile_executor_thread_.joinable()) {
    tactile_executor_thread_.join();
  }
  tactile_executor_.reset();
  tactile_subscription_.reset();
  tactile_callback_group_.reset();
  if (heartbeat_timer_ != nullptr) {
    heartbeat_timer_->stop();
  }
  requestDisableBestEffort();
}

void HandControlPanel::buildUi()
{
  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(6, 6, 6, 6);

  auto * title = new QLabel("Wuji Hand — control + tactile visualization");
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 1);
  title->setFont(title_font);
  root_layout->addWidget(title);

  connection_label_ = new QLabel("Control state: waiting for backend status; motion controls locked.");
  connection_label_->setWordWrap(true);
  connection_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  root_layout->addWidget(connection_label_);
  layout_label_ = new QLabel("Waiting for /hand_0/tactile/pressure ...");
  layout_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  root_layout->addWidget(layout_label_);

  auto * controls = new QGroupBox("Manual motion");
  auto * control_layout = new QVBoxLayout(controls);

  enable_button_ = new QPushButton("Enable");
  pose_selector_ = new QComboBox();
  pose_selector_->setObjectName("pose_selector");
  pose_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  for (const auto & pose : kPoseUiEntries) {
    pose_selector_->addItem(
      QString::fromUtf8(pose.display_name), QString::fromLatin1(pose.pose_id));
  }
  selected_pose_id_ = pose_selector_->currentData().toString();
  move_pose_button_ = new QPushButton("Move to Pose");
  move_pose_button_->setObjectName("move_to_pose_button");
  open_button_ = new QPushButton("Open");
  open_button_->setObjectName("open_pose_button");
  disable_button_ = new QPushButton("Disable");
  disable_button_->setStyleSheet("QPushButton { font-weight: bold; color: #b91c1c; }");

  auto * pose_row = new QHBoxLayout();
  pose_row->addWidget(enable_button_);
  pose_row->addSpacing(12);
  pose_row->addWidget(new QLabel("Pose:"));
  pose_row->addWidget(pose_selector_, 1);
  pose_row->addWidget(move_pose_button_);
  pose_row->addWidget(open_button_);
  pose_row->addStretch(1);
  pose_row->addWidget(disable_button_);
  control_layout->addLayout(pose_row);

  request_result_label_ = new QLabel(
    "Status: no action requested. Backend remains authoritative for every safety gate.");
  request_result_label_->setWordWrap(true);
  request_result_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  control_layout->addWidget(request_result_label_);
  root_layout->addWidget(controls);

  auto * settings = new QGroupBox("Fixed display scales and captured baseline");
  auto * settings_layout = new QGridLayout(settings);
  raw_vmax_ = new QDoubleSpinBox();
  baseline_vmax_ = new QDoubleSpinBox();
  baseline_threshold_ = new QDoubleSpinBox();
  baseline_offset_ = new QDoubleSpinBox();
  temporal_vmax_ = new QDoubleSpinBox();
  for (auto * spin : {raw_vmax_, baseline_vmax_, baseline_threshold_, baseline_offset_, temporal_vmax_}) {
    spin->setDecimals(3);
    spin->setRange(0.0, 10.0);
    spin->setSingleStep(0.01);
  }
  raw_vmax_->setMinimum(0.001);
  raw_vmax_->setValue(kRawDefaultMaximum);
  baseline_vmax_->setMinimum(0.05);
  baseline_vmax_->setValue(kBaselineDefaultMaximum);
  baseline_threshold_->setValue(kBaselineDefaultThreshold);
  baseline_offset_->setValue(0.0);
  temporal_vmax_->setMinimum(0.005);
  temporal_vmax_->setValue(kTemporalDefaultMaximum);

  settings_layout->addWidget(new QLabel("Raw vmax"), 0, 0);
  settings_layout->addWidget(raw_vmax_, 0, 1);
  settings_layout->addWidget(new QLabel("Baseline vmax"), 0, 2);
  settings_layout->addWidget(baseline_vmax_, 0, 3);
  settings_layout->addWidget(new QLabel("Baseline threshold (display only)"), 0, 4);
  settings_layout->addWidget(baseline_threshold_, 0, 5);
  settings_layout->addWidget(new QLabel("Baseline offset (display only)"), 1, 0);
  settings_layout->addWidget(baseline_offset_, 1, 1);
  settings_layout->addWidget(new QLabel("Temporal vmax"), 1, 2);
  settings_layout->addWidget(temporal_vmax_, 1, 3);
  capture_baseline_button_ = new QPushButton("Capture Baseline (2 s median)");
  reset_baseline_button_ = new QPushButton("Reset Baseline");
  settings_layout->addWidget(capture_baseline_button_, 1, 4);
  settings_layout->addWidget(reset_baseline_button_, 1, 5);
  baseline_state_label_ = new QLabel("Baseline not captured; residual data is intentionally uninitialized.");
  baseline_state_label_->setWordWrap(true);
  baseline_state_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  settings_layout->addWidget(baseline_state_label_, 2, 0, 1, 6);
  root_layout->addWidget(settings);

  raw_heatmap_ = new HeatmapWidget();
  baseline_heatmap_ = new HeatmapWidget();
  temporal_heatmap_ = new HeatmapWidget();
  raw_stats_label_ = new QLabel("No raw data.");
  baseline_stats_label_ = new QLabel("Baseline not captured.");
  temporal_stats_label_ = new QLabel("Waiting for two compatible high-rate frames.");
  auto * heatmaps = new QHBoxLayout();
  heatmaps->addWidget(heatmapGroup("Raw Pressure", raw_heatmap_, raw_stats_label_), 1);
  heatmaps->addWidget(heatmapGroup("Baseline Residual", baseline_heatmap_, baseline_stats_label_), 1);
  heatmaps->addWidget(heatmapGroup("Temporal Peak |Delta|", temporal_heatmap_, temporal_stats_label_), 1);
  root_layout->addLayout(heatmaps, 1);
}

void HandControlPanel::connectUi()
{
  connect(capture_baseline_button_, &QPushButton::clicked, this, &HandControlPanel::startBaselineCapture);
  connect(reset_baseline_button_, &QPushButton::clicked, this, &HandControlPanel::resetBaseline);
  for (auto * spin : {raw_vmax_, baseline_vmax_, baseline_threshold_, baseline_offset_, temporal_vmax_}) {
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {renderLatest();});
  }
  connect(
    pose_selector_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, &HandControlPanel::onPoseSelectionChanged);
  connect(
    move_pose_button_, &QPushButton::clicked,
    this, &HandControlPanel::requestSelectedPose);
  connect(open_button_, &QPushButton::clicked, this, [this]() {
    requestPose("open", "Open");
  });

  connect(enable_button_, &QPushButton::clicked, this, [this]() {
    if (QMessageBox::warning(
        this, "Enable Wuji Hand",
        "Enable captures the fresh real pose as the existing Cup Grasp reference "
        "and enables motors. It sends no target by itself. Continue?",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
    {
      sendAction(ControlCommand::Request::ENABLE, "Enable");
    }
  });
  connect(disable_button_, &QPushButton::clicked, this, [this]() {
    sendAction(ControlCommand::Request::DISABLE, "Disable");
  });
}

void HandControlPanel::onPoseSelectionChanged(int index)
{
  if (index < 0 || index >= pose_selector_->count()) {
    selected_pose_id_.clear();
    request_result_label_->setText("Selected pose is unavailable; no command was sent.");
    return;
  }
  selected_pose_id_ = pose_selector_->itemData(index).toString();
  request_result_label_->setText(
    QString("Selected pose: %1. Selection alone sends no command.")
    .arg(selected_pose_id_));
}

void HandControlPanel::requestSelectedPose()
{
  if (selected_pose_id_.isEmpty()) {
    request_result_label_->setText("Selected pose is unavailable. No command was sent.");
    return;
  }
  requestPose(selected_pose_id_, pose_selector_->currentText());
}

void HandControlPanel::requestPose(
  const QString & pose_id, const QString & display_name)
{
  if (pose_id.isEmpty()) {
    request_result_label_->setText("Requested pose is unavailable. No command was sent.");
    return;
  }
  const QString target_source =
    pose_id == "cup_grasp" ?
    QString(
      "The backend will reuse the existing verified final Cup Grasp target; "
      "no qpos is regenerated or modified.") :
    QString("The backend will load the exact 20D target from the read-only JSON library.");
  const QString warning = QString(
    "Move the physical hand to %1 (%2)?\n\n"
    "%3\n\n"
    "Motion starts from fresh measured joint feedback and uses the existing "
    "bounded safe-motion pipeline. Runtime firmware limits with the 0.08 rad margin "
    "must pass; the target will be rejected rather than clipped.\n\n"
    "Observe one motion at a time.")
    .arg(display_name)
    .arg(pose_id)
    .arg(target_source);
  if (QMessageBox::warning(
      this, "Move to Pose", warning,
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
  {
    sendAction(
      ControlCommand::Request::MOVE_POSE,
      QString("Move to Pose (%1)").arg(display_name),
      pose_id);
  }
}

void HandControlPanel::onInitialize()
{
  node_abstraction_ = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction_) {
    connection_label_->setText("RViz ROS node is unavailable; panel initialization failed.");
    return;
  }
  node_ = node_abstraction_->get_raw_node();
  if (!node_->has_parameter("wuji_hand_name")) {
    node_->declare_parameter<std::string>("wuji_hand_name", "hand_0");
  }
  hand_name_ = QString::fromStdString(node_->get_parameter("wuji_hand_name").as_string());
  hand_name_.remove('/');
  if (hand_name_.isEmpty()) {
    hand_name_ = "hand_0";
  }
  const std::string prefix = "/" + hand_name_.toStdString();
  layout_label_->setText(QString("Waiting for %1/tactile/pressure ...").arg(QString::fromStdString(prefix)));
  RCLCPP_INFO(
    node_->get_logger(),
    "[POSE_UI] initialized %d explicit display-name/pose-id entries; "
    "execution=backend_named_pose_id_only",
    pose_selector_->count());

  tactile_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions tactile_options;
  tactile_options.callback_group = tactile_callback_group_;
  tactile_subscription_ = node_->create_subscription<TactileFrame>(
    prefix + "/tactile/pressure", rclcpp::SensorDataQoS(),
    std::bind(&HandControlPanel::onTactileFrame, this, std::placeholders::_1), tactile_options);
  tactile_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  tactile_executor_->add_callback_group(
    tactile_callback_group_, node_->get_node_base_interface());
  tactile_executor_thread_ = std::thread([this]() {
      try {
        tactile_executor_->spin();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(node_->get_logger(), "Tactile executor stopped unexpectedly: %s", error.what());
      }
    });
  status_subscription_ = node_->create_subscription<ControlStatus>(
    prefix + "/hand_control/status", rclcpp::QoS(10),
    std::bind(&HandControlPanel::onControlStatus, this, std::placeholders::_1));
  command_client_ = node_->create_client<ControlCommand>(prefix + "/hand_control/command");
  heartbeat_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
    prefix + "/hand_control/ui_heartbeat", rclcpp::QoS(10));

  diagnostics_age_.start();
  tactile_render_timer_ = new QTimer(this);
  tactile_render_timer_->setTimerType(Qt::PreciseTimer);
  connect(tactile_render_timer_, &QTimer::timeout, this, &HandControlPanel::processLatestTactileFrame);

  heartbeat_timer_ = new QTimer(this);
  connect(heartbeat_timer_, &QTimer::timeout, this, &HandControlPanel::publishHeartbeat);
  heartbeat_timer_->start(kHeartbeatPeriodMs);
  publishHeartbeat();

  status_watch_timer_ = new QTimer(this);
  connect(status_watch_timer_, &QTimer::timeout, this, &HandControlPanel::statusWatchTick);
  status_watch_timer_->start(250);
  connection_label_->setText("RViz panel connected to ROS; waiting for the single-SDK backend status.");

  if (isVisible()) {
    panel_shown_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[STARTUP_LAYOUT_STABILIZATION] panel already visible when onInitialize completed; panel=%s",
      widgetGeometry(this).toUtf8().constData());
    scheduleRvizLayoutCheck();
  }
}

void HandControlPanel::showEvent(QShowEvent * event)
{
  rviz_common::Panel::showEvent(event);
  const bool first_show = !panel_shown_;
  panel_shown_ = true;
  if (first_show && node_ != nullptr) {
    RCLCPP_INFO(
      node_->get_logger(), "[STARTUP_LAYOUT_STABILIZATION] panel shown; panel=%s",
      widgetGeometry(this).toUtf8().constData());
  }
  scheduleRvizLayoutCheck();
}

void HandControlPanel::scheduleRvizLayoutCheck()
{
  if (!panel_shown_ || node_ == nullptr || tactile_render_timer_ == nullptr ||
    layout_stabilized_ || layout_check_scheduled_)
  {
    return;
  }
  layout_check_scheduled_ = true;
  QTimer::singleShot(0, this, &HandControlPanel::waitForStableRvizLayout);
}

void HandControlPanel::waitForStableRvizLayout()
{
  layout_check_scheduled_ = false;
  if (layout_stabilized_ || tactile_render_timer_ == nullptr) {
    return;
  }
  if (QThread::currentThread() != thread()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[STARTUP_LAYOUT_STABILIZATION] layout check refused outside the Qt GUI thread");
    return;
  }

  QDockWidget * dock = ancestorDockWidget(this);
  QMainWindow * main_window = qobject_cast<QMainWindow *>(window());
  rviz_common::RenderPanel * render_panel = nullptr;
  auto * display_context = getDisplayContext();
  if (display_context != nullptr && display_context->getViewManager() != nullptr) {
    render_panel = display_context->getViewManager()->getRenderPanel();
  }

  ++layout_stabilization_attempts_;
  const bool geometry_is_usable =
    hasUsableGeometry(this) && hasUsableGeometry(dock) &&
    hasUsableGeometry(main_window) && hasUsableGeometry(render_panel);
  const QString geometry =
    QString("panel={%1} dock={%2} main={%3} render={%4} floating=%5")
    .arg(widgetGeometry(this))
    .arg(widgetGeometry(dock))
    .arg(widgetGeometry(main_window))
    .arg(widgetGeometry(render_panel))
    .arg(dock != nullptr && dock->isFloating() ? "yes" : "no");

  if (geometry_is_usable && geometry == previous_layout_geometry_) {
    ++layout_stable_sample_count_;
  } else if (geometry_is_usable) {
    layout_stable_sample_count_ = 1;
  } else {
    layout_stable_sample_count_ = 0;
  }
  previous_layout_geometry_ = geometry;

  if (layout_stable_sample_count_ >= kRequiredStableLayoutSamples) {
    layout_stabilized_ = true;
    if (display_context != nullptr) {
      display_context->queueRender();
    }
    tactile_render_timer_->start(kTactileRenderPeriodMs);
    RCLCPP_INFO(
      node_->get_logger(),
      "[STARTUP_LAYOUT_STABILIZATION] layout stable after %d queued checks; %s; qt_tid=%llu",
      layout_stabilization_attempts_, geometry.toUtf8().constData(),
      static_cast<unsigned long long>(currentThreadId()));
    RCLCPP_INFO(
      node_->get_logger(),
      "[STARTUP_LAYOUT_STABILIZATION] tactile render enabled at %d ms (50 Hz)",
      kTactileRenderPeriodMs);
    return;
  }

  if (layout_stabilization_attempts_ >= kMaximumLayoutStabilizationAttempts) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[STARTUP_LAYOUT_STABILIZATION] layout did not stabilize after %d queued checks; "
      "tactile rendering remains stopped; last %s",
      layout_stabilization_attempts_, geometry.toUtf8().constData());
    return;
  }
  scheduleRvizLayoutCheck();
}

void HandControlPanel::onTactileFrame(const TactileFrame::SharedPtr message)
{
  tactile_callback_thread_id_.store(currentThreadId(), std::memory_order_relaxed);
  const uint32_t sequence = message->sequence;
  if (have_backend_sequence_) {
    const uint32_t advance = sequence >= backend_last_sequence_ ?
      sequence - backend_last_sequence_ :
      sequence + kTactileSequenceModulus - backend_last_sequence_;
    if (advance < kTactileSequenceModulus / 2U) {
      backend_sequence_advance_count_.fetch_add(advance, std::memory_order_relaxed);
    }
  }
  backend_last_sequence_ = sequence;
  have_backend_sequence_ = true;
  tactile_callback_count_.fetch_add(1, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(pending_tactile_mutex_);
  if (pending_tactile_frames_.size() >= kPendingTactileCapacity) {
    pending_tactile_frames_.pop_front();
    tactile_buffer_drop_count_.fetch_add(1, std::memory_order_relaxed);
  }
  pending_tactile_frames_.push_back(message);
}

void HandControlPanel::processLatestTactileFrame()
{
  if (QThread::currentThread() != thread()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Tactile GUI timer ran outside the Qt GUI thread; frame ignored.");
    return;
  }
  ++gui_timer_tick_count_;
  gui_timer_thread_id_ = currentThreadId();

  std::deque<TactileFrame::SharedPtr> pending;
  {
    std::lock_guard<std::mutex> lock(pending_tactile_mutex_);
    pending.swap(pending_tactile_frames_);
  }
  if (!pending.empty()) {
    gui_max_batch_frame_count_ = std::max<uint64_t>(
      gui_max_batch_frame_count_, static_cast<uint64_t>(pending.size()));
    const size_t applied = applyTactileFrames(pending);
    if (applied > 0) {
      processed_tactile_frame_count_ += applied;
      ++gui_refresh_count_;
    }
  }

  const qint64 elapsed_ms = diagnostics_age_.elapsed();
  if (elapsed_ms >= kDiagnosticsPeriodMs) {
    const uint64_t backend_total =
      backend_sequence_advance_count_.load(std::memory_order_relaxed);
    const uint64_t callback_total = tactile_callback_count_.load(std::memory_order_relaxed);
    const uint64_t buffer_drop_total =
      tactile_buffer_drop_count_.load(std::memory_order_relaxed);
    const uint64_t callback_thread_id =
      tactile_callback_thread_id_.load(std::memory_order_relaxed);
    const double elapsed_s = static_cast<double>(elapsed_ms) / 1000.0;
    const double backend_hz = static_cast<double>(
      backend_total - diagnostics_last_backend_sequence_advance_count_) / elapsed_s;
    const double callback_hz =
      static_cast<double>(callback_total - diagnostics_last_callback_count_) / elapsed_s;
    const double processed_hz = static_cast<double>(
      processed_tactile_frame_count_ - diagnostics_last_processed_tactile_frame_count_) / elapsed_s;
    const double qtimer_tick_hz = static_cast<double>(
      gui_timer_tick_count_ - diagnostics_last_gui_timer_tick_count_) / elapsed_s;
    const double gui_frame_hz =
      static_cast<double>(gui_refresh_count_ - diagnostics_last_gui_refresh_count_) / elapsed_s;
    const double actual_render_hz = static_cast<double>(
      actual_render_count_ - diagnostics_last_actual_render_count_) / elapsed_s;
    const uint64_t buffer_drops = buffer_drop_total - diagnostics_last_buffer_drop_count_;
    RCLCPP_INFO(
      node_->get_logger(),
      "[DISPLAY_DIAGNOSTIC] backend_hz=%.1f callback_hz=%.1f qtimer_tick_hz=%.1f "
      "gui_frame_hz=%.1f actual_render_hz=%.1f processed_hz=%.1f "
      "buffer_drops=%llu max_batch=%llu callback_tid=%llu qtimer_tid=%llu "
      "render_tid=%llu",
      backend_hz, callback_hz, qtimer_tick_hz, gui_frame_hz, actual_render_hz, processed_hz,
      static_cast<unsigned long long>(buffer_drops),
      static_cast<unsigned long long>(gui_max_batch_frame_count_),
      static_cast<unsigned long long>(callback_thread_id),
      static_cast<unsigned long long>(gui_timer_thread_id_),
      static_cast<unsigned long long>(actual_render_thread_id_));
    diagnostics_last_backend_sequence_advance_count_ = backend_total;
    diagnostics_last_callback_count_ = callback_total;
    diagnostics_last_processed_tactile_frame_count_ = processed_tactile_frame_count_;
    diagnostics_last_buffer_drop_count_ = buffer_drop_total;
    diagnostics_last_gui_timer_tick_count_ = gui_timer_tick_count_;
    diagnostics_last_gui_refresh_count_ = gui_refresh_count_;
    diagnostics_last_actual_render_count_ = actual_render_count_;
    gui_max_batch_frame_count_ = 0;
    diagnostics_age_.restart();
  }
}

void HandControlPanel::onControlStatus(const ControlStatus::SharedPtr message)
{
  QMetaObject::invokeMethod(
    this, [this, copy = *message]() {applyControlStatus(copy);}, Qt::QueuedConnection);
}

size_t HandControlPanel::applyTactileFrames(
  const std::deque<TactileFrame::SharedPtr> & messages)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> batch_peak_absolute_delta;
  std::vector<double> batch_peak_signed_delta;
  bool batch_temporal_ready = false;
  size_t applied_count = 0;

  for (const auto & message_pointer : messages) {
    if (!message_pointer) {
      continue;
    }
    const TactileFrame & message = *message_pointer;
    const uint32_t rows = message.rows;
    const uint32_t cols = message.cols;
    const size_t cell_count = static_cast<size_t>(rows) * cols;
    if (rows == 0 || cols == 0 || cell_count != message.pressure.size()) {
      continue;
    }
    if (have_sequence_ && latest_sequence_ == message.sequence) {
      continue;
    }

    const bool shape_changed = have_frame_ && (rows_ != rows || cols_ != cols);
    std::vector<double> current(message.pressure.begin(), message.pressure.end());
    if (shape_changed) {
      previous_raw_.clear();
      baseline_ready_ = false;
      baseline_.clear();
      baseline_valid_.clear();
      clearBaselineCapture();
      baseline_state_label_->setText(
        "Tactile layout changed; old baseline and temporal predecessor were discarded.");
      batch_peak_absolute_delta.assign(cell_count, nan);
      batch_peak_signed_delta.assign(cell_count, nan);
      batch_temporal_ready = false;
    } else if (batch_peak_absolute_delta.size() != cell_count) {
      batch_peak_absolute_delta.assign(cell_count, nan);
      batch_peak_signed_delta.assign(cell_count, nan);
    }

    const bool have_predecessor = previous_raw_.size() == current.size();
    if (have_predecessor) {
      batch_temporal_ready = true;
      for (size_t index = 0; index < current.size(); ++index) {
        if (!finite(current[index]) || !finite(previous_raw_[index])) {
          continue;
        }
        const double delta = current[index] - previous_raw_[index];
        const double magnitude = std::abs(delta);
        if (!finite(batch_peak_absolute_delta[index]) ||
          magnitude > batch_peak_absolute_delta[index])
        {
          batch_peak_absolute_delta[index] = magnitude;
          batch_peak_signed_delta[index] = delta;
        }
      }
    }

    rows_ = rows;
    cols_ = cols;
    raw_pressure_ = std::move(current);
    have_frame_ = true;
    have_sequence_ = true;
    latest_sequence_ = message.sequence;
    previous_raw_ = raw_pressure_;
    advanceBaselineCapture();
    ++applied_count;
  }

  if (applied_count == 0) {
    return 0;
  }
  temporal_signed_delta_ = std::move(batch_peak_signed_delta);
  temporal_peak_absolute_delta_ = std::move(batch_peak_absolute_delta);
  temporal_batch_frame_count_ = applied_count;
  temporal_ready_ = batch_temporal_ready;
  renderLatest();
  return applied_count;
}

void HandControlPanel::applyControlStatus(const ControlStatus & message)
{
  latest_status_ = message;
  have_status_ = true;
  status_age_.restart();
  connection_label_->setText(
    QString("Control state: %1; SDK %2. %3")
    .arg(stateName(message.state))
    .arg(message.connected ? "connected" : "not connected")
    .arg(QString::fromStdString(message.detail)));
  refreshControlButtons();
}

void HandControlPanel::renderLatest()
{
  if (!have_frame_) {
    return;
  }
  ++actual_render_count_;
  actual_render_thread_id_ = currentThreadId();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<uint8_t> all_active(raw_pressure_.size(), 1);
  raw_heatmap_->setGrid(rows_, cols_, raw_pressure_, all_active, raw_vmax_->value());

  double raw_sum = 0.0;
  double raw_maximum = -std::numeric_limits<double>::infinity();
  size_t raw_count = 0;
  size_t raw_max_index = 0;
  for (size_t index = 0; index < raw_pressure_.size(); ++index) {
    const double value = raw_pressure_[index];
    if (!finite(value)) {
      continue;
    }
    raw_sum += value;
    ++raw_count;
    if (value > raw_maximum) {
      raw_maximum = value;
      raw_max_index = index;
    }
  }
  layout_label_->setText(
    QString("Layout %1x%2, %3 values, sequence=%4, finite raw taxels=%5")
    .arg(rows_).arg(cols_).arg(raw_pressure_.size()).arg(latest_sequence_).arg(raw_count));
  raw_stats_label_->setText(
    raw_count > 0 ?
    QString("max=%1 @ %2; mean=%3; fixed vmax=%4")
    .arg(raw_maximum, 0, 'f', 5).arg(formatCell(raw_max_index, cols_))
    .arg(raw_sum / raw_count, 0, 'f', 5).arg(raw_vmax_->value(), 0, 'f', 3) :
    QString("No finite raw taxels."));

  std::vector<double> baseline_display(raw_pressure_.size(), nan);
  std::vector<double> baseline_positive(raw_pressure_.size(), nan);
  std::vector<uint8_t> baseline_active(raw_pressure_.size(), 0);
  QString baseline_overlay;
  if (baseline_ready_ && baseline_.size() == raw_pressure_.size()) {
    const double offset = baseline_offset_->value();
    const double threshold = baseline_threshold_->value();
    for (size_t index = 0; index < raw_pressure_.size(); ++index) {
      if (!finite(raw_pressure_[index]) || index >= baseline_valid_.size() || baseline_valid_[index] == 0) {
        continue;
      }
      const double signed_delta = raw_pressure_[index] - baseline_[index];
      baseline_positive[index] = std::max(signed_delta, 0.0);
      baseline_display[index] = std::max(signed_delta - offset, 0.0);
      baseline_active[index] = baseline_display[index] >= threshold ? 1 : 0;
    }
  } else {
    baseline_overlay = baseline_capture_active_ ?
      "Capturing 2 s median baseline...\nKeep the tactile glove free of external contact." :
      "Baseline not captured\nResidual data is uninitialized.";
  }
  baseline_heatmap_->setGrid(
    rows_, cols_, baseline_display, baseline_active, baseline_vmax_->value(), baseline_overlay);
  double baseline_sum = 0.0;
  double baseline_maximum = -std::numeric_limits<double>::infinity();
  size_t baseline_count = 0;
  size_t baseline_max_index = 0;
  for (size_t index = 0; index < baseline_positive.size(); ++index) {
    if (!finite(baseline_positive[index])) {
      continue;
    }
    baseline_sum += baseline_positive[index];
    ++baseline_count;
    if (baseline_positive[index] > baseline_maximum) {
      baseline_maximum = baseline_positive[index];
      baseline_max_index = index;
    }
  }
  baseline_stats_label_->setText(
    baseline_count > 0 ?
    QString("+max=%1 @ %2; +mean=%3; stats are unclipped/unthresholded")
    .arg(baseline_maximum, 0, 'f', 5).arg(formatCell(baseline_max_index, cols_))
    .arg(baseline_sum / baseline_count, 0, 'f', 5) :
    QString("Baseline residual unavailable."));

  std::vector<uint8_t> temporal_active(raw_pressure_.size(), 1);
  temporal_heatmap_->setGrid(
    rows_, cols_, temporal_peak_absolute_delta_, temporal_active, temporal_vmax_->value(),
    temporal_ready_ ? QString() : QString("Waiting for the next compatible high-rate frame."));
  double temporal_sum = 0.0;
  double temporal_maximum = -std::numeric_limits<double>::infinity();
  double temporal_signed_at_maximum = std::numeric_limits<double>::quiet_NaN();
  size_t temporal_count = 0;
  size_t temporal_max_index = 0;
  for (size_t index = 0; index < temporal_peak_absolute_delta_.size(); ++index) {
    if (!finite(temporal_peak_absolute_delta_[index])) {
      continue;
    }
    temporal_sum += temporal_peak_absolute_delta_[index];
    ++temporal_count;
    if (temporal_peak_absolute_delta_[index] > temporal_maximum) {
      temporal_maximum = temporal_peak_absolute_delta_[index];
      temporal_max_index = index;
      temporal_signed_at_maximum = temporal_signed_delta_[index];
    }
  }
  temporal_stats_label_->setText(
    temporal_count > 0 ?
    QString("|delta| peak=%1 (signed %2) @ %3; |delta| mean=%4; "
      "%5 high-rate frames aggregated; no threshold")
    .arg(temporal_maximum, 0, 'f', 5)
    .arg(temporal_signed_at_maximum, 0, 'f', 5)
    .arg(formatCell(temporal_max_index, cols_))
    .arg(temporal_sum / temporal_count, 0, 'f', 5)
    .arg(temporal_batch_frame_count_) :
    QString("Temporal delta unavailable."));
}

void HandControlPanel::refreshControlButtons()
{
  const bool fresh = have_status_ && status_age_.isValid() && status_age_.elapsed() <= kStatusTimeoutMs;
  const bool connected = fresh && latest_status_.connected;
  const uint8_t state = fresh ? latest_status_.state : ControlStatus::DISABLED;
  const bool moving = state == ControlStatus::MOVING_TO_POSE;
  const bool pose_selector_enabled = !moving;
  const bool move_pose_enabled = connected && state == ControlStatus::IDLE;
  if (pose_selector_->isEnabled() != pose_selector_enabled) {
    pose_selector_->setEnabled(pose_selector_enabled);
  }
  if (move_pose_button_->isEnabled() != move_pose_enabled) {
    move_pose_button_->setEnabled(move_pose_enabled);
  }
  if (open_button_->isEnabled() != move_pose_enabled) {
    open_button_->setEnabled(move_pose_enabled);
  }
  enable_button_->setEnabled(connected && state == ControlStatus::DISABLED);
  disable_button_->setEnabled(command_client_ != nullptr && command_client_->service_is_ready());
}

void HandControlPanel::statusWatchTick()
{
  if (!have_status_ || !status_age_.isValid() || status_age_.elapsed() > kStatusTimeoutMs) {
    connection_label_->setText(
      "Control status heartbeat is absent/stale; all motion controls are locked. Disable remains available as a best-effort request.");
  }
  refreshControlButtons();
}

void HandControlPanel::startBaselineCapture()
{
  if (!have_frame_) {
    baseline_state_label_->setText("No tactile frame is available; baseline capture was not started.");
    return;
  }
  baseline_ready_ = false;
  baseline_.clear();
  baseline_valid_.clear();
  baseline_capture_active_ = true;
  baseline_capture_has_first_sample_ = false;
  baseline_capture_last_sequence_ = latest_sequence_;
  baseline_capture_frame_count_ = 0;
  baseline_samples_.assign(raw_pressure_.size(), {});
  baseline_state_label_->setText("Waiting for the first fresh frame of the 2 s median baseline capture.");
  renderLatest();
}

void HandControlPanel::resetBaseline()
{
  baseline_ready_ = false;
  baseline_.clear();
  baseline_valid_.clear();
  clearBaselineCapture();
  baseline_state_label_->setText("Baseline reset; residual data is intentionally uninitialized.");
  renderLatest();
}

void HandControlPanel::advanceBaselineCapture()
{
  if (!baseline_capture_active_ || latest_sequence_ == baseline_capture_last_sequence_) {
    return;
  }
  if (baseline_samples_.size() != raw_pressure_.size()) {
    resetBaseline();
    baseline_state_label_->setText("Baseline capture cancelled because the tactile layout changed.");
    return;
  }
  baseline_capture_last_sequence_ = latest_sequence_;
  const auto now = std::chrono::steady_clock::now();
  if (!baseline_capture_has_first_sample_) {
    baseline_capture_first_sample_ = now;
    baseline_capture_has_first_sample_ = true;
  }
  for (size_t index = 0; index < raw_pressure_.size(); ++index) {
    if (finite(raw_pressure_[index])) {
      baseline_samples_[index].push_back(raw_pressure_[index]);
    }
  }
  ++baseline_capture_frame_count_;
  const double elapsed = std::chrono::duration<double>(now - baseline_capture_first_sample_).count();
  if (elapsed < kBaselineCaptureSeconds) {
    baseline_state_label_->setText(
      QString("Capturing baseline: %1/2.0 s, %2 fresh frames; keep external contact removed.")
      .arg(elapsed, 0, 'f', 1).arg(baseline_capture_frame_count_));
    return;
  }

  baseline_.assign(raw_pressure_.size(), std::numeric_limits<double>::quiet_NaN());
  baseline_valid_.assign(raw_pressure_.size(), 0);
  size_t valid_count = 0;
  for (size_t index = 0; index < baseline_samples_.size(); ++index) {
    if (baseline_samples_[index].size() >= kMinimumBaselineSamples) {
      baseline_[index] = median(std::move(baseline_samples_[index]));
      baseline_valid_[index] = 1;
      ++valid_count;
    }
  }
  const size_t frame_count = baseline_capture_frame_count_;
  baseline_ready_ = true;
  clearBaselineCapture();
  baseline_state_label_->setText(
    QString("Baseline complete: 2 s median, %1 frames, %2/%3 valid taxels (>=5 finite samples each).")
    .arg(frame_count).arg(valid_count).arg(raw_pressure_.size()));
}

void HandControlPanel::clearBaselineCapture()
{
  baseline_capture_active_ = false;
  baseline_capture_has_first_sample_ = false;
  baseline_capture_frame_count_ = 0;
  baseline_samples_.clear();
}

void HandControlPanel::sendAction(
  uint8_t action, const QString & label, const QString & pose_id)
{
  if (!command_client_ || !command_client_->service_is_ready()) {
    request_result_label_->setText(QString("%1 not sent: constrained backend service is unavailable.").arg(label));
    refreshControlButtons();
    return;
  }
  auto request = std::make_shared<ControlCommand::Request>();
  request->action = action;
  request->pose_id = pose_id.toStdString();
  request_result_label_->setText(QString("%1 requested; waiting for backend acceptance...").arg(label));
  QPointer<HandControlPanel> guard(this);
  command_client_->async_send_request(
    request, [guard, label](rclcpp::Client<ControlCommand>::SharedFuture future) {
      if (guard.isNull()) {
        return;
      }
      try {
        const auto response = future.get();
        const QString detail = QString::fromStdString(response->detail);
        QMetaObject::invokeMethod(
          guard.data(), [guard, label, accepted = response->accepted, detail]() {
            if (!guard.isNull()) {
              guard->request_result_label_->setText(
                QString("%1 %2: %3").arg(label).arg(accepted ? "accepted" : "refused").arg(detail));
            }
          }, Qt::QueuedConnection);
      } catch (const std::exception & error) {
        const QString detail = QString::fromUtf8(error.what());
        QMetaObject::invokeMethod(
          guard.data(), [guard, label, detail]() {
            if (!guard.isNull()) {
              guard->request_result_label_->setText(QString("%1 service failed: %2").arg(label).arg(detail));
            }
          }, Qt::QueuedConnection);
      }
    });
}

void HandControlPanel::requestDisableBestEffort()
{
  try {
    if (!command_client_ || !command_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<ControlCommand::Request>();
    request->action = ControlCommand::Request::DISABLE;
    command_client_->async_send_request(request);
  } catch (const std::exception &) {
    // The UI heartbeat watchdog remains the shutdown fail-safe if ROS is already tearing down.
  }
}

void HandControlPanel::publishHeartbeat()
{
  if (heartbeat_publisher_) {
    heartbeat_publisher_->publish(std_msgs::msg::Empty());
  }
}

double HandControlPanel::median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  if (values.size() % 2 == 1) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1] + values[middle]);
}

QString HandControlPanel::stateName(uint8_t state)
{
  switch (state) {
    case ControlStatus::DISABLED: return "DISABLED";
    case ControlStatus::IDLE: return "IDLE";
    case ControlStatus::MOVING_TO_POSE: return "MOVING_TO_POSE";
    default: return QString("UNKNOWN(%1)").arg(state);
  }
}

}  // namespace wuji_rviz_panel

PLUGINLIB_EXPORT_CLASS(wuji_rviz_panel::HandControlPanel, rviz_common::Panel)
