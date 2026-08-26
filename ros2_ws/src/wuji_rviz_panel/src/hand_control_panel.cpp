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
#include <QSlider>
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
#include <QSignalBlocker>
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
constexpr double kBaselineCaptureSeconds = 5.0;
constexpr double kThresholdCaptureSeconds = 5.0;
constexpr size_t kMinimumValidFrames = 300;
constexpr size_t kMinimumBaselineSamples = 5;
constexpr double kAutoThresholdPercentile = 99.9;
constexpr double kHighNoiseWarningThreshold = 0.2;
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
constexpr double kBaselineVmaxMinimum = 0.05;
constexpr double kTemporalVmaxMinimum = 0.01;
constexpr double kVmaxNeutralMaximum = 1.0;
constexpr double kBaselineThresholdMaximum = 1.0;
constexpr int kSensitivitySliderMaximum = 10000;
constexpr int kThresholdSliderMaximum = 100000;
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

double sensitivitySliderToVmax(int slider_position, double vmax_minimum)
{
  if (slider_position <= 0) {
    return kVmaxNeutralMaximum;
  }
  if (slider_position >= kSensitivitySliderMaximum) {
    return vmax_minimum;
  }
  const double sensitivity =
    static_cast<double>(slider_position) / kSensitivitySliderMaximum;
  return std::exp(
    std::log(kVmaxNeutralMaximum) +
    sensitivity * (std::log(vmax_minimum) - std::log(kVmaxNeutralMaximum)));
}

int vmaxToSensitivitySlider(double vmax, double vmax_minimum)
{
  if (vmax >= kVmaxNeutralMaximum) {
    return 0;
  }
  if (vmax <= vmax_minimum) {
    return kSensitivitySliderMaximum;
  }
  const double sensitivity =
    (std::log(vmax) - std::log(kVmaxNeutralMaximum)) /
    (std::log(vmax_minimum) - std::log(kVmaxNeutralMaximum));
  return static_cast<int>(std::lround(sensitivity * kSensitivitySliderMaximum));
}

double thresholdSliderToValue(int slider_position)
{
  if (slider_position <= 0) {
    return 0.0;
  }
  if (slider_position >= kThresholdSliderMaximum) {
    return kBaselineThresholdMaximum;
  }
  return static_cast<double>(slider_position) / kThresholdSliderMaximum;
}

QString vmaxValueText(double vmax)
{
  return QString("vmax=%1").arg(vmax, 0, 'f', 3);
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
        color = is_active ? turboColor(values_[index], fixed_maximum).rgb() : inactive_color;
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

QColor HeatmapWidget::turboColor(double value, double maximum)
{
  // Polynomial approximation of Google's Turbo color map. The heatmap still
  // clips only at the drawing stage; stored values and statistics are intact.
  static constexpr std::array<double, 6> red_coefficients{
    0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943};
  static constexpr std::array<double, 6> green_coefficients{
    0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604};
  static constexpr std::array<double, 6> blue_coefficients{
    0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973};

  const double ratio = std::clamp(value / maximum, 0.0, 1.0);
  const auto evaluate = [ratio](const std::array<double, 6> & coefficients) {
      return (((((coefficients[5] * ratio + coefficients[4]) * ratio + coefficients[3]) *
             ratio + coefficients[2]) * ratio + coefficients[1]) * ratio + coefficients[0]);
    };
  const auto channel = [](double component) {
      return static_cast<int>(std::lround(std::clamp(component, 0.0, 1.0) * 255.0));
    };
  return QColor(
    channel(evaluate(red_coefficients)),
    channel(evaluate(green_coefficients)),
    channel(evaluate(blue_coefficients)));
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

  auto * settings = new QGroupBox(QString::fromUtf8("显示设置"));
  auto * settings_layout = new QGridLayout(settings);

  // Keep the existing value holders for internal compatibility. Only the
  // three semantic sliders below are exposed to the operator.
  raw_vmax_ = new QDoubleSpinBox(settings);
  baseline_vmax_ = new QDoubleSpinBox(settings);
  baseline_threshold_ = new QDoubleSpinBox(settings);
  baseline_offset_ = new QDoubleSpinBox(settings);
  temporal_vmax_ = new QDoubleSpinBox(settings);
  for (auto * spin : {raw_vmax_, baseline_vmax_, baseline_threshold_, baseline_offset_,
      temporal_vmax_})
  {
    spin->setDecimals(5);
    spin->setSingleStep(0.001);
    spin->hide();
  }
  raw_vmax_->setObjectName("raw_vmax_internal");
  baseline_vmax_->setObjectName("baseline_vmax_internal");
  baseline_threshold_->setObjectName("baseline_threshold_internal");
  baseline_offset_->setObjectName("baseline_offset_internal");
  temporal_vmax_->setObjectName("temporal_vmax_internal");
  raw_vmax_->setRange(kRawDefaultMaximum, kRawDefaultMaximum);
  raw_vmax_->setValue(kRawDefaultMaximum);
  baseline_vmax_->setRange(kBaselineVmaxMinimum, kVmaxNeutralMaximum);
  baseline_vmax_->setValue(kBaselineDefaultMaximum);
  baseline_threshold_->setDecimals(15);
  baseline_threshold_->setRange(0.0, kBaselineThresholdMaximum);
  baseline_threshold_->setValue(kBaselineDefaultThreshold);
  baseline_offset_->setRange(0.0, 0.0);
  baseline_offset_->setValue(0.0);
  temporal_vmax_->setRange(kTemporalVmaxMinimum, kVmaxNeutralMaximum);
  temporal_vmax_->setValue(kTemporalDefaultMaximum);

  contact_sensitivity_slider_ = new QSlider(Qt::Horizontal, settings);
  contact_threshold_slider_ = new QSlider(Qt::Horizontal, settings);
  dynamic_sensitivity_slider_ = new QSlider(Qt::Horizontal, settings);
  contact_sensitivity_slider_->setObjectName("contact_sensitivity_slider");
  contact_threshold_slider_->setObjectName("contact_threshold_slider");
  dynamic_sensitivity_slider_->setObjectName("dynamic_sensitivity_slider");
  contact_sensitivity_slider_->setRange(0, kSensitivitySliderMaximum);
  dynamic_sensitivity_slider_->setRange(0, kSensitivitySliderMaximum);
  contact_threshold_slider_->setRange(0, kThresholdSliderMaximum);
  contact_sensitivity_slider_->setValue(
    vmaxToSensitivitySlider(kBaselineDefaultMaximum, kBaselineVmaxMinimum));
  dynamic_sensitivity_slider_->setValue(
    vmaxToSensitivitySlider(kTemporalDefaultMaximum, kTemporalVmaxMinimum));
  contact_threshold_slider_->setValue(
    static_cast<int>(std::lround(kBaselineDefaultThreshold * kThresholdSliderMaximum)));
  for (auto * slider :
    {contact_sensitivity_slider_, contact_threshold_slider_, dynamic_sensitivity_slider_})
  {
    slider->setTracking(true);
    slider->setSingleStep(1);
    slider->setPageStep(100);
  }

  auto * contact_sensitivity_label = new QLabel(QString::fromUtf8("接触灵敏度"), settings);
  auto * contact_threshold_label = new QLabel(QString::fromUtf8("接触阈值"), settings);
  auto * dynamic_sensitivity_label = new QLabel(QString::fromUtf8("动态灵敏度"), settings);
  contact_sensitivity_value_label_ =
    new QLabel(vmaxValueText(baseline_vmax_->value()), settings);
  contact_threshold_value_label_ = new QLabel(settings);
  dynamic_sensitivity_value_label_ =
    new QLabel(vmaxValueText(temporal_vmax_->value()), settings);
  contact_sensitivity_value_label_->setObjectName("contact_sensitivity_value");
  contact_threshold_value_label_->setObjectName("contact_threshold_value");
  dynamic_sensitivity_value_label_->setObjectName("dynamic_sensitivity_value");
  for (auto * value_label :
    {contact_sensitivity_value_label_, contact_threshold_value_label_,
      dynamic_sensitivity_value_label_})
  {
    value_label->setMinimumWidth(82);
  }

  const QString contact_tooltip = QString::fromUtf8(
    "调高后更容易看到微弱的新增接触。\n最低灵敏度对应完整 [0,1] 显示范围。");
  const QString threshold_tooltip = QString::fromUtf8(
    "过滤无接触状态下的微弱噪声。\n执行 Baseline 标定后可自动估计。\n"
    "设为“关闭”时不进行阈值过滤。");
  const QString dynamic_tooltip = QString::fromUtf8(
    "调高后更容易看到接触、释放、滑动和冲击等快速变化。\n"
    "最低灵敏度对应完整 [0,1] 显示范围。");
  contact_sensitivity_label->setToolTip(contact_tooltip);
  contact_sensitivity_slider_->setToolTip(contact_tooltip);
  contact_sensitivity_value_label_->setToolTip(contact_tooltip);
  contact_threshold_label->setToolTip(threshold_tooltip);
  contact_threshold_slider_->setToolTip(threshold_tooltip);
  contact_threshold_value_label_->setToolTip(threshold_tooltip);
  dynamic_sensitivity_label->setToolTip(dynamic_tooltip);
  dynamic_sensitivity_slider_->setToolTip(dynamic_tooltip);
  dynamic_sensitivity_value_label_->setToolTip(dynamic_tooltip);

  settings_layout->addWidget(contact_sensitivity_label, 0, 0);
  settings_layout->addWidget(new QLabel(QString::fromUtf8("低"), settings), 0, 1);
  settings_layout->addWidget(contact_sensitivity_slider_, 0, 2);
  settings_layout->addWidget(new QLabel(QString::fromUtf8("高"), settings), 0, 3);
  settings_layout->addWidget(contact_sensitivity_value_label_, 0, 4);
  settings_layout->addWidget(contact_threshold_label, 1, 0);
  settings_layout->addWidget(new QLabel(QString::fromUtf8("关闭"), settings), 1, 1);
  settings_layout->addWidget(contact_threshold_slider_, 1, 2);
  settings_layout->addWidget(new QLabel(QString::fromUtf8("高"), settings), 1, 3);
  settings_layout->addWidget(contact_threshold_value_label_, 1, 4);
  auto_threshold_button_ = new QPushButton(QString::fromUtf8("自动估计"), settings);
  auto_threshold_button_->setObjectName("auto_threshold_button");
  auto_threshold_button_->setToolTip(threshold_tooltip);
  settings_layout->addWidget(auto_threshold_button_, 1, 5);
  settings_layout->addWidget(dynamic_sensitivity_label, 2, 0);
  settings_layout->addWidget(new QLabel(QString::fromUtf8("低"), settings), 2, 1);
  settings_layout->addWidget(dynamic_sensitivity_slider_, 2, 2);
  settings_layout->addWidget(new QLabel(QString::fromUtf8("高"), settings), 2, 3);
  settings_layout->addWidget(dynamic_sensitivity_value_label_, 2, 4);
  settings_layout->setColumnStretch(2, 1);

  capture_baseline_button_ = new QPushButton("Capture Baseline");
  capture_baseline_button_->setObjectName("capture_baseline_button");
  reset_baseline_button_ = new QPushButton("Reset Baseline");
  auto * baseline_buttons = new QHBoxLayout();
  baseline_buttons->addWidget(capture_baseline_button_);
  baseline_buttons->addWidget(reset_baseline_button_);
  baseline_buttons->addStretch(1);
  settings_layout->addLayout(baseline_buttons, 3, 0, 1, 6);
  baseline_state_label_ = new QLabel("Baseline not captured; residual data is intentionally uninitialized.");
  baseline_state_label_->setObjectName("baseline_state_label");
  baseline_state_label_->setWordWrap(true);
  baseline_state_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  settings_layout->addWidget(baseline_state_label_, 4, 0, 1, 6);
  updateThresholdDisplay();
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
  connect(auto_threshold_button_, &QPushButton::clicked, this, &HandControlPanel::startThresholdCapture);
  connect(reset_baseline_button_, &QPushButton::clicked, this, &HandControlPanel::resetBaseline);
  connect(
    contact_sensitivity_slider_, &QSlider::valueChanged, this, [this](int slider_position) {
      baseline_vmax_->setValue(
        sensitivitySliderToVmax(slider_position, kBaselineVmaxMinimum));
      contact_sensitivity_value_label_->setText(vmaxValueText(baseline_vmax_->value()));
      renderLatest();
    });
  connect(
    contact_threshold_slider_, &QSlider::valueChanged, this, [this](int slider_position) {
      baseline_threshold_->setValue(thresholdSliderToValue(slider_position));
      threshold_mode_ =
        slider_position == 0 ? ThresholdMode::Off : ThresholdMode::Manual;
      updateThresholdDisplay();
      renderLatest();
    });
  connect(contact_threshold_slider_, &QSlider::sliderPressed, this, [this]() {
    threshold_mode_ =
      contact_threshold_slider_->value() == 0 ?
      ThresholdMode::Off : ThresholdMode::Manual;
    updateThresholdDisplay();
  });
  connect(
    dynamic_sensitivity_slider_, &QSlider::valueChanged, this, [this](int slider_position) {
      temporal_vmax_->setValue(
        sensitivitySliderToVmax(slider_position, kTemporalVmaxMinimum));
      dynamic_sensitivity_value_label_->setText(vmaxValueText(temporal_vmax_->value()));
      renderLatest();
    });
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
    if (threshold_capture_active_) {
      baseline_overlay = QString::fromUtf8(
        "正在估计接触阈值...\n请保持手套无任何外部接触。");
    }
  } else {
    baseline_overlay = baseline_capture_active_ ?
      QString::fromUtf8("正在采集 Baseline...\n请保持手套无任何外部接触。") :
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
  clearBaselineCapture();
  baseline_ready_ = false;
  baseline_.clear();
  baseline_valid_.clear();
  baseline_valid_taxel_count_ = 0;
  baseline_capture_active_ = true;
  baseline_capture_has_first_sample_ = false;
  full_calibration_active_ = true;
  baseline_capture_last_sequence_ = latest_sequence_;
  baseline_capture_frame_count_ = 0;
  threshold_capture_frame_count_ = 0;
  baseline_samples_.assign(raw_pressure_.size(), {});
  auto_threshold_button_->setEnabled(false);
  baseline_state_label_->setText(QString::fromUtf8(
      "正在等待新的 tactile frame 以开始 Baseline 标定；请保持手套无任何外部接触。"));
  renderLatest();
}

void HandControlPanel::startThresholdCapture()
{
  const bool have_valid_baseline =
    baseline_ready_ && baseline_.size() == raw_pressure_.size() &&
    baseline_valid_.size() == baseline_.size() &&
    std::any_of(
    baseline_valid_.begin(), baseline_valid_.end(),
    [](uint8_t valid) {return valid != 0;});
  if (!have_valid_baseline) {
    baseline_state_label_->setText(QString::fromUtf8(
        "无法自动估计接触阈值：当前没有有效 Baseline，请先执行 Capture Baseline。"));
    return;
  }
  beginThresholdCapture(false);
  renderLatest();
}

void HandControlPanel::beginThresholdCapture(bool part_of_full_calibration)
{
  baseline_capture_active_ = false;
  baseline_capture_has_first_sample_ = false;
  baseline_samples_.clear();
  threshold_capture_active_ = true;
  threshold_capture_has_first_sample_ = false;
  full_calibration_active_ = part_of_full_calibration;
  threshold_capture_last_sequence_ = latest_sequence_;
  threshold_capture_frame_count_ = 0;
  threshold_capture_duration_seconds_ = 0.0;
  threshold_residual_samples_.clear();
  residual_median_ = std::numeric_limits<double>::quiet_NaN();
  residual_p95_ = std::numeric_limits<double>::quiet_NaN();
  residual_p99_ = std::numeric_limits<double>::quiet_NaN();
  residual_p999_ = std::numeric_limits<double>::quiet_NaN();
  residual_maximum_ = std::numeric_limits<double>::quiet_NaN();
  auto_threshold_ = std::numeric_limits<double>::quiet_NaN();
  auto_threshold_button_->setEnabled(false);
  baseline_state_label_->setText(QString::fromUtf8(
      "正在等待新的 tactile frame 以估计接触阈值；Baseline 已固定，请继续保持无外部接触。"));
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
  const auto now = std::chrono::steady_clock::now();
  if (threshold_capture_active_) {
    advanceThresholdCapture(now);
    return;
  }
  if (!baseline_capture_active_ || latest_sequence_ == baseline_capture_last_sequence_) {
    return;
  }
  if (baseline_samples_.size() != raw_pressure_.size()) {
    resetBaseline();
    baseline_state_label_->setText("Baseline capture cancelled because the tactile layout changed.");
    return;
  }
  baseline_capture_last_sequence_ = latest_sequence_;
  if (!baseline_capture_has_first_sample_) {
    baseline_capture_first_sample_ = now;
    baseline_capture_has_first_sample_ = true;
  }
  bool valid_frame = false;
  for (size_t index = 0; index < raw_pressure_.size(); ++index) {
    if (finite(raw_pressure_[index])) {
      baseline_samples_[index].push_back(raw_pressure_[index]);
      valid_frame = true;
    }
  }
  if (valid_frame) {
    ++baseline_capture_frame_count_;
  }
  const double elapsed = std::chrono::duration<double>(now - baseline_capture_first_sample_).count();
  if (elapsed < kBaselineCaptureSeconds ||
    baseline_capture_frame_count_ < kMinimumValidFrames)
  {
    const bool waiting_for_frames =
      elapsed >= kBaselineCaptureSeconds &&
      baseline_capture_frame_count_ < kMinimumValidFrames;
    baseline_state_label_->setText(
      waiting_for_frames ?
      QString::fromUtf8(
        "正在等待足够有效数据... Baseline %1 / 5.0 s，有效帧 %2 / 300；请保持无外部接触。")
      .arg(elapsed, 0, 'f', 1).arg(baseline_capture_frame_count_) :
      QString::fromUtf8(
        "正在采集 Baseline... %1 / 5.0 s，有效帧 %2 / 300；请保持无外部接触。")
      .arg(elapsed, 0, 'f', 1).arg(baseline_capture_frame_count_));
    return;
  }

  baseline_capture_duration_seconds_ = elapsed;
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
  baseline_valid_taxel_count_ = valid_count;
  if (valid_count == 0) {
    baseline_ready_ = false;
    clearBaselineCapture();
    baseline_state_label_->setText(QString::fromUtf8(
        "Baseline 标定取消：没有 taxel 获得足够的 finite 样本，请检查数据有效性后重试。"));
    return;
  }
  baseline_ready_ = true;
  beginThresholdCapture(true);
}

void HandControlPanel::advanceThresholdCapture(
  const std::chrono::steady_clock::time_point & now)
{
  if (!threshold_capture_active_ || latest_sequence_ == threshold_capture_last_sequence_) {
    return;
  }
  if (!baseline_ready_ || baseline_.size() != raw_pressure_.size() ||
    baseline_valid_.size() != raw_pressure_.size())
  {
    clearBaselineCapture();
    baseline_state_label_->setText(QString::fromUtf8(
        "接触阈值估计已取消：Baseline 或 tactile 布局已失效。"));
    return;
  }

  threshold_capture_last_sequence_ = latest_sequence_;
  if (!threshold_capture_has_first_sample_) {
    threshold_capture_first_sample_ = now;
    threshold_capture_has_first_sample_ = true;
  }
  bool valid_frame = false;
  for (size_t index = 0; index < raw_pressure_.size(); ++index) {
    if (baseline_valid_[index] == 0 || !finite(baseline_[index]) ||
      !finite(raw_pressure_[index]))
    {
      continue;
    }
    const double residual = std::max(raw_pressure_[index] - baseline_[index], 0.0);
    if (finite(residual)) {
      threshold_residual_samples_.push_back(residual);
      valid_frame = true;
    }
  }
  if (valid_frame) {
    ++threshold_capture_frame_count_;
  }

  const double elapsed =
    std::chrono::duration<double>(now - threshold_capture_first_sample_).count();
  if (elapsed < kThresholdCaptureSeconds ||
    threshold_capture_frame_count_ < kMinimumValidFrames)
  {
    const bool waiting_for_frames =
      elapsed >= kThresholdCaptureSeconds &&
      threshold_capture_frame_count_ < kMinimumValidFrames;
    baseline_state_label_->setText(
      waiting_for_frames ?
      QString::fromUtf8(
        "正在等待足够有效数据... 接触阈值 %1 / 5.0 s，有效帧 %2 / 300；请保持无外部接触。")
      .arg(elapsed, 0, 'f', 1).arg(threshold_capture_frame_count_) :
      QString::fromUtf8(
        "正在估计接触阈值... %1 / 5.0 s，有效帧 %2 / 300；请保持无外部接触。")
      .arg(elapsed, 0, 'f', 1).arg(threshold_capture_frame_count_));
    return;
  }

  threshold_capture_duration_seconds_ = elapsed;
  std::sort(threshold_residual_samples_.begin(), threshold_residual_samples_.end());
  residual_median_ = percentileOfSorted(threshold_residual_samples_, 50.0);
  residual_p95_ = percentileOfSorted(threshold_residual_samples_, 95.0);
  residual_p99_ = percentileOfSorted(threshold_residual_samples_, 99.0);
  residual_p999_ =
    percentileOfSorted(threshold_residual_samples_, kAutoThresholdPercentile);
  residual_maximum_ = threshold_residual_samples_.empty() ?
    std::numeric_limits<double>::quiet_NaN() : threshold_residual_samples_.back();
  if (!finite(residual_p999_)) {
    clearBaselineCapture();
    baseline_state_label_->setText(QString::fromUtf8(
        "接触阈值估计已取消：Phase 2 没有可用于 P99.9 的 finite residual。"));
    return;
  }

  const bool completed_full_calibration = full_calibration_active_;
  setAutomaticThreshold(std::clamp(residual_p999_, 0.0, 1.0));
  threshold_capture_active_ = false;
  threshold_capture_has_first_sample_ = false;
  full_calibration_active_ = false;
  threshold_residual_samples_.clear();
  auto_threshold_button_->setEnabled(true);

  const QString high_noise_warning =
    auto_threshold_ > kHighNoiseWarningThreshold ?
    QString::fromUtf8(
      "\nBaseline noise is unusually high. "
      "Please check glove preload, fit, sensor drift, or stability.") :
    QString();
  baseline_state_label_->setText(
    completed_full_calibration ?
    QString::fromUtf8(
      "标定完成：Baseline %1 s / %2 有效帧，接触阈值 %3 s / %4 有效帧，"
      "P99.9=%5（自动）。%6")
    .arg(baseline_capture_duration_seconds_, 0, 'f', 2)
    .arg(baseline_capture_frame_count_)
    .arg(threshold_capture_duration_seconds_, 0, 'f', 2)
    .arg(threshold_capture_frame_count_)
    .arg(auto_threshold_, 0, 'f', 6)
    .arg(high_noise_warning) :
    QString::fromUtf8(
      "接触阈值自动估计完成：%1 s / %2 有效帧，P99.9=%3（自动）。%4")
    .arg(threshold_capture_duration_seconds_, 0, 'f', 2)
    .arg(threshold_capture_frame_count_)
    .arg(auto_threshold_, 0, 'f', 6)
    .arg(high_noise_warning));

  if (node_ != nullptr) {
    if (completed_full_calibration) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Baseline calibration completed:\n"
        "baseline_duration = %.6f s\n"
        "baseline_valid_frames = %zu\n"
        "threshold_duration = %.6f s\n"
        "threshold_valid_frames = %zu\n"
        "valid_taxels = %zu\n"
        "threshold_percentile = %.1f\n"
        "residual_median = %.9f\n"
        "residual_p95 = %.9f\n"
        "residual_p99 = %.9f\n"
        "residual_p99.9 = %.9f\n"
        "residual_max = %.9f\n"
        "auto_threshold = %.9f",
        baseline_capture_duration_seconds_, baseline_capture_frame_count_,
        threshold_capture_duration_seconds_, threshold_capture_frame_count_,
        baseline_valid_taxel_count_, kAutoThresholdPercentile,
        residual_median_, residual_p95_, residual_p99_, residual_p999_,
        residual_maximum_, auto_threshold_);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "Contact threshold estimation completed: duration=%.6f s, valid_frames=%zu, "
        "valid_taxels=%zu, percentile=%.1f, residual_median=%.9f, "
        "residual_p95=%.9f, residual_p99=%.9f, residual_p99.9=%.9f, "
        "residual_max=%.9f, auto_threshold=%.9f",
        threshold_capture_duration_seconds_, threshold_capture_frame_count_,
        baseline_valid_taxel_count_, kAutoThresholdPercentile,
        residual_median_, residual_p95_, residual_p99_, residual_p999_,
        residual_maximum_, auto_threshold_);
    }
    if (auto_threshold_ > kHighNoiseWarningThreshold) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Baseline noise is unusually high. "
        "Please check glove preload, fit, sensor drift, or stability.");
    }
  }
  renderLatest();
}

void HandControlPanel::clearBaselineCapture()
{
  baseline_capture_active_ = false;
  baseline_capture_has_first_sample_ = false;
  threshold_capture_active_ = false;
  threshold_capture_has_first_sample_ = false;
  full_calibration_active_ = false;
  baseline_capture_frame_count_ = 0;
  threshold_capture_frame_count_ = 0;
  baseline_samples_.clear();
  threshold_residual_samples_.clear();
  baseline_valid_taxel_count_ = 0;
  baseline_capture_duration_seconds_ = 0.0;
  threshold_capture_duration_seconds_ = 0.0;
  residual_median_ = std::numeric_limits<double>::quiet_NaN();
  residual_p95_ = std::numeric_limits<double>::quiet_NaN();
  residual_p99_ = std::numeric_limits<double>::quiet_NaN();
  residual_p999_ = std::numeric_limits<double>::quiet_NaN();
  residual_maximum_ = std::numeric_limits<double>::quiet_NaN();
  auto_threshold_ = std::numeric_limits<double>::quiet_NaN();
  if (auto_threshold_button_ != nullptr) {
    auto_threshold_button_->setEnabled(true);
  }
}

void HandControlPanel::updateThresholdDisplay()
{
  if (threshold_mode_ == ThresholdMode::Off) {
    contact_threshold_value_label_->setText(QString::fromUtf8("关闭"));
    return;
  }
  const double threshold = baseline_threshold_->value();
  const int precision = threshold > 0.0 && threshold < 0.01 ? 5 : 3;
  const QString suffix =
    threshold_mode_ == ThresholdMode::Auto ?
    QString::fromUtf8("（自动）") : QString::fromUtf8("（手动）");
  contact_threshold_value_label_->setText(
    QString::number(threshold, 'f', precision) + suffix);
}

void HandControlPanel::setAutomaticThreshold(double threshold)
{
  auto_threshold_ = std::clamp(threshold, 0.0, 1.0);
  const QSignalBlocker slider_blocker(contact_threshold_slider_);
  contact_threshold_slider_->setValue(
    static_cast<int>(std::lround(auto_threshold_ * kThresholdSliderMaximum)));
  baseline_threshold_->setValue(auto_threshold_);
  threshold_mode_ = ThresholdMode::Auto;
  updateThresholdDisplay();
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

double HandControlPanel::percentileOfSorted(
  const std::vector<double> & sorted_values, double percentile)
{
  if (sorted_values.empty() || !finite(percentile)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double bounded_percentile = std::clamp(percentile, 0.0, 100.0);
  const double position =
    (bounded_percentile / 100.0) * static_cast<double>(sorted_values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = static_cast<size_t>(std::ceil(position));
  if (lower == upper) {
    return sorted_values[lower];
  }
  const double fraction = position - static_cast<double>(lower);
  return sorted_values[lower] +
         fraction * (sorted_values[upper] - sorted_values[lower]);
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
