#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>

#include <gtest/gtest.h>

#include "wuji_rviz_panel/hand_control_panel.hpp"

namespace wuji_rviz_panel
{

class HandControlPanelTestPeer
{
public:
  using Frame = wuji_tactile_msgs::msg::TactilePressureFrame;

  static Frame::SharedPtr frame(uint32_t sequence, const std::vector<float> & pressure)
  {
    auto message = std::make_shared<Frame>();
    message->sequence = sequence;
    message->device_timestamp_ms = sequence * 8;
    message->rows = 1;
    message->cols = static_cast<uint32_t>(pressure.size());
    message->pressure = pressure;
    return message;
  }

  static void apply(
    HandControlPanel & panel, const std::vector<Frame::SharedPtr> & frames)
  {
    std::deque<Frame::SharedPtr> pending(frames.begin(), frames.end());
    panel.applyTactileFrames(pending);
  }

  static void apply(
    HandControlPanel & panel, uint32_t sequence, const std::vector<float> & pressure)
  {
    apply(panel, {frame(sequence, pressure)});
  }

  static void startBaseline(HandControlPanel & panel)
  {
    panel.startBaselineCapture();
  }

  static void makeBaselineDurationReady(HandControlPanel & panel)
  {
    panel.baseline_capture_first_sample_ =
      std::chrono::steady_clock::now() - std::chrono::milliseconds(5100);
  }

  static void makeThresholdDurationReady(HandControlPanel & panel)
  {
    panel.threshold_capture_first_sample_ =
      std::chrono::steady_clock::now() - std::chrono::milliseconds(5100);
  }

  static bool baselineCaptureActive(const HandControlPanel & panel)
  {
    return panel.baseline_capture_active_;
  }

  static bool thresholdCaptureActive(const HandControlPanel & panel)
  {
    return panel.threshold_capture_active_;
  }

  static bool baselineReady(const HandControlPanel & panel)
  {
    return panel.baseline_ready_;
  }

  static size_t baselineFrames(const HandControlPanel & panel)
  {
    return panel.baseline_capture_frame_count_;
  }

  static size_t thresholdFrames(const HandControlPanel & panel)
  {
    return panel.threshold_capture_frame_count_;
  }

  static double baselineDuration(const HandControlPanel & panel)
  {
    return panel.baseline_capture_duration_seconds_;
  }

  static double thresholdDuration(const HandControlPanel & panel)
  {
    return panel.threshold_capture_duration_seconds_;
  }

  static const std::vector<double> & baseline(const HandControlPanel & panel)
  {
    return panel.baseline_;
  }

  static const std::vector<double> & raw(const HandControlPanel & panel)
  {
    return panel.raw_pressure_;
  }

  static const std::vector<double> & temporal(const HandControlPanel & panel)
  {
    return panel.temporal_peak_absolute_delta_;
  }

  static double residualMedian(const HandControlPanel & panel)
  {
    return panel.residual_median_;
  }

  static double residualP99(const HandControlPanel & panel)
  {
    return panel.residual_p99_;
  }

  static double residualP999(const HandControlPanel & panel)
  {
    return panel.residual_p999_;
  }

  static double residualMaximum(const HandControlPanel & panel)
  {
    return panel.residual_maximum_;
  }

  static double autoThreshold(const HandControlPanel & panel)
  {
    return panel.auto_threshold_;
  }

  static bool thresholdIsAuto(const HandControlPanel & panel)
  {
    return panel.threshold_mode_ == HandControlPanel::ThresholdMode::Auto;
  }

  static bool thresholdIsManual(const HandControlPanel & panel)
  {
    return panel.threshold_mode_ == HandControlPanel::ThresholdMode::Manual;
  }

  static bool thresholdIsOff(const HandControlPanel & panel)
  {
    return panel.threshold_mode_ == HandControlPanel::ThresholdMode::Off;
  }

  static void setAutomaticThreshold(HandControlPanel & panel, double threshold)
  {
    panel.setAutomaticThreshold(threshold);
  }
};

}  // namespace wuji_rviz_panel

namespace
{

QApplication * ensureApplication()
{
  static int argc = 1;
  static char application_name[] = "wuji_rviz_panel_test";
  static char * argv[] = {application_name, nullptr};
  static QApplication application(argc, argv);
  return &application;
}

double linearPercentile(
  const std::vector<double> & sorted_values, double percentile)
{
  const double position =
    (percentile / 100.0) * static_cast<double>(sorted_values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = static_cast<size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted_values[lower] +
         fraction * (sorted_values[upper] - sorted_values[lower]);
}

void expectSameValuesIncludingNan(
  const std::vector<double> & actual, const std::vector<double> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t index = 0; index < expected.size(); ++index) {
    if (std::isnan(expected[index])) {
      EXPECT_TRUE(std::isnan(actual[index]));
    } else {
      EXPECT_DOUBLE_EQ(actual[index], expected[index]);
    }
  }
}

class DisplayControlsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_NE(ensureApplication(), nullptr);
    panel_ = std::make_unique<wuji_rviz_panel::HandControlPanel>();

    contact_sensitivity_ =
      panel_->findChild<QSlider *>("contact_sensitivity_slider");
    contact_threshold_ =
      panel_->findChild<QSlider *>("contact_threshold_slider");
    dynamic_sensitivity_ =
      panel_->findChild<QSlider *>("dynamic_sensitivity_slider");
    raw_vmax_ = panel_->findChild<QDoubleSpinBox *>("raw_vmax_internal");
    baseline_vmax_ =
      panel_->findChild<QDoubleSpinBox *>("baseline_vmax_internal");
    baseline_threshold_ =
      panel_->findChild<QDoubleSpinBox *>("baseline_threshold_internal");
    baseline_offset_ =
      panel_->findChild<QDoubleSpinBox *>("baseline_offset_internal");
    temporal_vmax_ =
      panel_->findChild<QDoubleSpinBox *>("temporal_vmax_internal");
    contact_threshold_value_ =
      panel_->findChild<QLabel *>("contact_threshold_value");
    baseline_state_ = panel_->findChild<QLabel *>("baseline_state_label");
    capture_baseline_ =
      panel_->findChild<QPushButton *>("capture_baseline_button");
    auto_threshold_ =
      panel_->findChild<QPushButton *>("auto_threshold_button");

    ASSERT_NE(contact_sensitivity_, nullptr);
    ASSERT_NE(contact_threshold_, nullptr);
    ASSERT_NE(dynamic_sensitivity_, nullptr);
    ASSERT_NE(raw_vmax_, nullptr);
    ASSERT_NE(baseline_vmax_, nullptr);
    ASSERT_NE(baseline_threshold_, nullptr);
    ASSERT_NE(baseline_offset_, nullptr);
    ASSERT_NE(temporal_vmax_, nullptr);
    ASSERT_NE(contact_threshold_value_, nullptr);
    ASSERT_NE(baseline_state_, nullptr);
    ASSERT_NE(capture_baseline_, nullptr);
    ASSERT_NE(auto_threshold_, nullptr);
  }

  std::unique_ptr<wuji_rviz_panel::HandControlPanel> panel_;
  QSlider * contact_sensitivity_{nullptr};
  QSlider * contact_threshold_{nullptr};
  QSlider * dynamic_sensitivity_{nullptr};
  QDoubleSpinBox * raw_vmax_{nullptr};
  QDoubleSpinBox * baseline_vmax_{nullptr};
  QDoubleSpinBox * baseline_threshold_{nullptr};
  QDoubleSpinBox * baseline_offset_{nullptr};
  QDoubleSpinBox * temporal_vmax_{nullptr};
  QLabel * contact_threshold_value_{nullptr};
  QLabel * baseline_state_{nullptr};
  QPushButton * capture_baseline_{nullptr};
  QPushButton * auto_threshold_{nullptr};
};

TEST_F(DisplayControlsTest, ShowsOnlyThreeSemanticControlsAndDefaults)
{
  EXPECT_EQ(panel_->findChildren<QSlider *>().size(), 3);
  EXPECT_TRUE(raw_vmax_->isHidden());
  EXPECT_TRUE(baseline_vmax_->isHidden());
  EXPECT_TRUE(baseline_threshold_->isHidden());
  EXPECT_TRUE(baseline_offset_->isHidden());
  EXPECT_TRUE(temporal_vmax_->isHidden());

  QStringList labels;
  for (const auto * label : panel_->findChildren<QLabel *>()) {
    labels.push_back(label->text());
  }
  EXPECT_TRUE(labels.contains(QString::fromUtf8("接触灵敏度")));
  EXPECT_TRUE(labels.contains(QString::fromUtf8("接触阈值")));
  EXPECT_TRUE(labels.contains(QString::fromUtf8("动态灵敏度")));
  EXPECT_FALSE(labels.contains("Raw vmax"));
  EXPECT_FALSE(labels.contains("Baseline vmax"));
  EXPECT_FALSE(labels.contains("Baseline offset"));
  EXPECT_FALSE(labels.contains("Baseline threshold"));
  EXPECT_FALSE(labels.contains("Temporal vmax"));

  bool found_display_settings = false;
  for (const auto * group : panel_->findChildren<QGroupBox *>()) {
    found_display_settings |= group->title() == QString::fromUtf8("显示设置");
  }
  EXPECT_TRUE(found_display_settings);

  EXPECT_DOUBLE_EQ(raw_vmax_->value(), 1.0);
  EXPECT_DOUBLE_EQ(raw_vmax_->minimum(), 1.0);
  EXPECT_DOUBLE_EQ(raw_vmax_->maximum(), 1.0);
  EXPECT_DOUBLE_EQ(baseline_vmax_->value(), 0.25);
  EXPECT_DOUBLE_EQ(baseline_vmax_->minimum(), 0.05);
  EXPECT_DOUBLE_EQ(baseline_vmax_->maximum(), 1.0);
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.10);
  EXPECT_DOUBLE_EQ(baseline_threshold_->minimum(), 0.0);
  EXPECT_DOUBLE_EQ(baseline_threshold_->maximum(), 1.0);
  EXPECT_DOUBLE_EQ(baseline_offset_->value(), 0.0);
  EXPECT_DOUBLE_EQ(baseline_offset_->minimum(), 0.0);
  EXPECT_DOUBLE_EQ(baseline_offset_->maximum(), 0.0);
  EXPECT_DOUBLE_EQ(temporal_vmax_->value(), 0.05);
  EXPECT_DOUBLE_EQ(temporal_vmax_->minimum(), 0.01);
  EXPECT_DOUBLE_EQ(temporal_vmax_->maximum(), 1.0);
  EXPECT_EQ(contact_threshold_value_->text(), QString::fromUtf8("0.100（手动）"));
  EXPECT_EQ(auto_threshold_->text(), QString::fromUtf8("自动估计"));

  EXPECT_TRUE(contact_sensitivity_->toolTip().contains(
      QString::fromUtf8("调高后更容易看到微弱的新增接触")));
  EXPECT_TRUE(contact_sensitivity_->toolTip().contains("[0,1]"));
  EXPECT_TRUE(contact_threshold_->toolTip().contains(
      QString::fromUtf8("关闭")));
  EXPECT_TRUE(dynamic_sensitivity_->toolTip().contains(
      QString::fromUtf8("接触、释放、滑动和冲击")));
  EXPECT_TRUE(dynamic_sensitivity_->toolTip().contains("[0,1]"));
}

TEST_F(DisplayControlsTest, SensitivitySlidersUseReverseLogMapping)
{
  EXPECT_EQ(contact_sensitivity_->minimum(), 0);
  EXPECT_EQ(contact_sensitivity_->maximum(), 10000);
  EXPECT_EQ(dynamic_sensitivity_->minimum(), 0);
  EXPECT_EQ(dynamic_sensitivity_->maximum(), 10000);

  contact_sensitivity_->setValue(0);
  EXPECT_DOUBLE_EQ(baseline_vmax_->value(), 1.0);
  const double baseline_low_sensitivity = baseline_vmax_->value();
  contact_sensitivity_->setValue(5000);
  const double baseline_mid_sensitivity = baseline_vmax_->value();
  contact_sensitivity_->setValue(10000);
  const double baseline_high_sensitivity = baseline_vmax_->value();
  EXPECT_GT(baseline_low_sensitivity, baseline_mid_sensitivity);
  EXPECT_GT(baseline_mid_sensitivity, baseline_high_sensitivity);
  EXPECT_DOUBLE_EQ(baseline_high_sensitivity, 0.05);

  dynamic_sensitivity_->setValue(0);
  EXPECT_DOUBLE_EQ(temporal_vmax_->value(), 1.0);
  const double temporal_low_sensitivity = temporal_vmax_->value();
  dynamic_sensitivity_->setValue(5000);
  const double temporal_mid_sensitivity = temporal_vmax_->value();
  dynamic_sensitivity_->setValue(10000);
  const double temporal_high_sensitivity = temporal_vmax_->value();
  EXPECT_GT(temporal_low_sensitivity, temporal_mid_sensitivity);
  EXPECT_GT(temporal_mid_sensitivity, temporal_high_sensitivity);
  EXPECT_DOUBLE_EQ(temporal_high_sensitivity, 0.01);

  constexpr double signal = 0.02;
  const double contact_neutral_display =
    std::clamp(signal / baseline_low_sensitivity, 0.0, 1.0);
  const double contact_sensitive_display =
    std::clamp(signal / baseline_high_sensitivity, 0.0, 1.0);
  EXPECT_GT(contact_sensitive_display, contact_neutral_display);
  const double temporal_neutral_display =
    std::clamp(signal / temporal_low_sensitivity, 0.0, 1.0);
  const double temporal_sensitive_display =
    std::clamp(signal / temporal_high_sensitivity, 0.0, 1.0);
  EXPECT_GT(temporal_sensitive_display, temporal_neutral_display);

  EXPECT_DOUBLE_EQ(raw_vmax_->value(), 1.0);
  EXPECT_DOUBLE_EQ(baseline_offset_->value(), 0.0);
}

TEST_F(DisplayControlsTest, ThresholdHasTrueOffBoundary)
{
  EXPECT_EQ(contact_threshold_->minimum(), 0);
  EXPECT_EQ(contact_threshold_->maximum(), 100000);

  contact_threshold_->setValue(0);
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.0);
  EXPECT_EQ(contact_threshold_value_->text(), QString::fromUtf8("关闭"));

  const double residuals[] = {0.0, 0.001, 0.02, 0.2};
  for (const double residual : residuals) {
    EXPECT_GE(residual, baseline_threshold_->value());
  }

  contact_threshold_->setValue(10000);
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.1);
  const auto active_at_point_one = std::count_if(
    std::begin(residuals), std::end(residuals),
    [this](double residual) {return residual >= baseline_threshold_->value();});
  contact_threshold_->setValue(50000);
  const auto active_at_point_five = std::count_if(
    std::begin(residuals), std::end(residuals),
    [this](double residual) {return residual >= baseline_threshold_->value();});
  EXPECT_GT(active_at_point_one, active_at_point_five);

  contact_threshold_->setValue(100000);
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 1.0);
  EXPECT_DOUBLE_EQ(raw_vmax_->value(), 1.0);
  EXPECT_DOUBLE_EQ(baseline_offset_->value(), 0.0);
}

TEST_F(DisplayControlsTest, CalibrationUsesIndependentFiveSecondPhasesAndP999)
{
  using Peer = wuji_rviz_panel::HandControlPanelTestPeer;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  Peer::apply(*panel_, 1, {0.2F, nan, 0.4F});
  capture_baseline_->click();
  EXPECT_TRUE(Peer::baselineCaptureActive(*panel_));
  EXPECT_FALSE(Peer::baselineReady(*panel_));

  // A fresh but entirely invalid frame starts wall-clock timing and does not
  // count toward the 300-valid-frame requirement.
  Peer::apply(*panel_, 2, {nan, nan, nan});
  EXPECT_EQ(Peer::baselineFrames(*panel_), 0U);
  Peer::makeBaselineDurationReady(*panel_);

  std::vector<Peer::Frame::SharedPtr> baseline_frames;
  for (uint32_t sample = 0; sample < 299; ++sample) {
    baseline_frames.push_back(Peer::frame(
        3 + sample,
        {sample < 150 ? 0.1F : 0.3F, nan, sample == 0 ? nan : 0.4F}));
  }
  Peer::apply(*panel_, baseline_frames);
  EXPECT_EQ(Peer::baselineFrames(*panel_), 299U);
  EXPECT_TRUE(Peer::baselineCaptureActive(*panel_));
  EXPECT_FALSE(Peer::baselineReady(*panel_));
  EXPECT_TRUE(baseline_state_->text().contains(
      QString::fromUtf8("正在等待足够有效数据")));

  Peer::apply(*panel_, 302, {0.3F, nan, 0.4F});
  ASSERT_TRUE(Peer::baselineReady(*panel_));
  EXPECT_FALSE(Peer::baselineCaptureActive(*panel_));
  EXPECT_TRUE(Peer::thresholdCaptureActive(*panel_));
  EXPECT_EQ(Peer::baselineFrames(*panel_), 300U);
  EXPECT_GE(Peer::baselineDuration(*panel_), 5.0);

  const std::vector<double> fixed_baseline = Peer::baseline(*panel_);
  ASSERT_EQ(fixed_baseline.size(), 3U);
  EXPECT_NEAR(fixed_baseline[0], 0.2, 1.0e-7);
  EXPECT_TRUE(std::isnan(fixed_baseline[1]));
  EXPECT_NEAR(fixed_baseline[2], 0.4, 1.0e-7);

  std::vector<double> expected_residuals;
  const auto threshold_frame = [&](uint32_t sequence, uint32_t sample) {
      const float raw_0 = static_cast<float>(
        fixed_baseline[0] + static_cast<double>(2 * sample) / 1000.0);
      const float raw_2 = static_cast<float>(
        fixed_baseline[2] + static_cast<double>(2 * sample + 1) / 1000.0);
      expected_residuals.push_back(
        std::max(static_cast<double>(raw_0) - fixed_baseline[0], 0.0));
      expected_residuals.push_back(
        std::max(static_cast<double>(raw_2) - fixed_baseline[2], 0.0));
      return Peer::frame(sequence, {raw_0, nan, raw_2});
    };

  Peer::apply(*panel_, {threshold_frame(303, 0)});
  Peer::makeThresholdDurationReady(*panel_);
  std::vector<Peer::Frame::SharedPtr> threshold_frames;
  for (uint32_t sample = 1; sample < 299; ++sample) {
    threshold_frames.push_back(threshold_frame(303 + sample, sample));
  }
  Peer::apply(*panel_, threshold_frames);
  EXPECT_EQ(Peer::thresholdFrames(*panel_), 299U);
  EXPECT_TRUE(Peer::thresholdCaptureActive(*panel_));
  ASSERT_EQ(Peer::baseline(*panel_).size(), fixed_baseline.size());
  EXPECT_DOUBLE_EQ(Peer::baseline(*panel_)[0], fixed_baseline[0]);
  EXPECT_TRUE(std::isnan(Peer::baseline(*panel_)[1]));
  EXPECT_DOUBLE_EQ(Peer::baseline(*panel_)[2], fixed_baseline[2]);

  Peer::apply(*panel_, {threshold_frame(602, 299)});
  EXPECT_FALSE(Peer::thresholdCaptureActive(*panel_));
  EXPECT_EQ(Peer::thresholdFrames(*panel_), 300U);
  EXPECT_GE(Peer::thresholdDuration(*panel_), 5.0);
  EXPECT_TRUE(Peer::thresholdIsAuto(*panel_));

  std::sort(expected_residuals.begin(), expected_residuals.end());
  ASSERT_EQ(expected_residuals.size(), 600U);
  const double expected_median = linearPercentile(expected_residuals, 50.0);
  const double expected_p99 = linearPercentile(expected_residuals, 99.0);
  const double expected_p999 = linearPercentile(expected_residuals, 99.9);
  EXPECT_NEAR(Peer::residualMedian(*panel_), expected_median, 1.0e-12);
  EXPECT_NEAR(Peer::residualP99(*panel_), expected_p99, 1.0e-12);
  EXPECT_NEAR(Peer::residualP999(*panel_), expected_p999, 1.0e-12);
  EXPECT_DOUBLE_EQ(Peer::residualMaximum(*panel_), expected_residuals.back());
  EXPECT_LT(Peer::residualP999(*panel_), Peer::residualMaximum(*panel_));
  EXPECT_NEAR(Peer::autoThreshold(*panel_), expected_p999, 1.0e-12);
  EXPECT_NEAR(baseline_threshold_->value(), expected_p999, 5.0e-10);
  EXPECT_TRUE(contact_threshold_value_->text().contains(
      QString::fromUtf8("自动")));
  EXPECT_TRUE(baseline_state_->text().contains(
      "Baseline noise is unusually high"));

  const std::vector<double> raw_before_controls = Peer::raw(*panel_);
  const std::vector<double> baseline_before_controls = Peer::baseline(*panel_);
  const std::vector<double> temporal_before_controls = Peer::temporal(*panel_);
  std::vector<double> residual_before_controls(raw_before_controls.size());
  for (size_t index = 0; index < raw_before_controls.size(); ++index) {
    residual_before_controls[index] =
      std::isfinite(raw_before_controls[index]) &&
      std::isfinite(baseline_before_controls[index]) ?
      std::max(raw_before_controls[index] - baseline_before_controls[index], 0.0) :
      std::numeric_limits<double>::quiet_NaN();
  }
  contact_sensitivity_->setValue(10000);
  dynamic_sensitivity_->setValue(10000);
  contact_threshold_->setValue(2000);
  expectSameValuesIncludingNan(Peer::raw(*panel_), raw_before_controls);
  expectSameValuesIncludingNan(Peer::temporal(*panel_), temporal_before_controls);
  ASSERT_EQ(Peer::baseline(*panel_).size(), baseline_before_controls.size());
  for (size_t index = 0; index < baseline_before_controls.size(); ++index) {
    if (std::isnan(baseline_before_controls[index])) {
      EXPECT_TRUE(std::isnan(Peer::baseline(*panel_)[index]));
      EXPECT_TRUE(std::isnan(residual_before_controls[index]));
    } else {
      EXPECT_DOUBLE_EQ(Peer::baseline(*panel_)[index], baseline_before_controls[index]);
      EXPECT_DOUBLE_EQ(
        std::max(Peer::raw(*panel_)[index] - Peer::baseline(*panel_)[index], 0.0),
        residual_before_controls[index]);
    }
  }
  EXPECT_TRUE(Peer::thresholdIsManual(*panel_));
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.02);
  EXPECT_TRUE(contact_threshold_value_->text().contains(
      QString::fromUtf8("手动")));
  Peer::apply(*panel_, 603, {0.3F, nan, 0.5F});
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.02);
  EXPECT_TRUE(Peer::thresholdIsManual(*panel_));

  contact_threshold_->setValue(0);
  EXPECT_TRUE(Peer::thresholdIsOff(*panel_));
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.0);
  EXPECT_EQ(contact_threshold_value_->text(), QString::fromUtf8("关闭"));
  const double nonnegative_residuals[] = {0.0, 0.001, 0.1, 1.0};
  for (double residual : nonnegative_residuals) {
    EXPECT_GE(residual, baseline_threshold_->value());
  }

  auto_threshold_->click();
  EXPECT_TRUE(Peer::thresholdCaptureActive(*panel_));
  Peer::apply(*panel_, 604, {
      static_cast<float>(fixed_baseline[0] + 0.01), nan,
      static_cast<float>(fixed_baseline[2] + 0.01)});
  Peer::makeThresholdDurationReady(*panel_);
  std::vector<Peer::Frame::SharedPtr> recapture_frames;
  for (uint32_t sequence = 605; sequence < 903; ++sequence) {
    recapture_frames.push_back(Peer::frame(
        sequence, {
          static_cast<float>(fixed_baseline[0] + 0.01), nan,
          static_cast<float>(fixed_baseline[2] + 0.01)}));
  }
  Peer::apply(*panel_, recapture_frames);
  EXPECT_EQ(Peer::thresholdFrames(*panel_), 299U);
  EXPECT_TRUE(Peer::thresholdCaptureActive(*panel_));
  Peer::apply(*panel_, 903, {
      static_cast<float>(fixed_baseline[0] + 0.01), nan,
      static_cast<float>(fixed_baseline[2] + 0.01)});
  EXPECT_FALSE(Peer::thresholdCaptureActive(*panel_));
  EXPECT_TRUE(Peer::thresholdIsAuto(*panel_));
  EXPECT_TRUE(contact_threshold_value_->text().contains(
      QString::fromUtf8("自动")));
  EXPECT_DOUBLE_EQ(Peer::baseline(*panel_)[0], fixed_baseline[0]);
  EXPECT_DOUBLE_EQ(Peer::baseline(*panel_)[2], fixed_baseline[2]);
}

TEST_F(DisplayControlsTest, AutoEstimateRequiresAValidBaseline)
{
  using Peer = wuji_rviz_panel::HandControlPanelTestPeer;
  Peer::apply(*panel_, 1, {0.1F});
  auto_threshold_->click();
  EXPECT_FALSE(Peer::thresholdCaptureActive(*panel_));
  EXPECT_TRUE(baseline_state_->text().contains(
      QString::fromUtf8("请先执行 Capture Baseline")));
}

TEST_F(DisplayControlsTest, AutomaticThresholdClipsOnlyToSdkRange)
{
  using Peer = wuji_rviz_panel::HandControlPanelTestPeer;
  Peer::setAutomaticThreshold(*panel_, 1.2);
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 1.0);
  EXPECT_TRUE(Peer::thresholdIsAuto(*panel_));

  Peer::setAutomaticThreshold(*panel_, -0.1);
  EXPECT_DOUBLE_EQ(baseline_threshold_->value(), 0.0);
  EXPECT_TRUE(Peer::thresholdIsAuto(*panel_));
  EXPECT_NE(contact_threshold_value_->text(), QString::fromUtf8("关闭"));

  ASSERT_TRUE(QMetaObject::invokeMethod(
      contact_threshold_, "sliderPressed", Qt::DirectConnection));
  EXPECT_TRUE(Peer::thresholdIsOff(*panel_));
  EXPECT_EQ(contact_threshold_value_->text(), QString::fromUtf8("关闭"));
}

TEST(HeatmapNormalizationTest, DoesNotMutateValuesMasksOrNan)
{
  ASSERT_NE(ensureApplication(), nullptr);
  wuji_rviz_panel::HeatmapWidget heatmap;
  std::vector<double> values{
    0.0, 0.02, std::numeric_limits<double>::quiet_NaN(), 1.0};
  std::vector<uint8_t> active{1, 0, 1, 1};
  const std::vector<double> original_values = values;
  const std::vector<uint8_t> original_active = active;

  heatmap.setGrid(2, 2, values, active, 0.05);
  EXPECT_DOUBLE_EQ(values[0], original_values[0]);
  EXPECT_DOUBLE_EQ(values[1], original_values[1]);
  EXPECT_TRUE(std::isnan(values[2]));
  EXPECT_TRUE(std::isnan(original_values[2]));
  EXPECT_DOUBLE_EQ(values[3], original_values[3]);
  EXPECT_EQ(active, original_active);

  heatmap.setGrid(2, 2, values, active, 1.0);
  EXPECT_DOUBLE_EQ(values[0], original_values[0]);
  EXPECT_DOUBLE_EQ(values[1], original_values[1]);
  EXPECT_TRUE(std::isnan(values[2]));
  EXPECT_DOUBLE_EQ(values[3], original_values[3]);
  EXPECT_EQ(active, original_active);
}
}  // namespace
