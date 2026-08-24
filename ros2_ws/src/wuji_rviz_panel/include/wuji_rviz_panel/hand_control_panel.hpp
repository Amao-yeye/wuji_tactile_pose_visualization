#ifndef WUJI_RVIZ_PANEL__HAND_CONTROL_PANEL_HPP_
#define WUJI_RVIZ_PANEL__HAND_CONTROL_PANEL_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QColor>
#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QSize>
#include <QString>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/empty.hpp>
#include <wuji_tactile_msgs/msg/hand_control_status.hpp>
#include <wuji_tactile_msgs/msg/tactile_pressure_frame.hpp>
#include <wuji_tactile_msgs/srv/hand_control_command.hpp>

class QComboBox;
class QDoubleSpinBox;
class QMouseEvent;
class QPaintEvent;
class QShowEvent;
class QTimer;

namespace wuji_rviz_panel
{

class HeatmapWidget : public QWidget
{
public:
  explicit HeatmapWidget(QWidget * parent = nullptr);

  void setGrid(
    uint32_t rows, uint32_t cols, const std::vector<double> & values,
    const std::vector<uint8_t> & active, double maximum, const QString & overlay = QString());
  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;

private:
  static QColor infernoColor(double value, double maximum);

  uint32_t rows_{0};
  uint32_t cols_{0};
  std::vector<double> values_;
  QImage heatmap_image_;
  QString overlay_;
};

class HandControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit HandControlPanel(QWidget * parent = nullptr);
  ~HandControlPanel() override;

  void onInitialize() override;

protected:
  void showEvent(QShowEvent * event) override;

private:
  using TactileFrame = wuji_tactile_msgs::msg::TactilePressureFrame;
  using ControlStatus = wuji_tactile_msgs::msg::HandControlStatus;
  using ControlCommand = wuji_tactile_msgs::srv::HandControlCommand;

  void buildUi();
  void connectUi();
  void onPoseSelectionChanged(int index);
  void requestSelectedPose();
  void requestPose(const QString & pose_id, const QString & display_name);
  void onTactileFrame(const TactileFrame::SharedPtr message);
  void onControlStatus(const ControlStatus::SharedPtr message);
  void processLatestTactileFrame();
  void scheduleRvizLayoutCheck();
  void waitForStableRvizLayout();
  size_t applyTactileFrames(const std::deque<TactileFrame::SharedPtr> & messages);
  void applyControlStatus(const ControlStatus & message);
  void renderLatest();
  void refreshControlButtons();
  void statusWatchTick();

  void startBaselineCapture();
  void resetBaseline();
  void advanceBaselineCapture();
  void clearBaselineCapture();

  void sendAction(
    uint8_t action, const QString & label, const QString & pose_id = QString());
  void requestDisableBestEffort();
  void publishHeartbeat();

  static double median(std::vector<double> values);
  static QString stateName(uint8_t state);

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_abstraction_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<TactileFrame>::SharedPtr tactile_subscription_;
  rclcpp::CallbackGroup::SharedPtr tactile_callback_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> tactile_executor_;
  std::thread tactile_executor_thread_;
  rclcpp::Subscription<ControlStatus>::SharedPtr status_subscription_;
  rclcpp::Client<ControlCommand>::SharedPtr command_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher_;

  QString hand_name_{"hand_0"};
  QString selected_pose_id_{"relaxed"};
  bool have_status_{false};
  ControlStatus latest_status_;
  QElapsedTimer status_age_;
  QElapsedTimer diagnostics_age_;
  QTimer * tactile_render_timer_{nullptr};
  QTimer * heartbeat_timer_{nullptr};
  QTimer * status_watch_timer_{nullptr};
  bool panel_shown_{false};
  bool layout_check_scheduled_{false};
  bool layout_stabilized_{false};
  int layout_stabilization_attempts_{0};
  int layout_stable_sample_count_{0};
  QString previous_layout_geometry_;

  std::mutex pending_tactile_mutex_;
  std::deque<TactileFrame::SharedPtr> pending_tactile_frames_;
  std::atomic<uint64_t> backend_sequence_advance_count_{0};
  std::atomic<uint64_t> tactile_callback_count_{0};
  std::atomic<uint64_t> tactile_callback_thread_id_{0};
  std::atomic<uint64_t> tactile_buffer_drop_count_{0};
  uint64_t gui_timer_tick_count_{0};
  uint64_t gui_refresh_count_{0};
  uint64_t gui_timer_thread_id_{0};
  uint64_t processed_tactile_frame_count_{0};
  uint64_t gui_max_batch_frame_count_{0};
  uint64_t actual_render_count_{0};
  uint64_t actual_render_thread_id_{0};
  uint64_t diagnostics_last_backend_sequence_advance_count_{0};
  uint64_t diagnostics_last_callback_count_{0};
  uint64_t diagnostics_last_processed_tactile_frame_count_{0};
  uint64_t diagnostics_last_buffer_drop_count_{0};
  uint64_t diagnostics_last_gui_timer_tick_count_{0};
  uint64_t diagnostics_last_gui_refresh_count_{0};
  uint64_t diagnostics_last_actual_render_count_{0};
  bool have_backend_sequence_{false};
  uint32_t backend_last_sequence_{0};
  bool have_frame_{false};
  bool have_sequence_{false};
  uint32_t latest_sequence_{0};
  uint32_t rows_{0};
  uint32_t cols_{0};
  std::vector<double> raw_pressure_;
  std::vector<double> previous_raw_;
  std::vector<double> temporal_signed_delta_;
  std::vector<double> temporal_peak_absolute_delta_;
  size_t temporal_batch_frame_count_{0};
  bool temporal_ready_{false};

  std::vector<double> baseline_;
  std::vector<uint8_t> baseline_valid_;
  bool baseline_ready_{false};
  bool baseline_capture_active_{false};
  bool baseline_capture_has_first_sample_{false};
  uint32_t baseline_capture_last_sequence_{0};
  std::chrono::steady_clock::time_point baseline_capture_first_sample_;
  std::vector<std::vector<double>> baseline_samples_;
  size_t baseline_capture_frame_count_{0};

  QLabel * connection_label_{nullptr};
  QLabel * layout_label_{nullptr};
  QLabel * raw_stats_label_{nullptr};
  QLabel * baseline_stats_label_{nullptr};
  QLabel * temporal_stats_label_{nullptr};
  QLabel * baseline_state_label_{nullptr};
  QLabel * request_result_label_{nullptr};

  HeatmapWidget * raw_heatmap_{nullptr};
  HeatmapWidget * baseline_heatmap_{nullptr};
  HeatmapWidget * temporal_heatmap_{nullptr};

  QDoubleSpinBox * raw_vmax_{nullptr};
  QDoubleSpinBox * baseline_vmax_{nullptr};
  QDoubleSpinBox * baseline_threshold_{nullptr};
  QDoubleSpinBox * baseline_offset_{nullptr};
  QDoubleSpinBox * temporal_vmax_{nullptr};

  QPushButton * capture_baseline_button_{nullptr};
  QPushButton * reset_baseline_button_{nullptr};
  QComboBox * pose_selector_{nullptr};
  QPushButton * move_pose_button_{nullptr};
  QPushButton * open_button_{nullptr};
  QPushButton * enable_button_{nullptr};
  QPushButton * disable_button_{nullptr};
};

}  // namespace wuji_rviz_panel

#endif  // WUJI_RVIZ_PANEL__HAND_CONTROL_PANEL_HPP_
