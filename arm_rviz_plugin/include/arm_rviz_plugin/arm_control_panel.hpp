#ifndef ARM_CONTROL_PANEL_HPP
#define ARM_CONTROL_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QCheckBox>
#include <QTimer>
#include <deque>
#include <memory>

namespace arm_rviz_plugin
{

class ArmControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ArmControlPanel(QWidget* parent = nullptr);
  virtual ~ArmControlPanel();

  // Load/Save configuration
  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void onScanFrontClicked();
  void onScanAllClicked();
  void onScanAndGraspClicked();
  void onGraspLiftClicked();
  void onDeliverPoseClicked();
  void onVoiceControlToggled(bool checked);
  void onShowDetailsToggled(bool checked);
  void spinNode();

protected:
  void setupUI();
  void setupROS();
  void commandResultCallback(const std_msgs::msg::String::SharedPtr msg);
  void addResultMessage(const QString& message);
  QString generateResultSummary(const QString& json_result);

  // UI Components
  QPushButton* scan_front_btn_;
  QPushButton* scan_all_btn_;
  QPushButton* scan_and_grasp_btn_;
  QPushButton* grasp_lift_btn_;
  QPushButton* deliver_pose_btn_;
  QCheckBox* voice_control_checkbox_;
  QCheckBox* show_details_checkbox_;
  QTextEdit* result_display_;
  QLabel* status_label_;

  // ROS2 Components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  QTimer* spin_timer_;
  
  // Message history
  std::deque<QString> message_history_;
  const size_t max_messages_ = 10;
  
  // Voice control state
  bool voice_control_enabled_;
  
  // Show details state
  bool show_details_enabled_;
};

}  // namespace arm_rviz_plugin

#endif  // ARM_CONTROL_PANEL_HPP
