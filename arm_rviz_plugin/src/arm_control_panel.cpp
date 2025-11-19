#include "../include/arm_rviz_plugin/arm_control_panel.hpp"

#include <QGroupBox>
#include <QDateTime>
#include <pluginlib/class_list_macros.hpp>

namespace arm_rviz_plugin
{

ArmControlPanel::ArmControlPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , voice_control_enabled_(false)
  , show_details_enabled_(true)
{
  setupUI();
  setupROS();
}

ArmControlPanel::~ArmControlPanel()
{
}

void ArmControlPanel::setupUI()
{
  QVBoxLayout* main_layout = new QVBoxLayout;
  
  // 去掉标题，节省空间
  // QLabel* title = new QLabel("<h2>Arm Control Panel</h2>");
  // title->setAlignment(Qt::AlignCenter);
  // main_layout->addWidget(title);
  
  // ========== 扫描按钮区 ==========
  QGroupBox* scan_group = new QGroupBox("Scan Commands");
  QHBoxLayout* scan_layout = new QHBoxLayout;  // 改为水平布局（按列排列）
  
  scan_front_btn_ = new QPushButton("Scan Front");
  scan_front_btn_->setMinimumHeight(35);  // 稍微减小高度
  scan_front_btn_->setStyleSheet(
    "QPushButton { background-color: #4CAF50; color: white; font-size: 13px; font-weight: bold; border-radius: 5px; }"
    "QPushButton:hover { background-color: #45a049; }"
    "QPushButton:pressed { background-color: #3d8b40; }"
  );
  connect(scan_front_btn_, &QPushButton::clicked, this, &ArmControlPanel::onScanFrontClicked);
  scan_layout->addWidget(scan_front_btn_);
  
  scan_all_btn_ = new QPushButton("Scan All");
  scan_all_btn_->setMinimumHeight(35);  // 稍微减小高度
  scan_all_btn_->setStyleSheet(
    "QPushButton { background-color: #2196F3; color: white; font-size: 13px; font-weight: bold; border-radius: 5px; }"
    "QPushButton:hover { background-color: #0b7dda; }"
    "QPushButton:pressed { background-color: #0a6bc5; }"
  );
  connect(scan_all_btn_, &QPushButton::clicked, this, &ArmControlPanel::onScanAllClicked);
  scan_layout->addWidget(scan_all_btn_);
  
  scan_group->setLayout(scan_layout);
  main_layout->addWidget(scan_group);
  
  // ========== 动作按钮区 ==========
  QGroupBox* action_group = new QGroupBox("Action Commands");
  QHBoxLayout* action_layout = new QHBoxLayout;  // 改为水平布局（按列排列）
  
  grasp_lift_btn_ = new QPushButton("Grasp & Lift");
  grasp_lift_btn_->setMinimumHeight(35);  // 稍微减小高度
  grasp_lift_btn_->setStyleSheet(
    "QPushButton { background-color: #FF5722; color: white; font-size: 13px; font-weight: bold; border-radius: 5px; }"
    "QPushButton:hover { background-color: #E64A19; }"
    "QPushButton:pressed { background-color: #D84315; }"
  );
  connect(grasp_lift_btn_, &QPushButton::clicked, this, &ArmControlPanel::onGraspLiftClicked);
  action_layout->addWidget(grasp_lift_btn_);
  
  deliver_pose_btn_ = new QPushButton("Deliver to Pose");
  deliver_pose_btn_->setMinimumHeight(35);  // 稍微减小高度
  deliver_pose_btn_->setStyleSheet(
    "QPushButton { background-color: #00BCD4; color: white; font-size: 13px; font-weight: bold; border-radius: 5px; }"
    "QPushButton:hover { background-color: #00ACC1; }"
    "QPushButton:pressed { background-color: #0097A7; }"
  );
  connect(deliver_pose_btn_, &QPushButton::clicked, this, &ArmControlPanel::onDeliverPoseClicked);
  action_layout->addWidget(deliver_pose_btn_);
  
  scan_and_grasp_btn_ = new QPushButton("Scan & Grasp");
  scan_and_grasp_btn_->setMinimumHeight(35);  // 稍微减小高度
  scan_and_grasp_btn_->setStyleSheet(
    "QPushButton { background-color: #FF9800; color: white; font-size: 13px; font-weight: bold; border-radius: 5px; }"
    "QPushButton:hover { background-color: #e68900; }"
    "QPushButton:pressed { background-color: #cc7a00; }"
  );
  connect(scan_and_grasp_btn_, &QPushButton::clicked, this, &ArmControlPanel::onScanAndGraspClicked);
  action_layout->addWidget(scan_and_grasp_btn_);
  
  action_group->setLayout(action_layout);
  main_layout->addWidget(action_group);
  
  // Voice control group
  QGroupBox* voice_group = new QGroupBox("Voice Control");
  QHBoxLayout* voice_layout = new QHBoxLayout;
  
  voice_control_checkbox_ = new QCheckBox("Enable Voice Control");
  voice_control_checkbox_->setStyleSheet("QCheckBox { font-size: 14px; font-weight: bold; }");
  connect(voice_control_checkbox_, &QCheckBox::toggled, this, &ArmControlPanel::onVoiceControlToggled);
  voice_layout->addWidget(voice_control_checkbox_);
  
  voice_layout->addStretch();
  voice_group->setLayout(voice_layout);
  main_layout->addWidget(voice_group);
  
  // Display options group
  QGroupBox* display_group = new QGroupBox("Display Options");
  QHBoxLayout* display_layout = new QHBoxLayout;
  
  show_details_checkbox_ = new QCheckBox("Show Details");
  show_details_checkbox_->setChecked(false);  // 默认不勾选
  show_details_checkbox_->setStyleSheet("QCheckBox { font-size: 14px; font-weight: bold; }");
  connect(show_details_checkbox_, &QCheckBox::toggled, this, &ArmControlPanel::onShowDetailsToggled);
  display_layout->addWidget(show_details_checkbox_);
  
  display_layout->addStretch();
  display_group->setLayout(display_layout);
  main_layout->addWidget(display_group);
  
  // Status label
  status_label_ = new QLabel("Status: Ready");
  status_label_->setStyleSheet("QLabel { color: green; font-size: 12px; font-weight: bold; padding: 5px; }");
  main_layout->addWidget(status_label_);
  
  // Result display group
  QGroupBox* result_group = new QGroupBox("Execution Results (Last 10)");
  QVBoxLayout* result_layout = new QVBoxLayout;
  
  result_display_ = new QTextEdit();
  result_display_->setReadOnly(true);
  result_display_->setMinimumHeight(200);
  result_display_->setStyleSheet(
    "QTextEdit { background-color: #f5f5f5; font-family: monospace; font-size: 11px; }"
  );
  result_layout->addWidget(result_display_);
  
  result_group->setLayout(result_layout);
  main_layout->addWidget(result_group);
  
  main_layout->addStretch();
  setLayout(main_layout);
}

void ArmControlPanel::setupROS()
{
  // Create ROS2 node
  node_ = std::make_shared<rclcpp::Node>("arm_control_panel_node");
  
  // Create publisher for arm commands
  command_pub_ = node_->create_publisher<std_msgs::msg::String>("/arm_command", 10);
  
  // Create subscriber for command results
  result_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/arm_command_result",
    10,
    std::bind(&ArmControlPanel::commandResultCallback, this, std::placeholders::_1)
  );
  
  // Create a timer to spin the node and process callbacks
  // This is critical for RViz plugins to receive messages!
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &ArmControlPanel::spinNode);
  spin_timer_->start(100);  // Spin every 100ms
  
  RCLCPP_INFO(node_->get_logger(), "Arm Control Panel initialized");
  addResultMessage("✓ Control panel initialized");
}

void ArmControlPanel::onScanFrontClicked()
{
  auto msg = std_msgs::msg::String();
  msg.data = "scan_front";
  command_pub_->publish(msg);
  
  status_label_->setText("Status: Executing scan front...");
  status_label_->setStyleSheet("QLabel { color: orange; font-size: 12px; font-weight: bold; padding: 5px; }");
  
  RCLCPP_INFO(node_->get_logger(), "Published command: scan_front");
  addResultMessage("→ Sent command: scan_front");
}

void ArmControlPanel::onScanAllClicked()
{
  auto msg = std_msgs::msg::String();
  msg.data = "scan_all";
  command_pub_->publish(msg);
  
  status_label_->setText("Status: Executing scan all...");
  status_label_->setStyleSheet("QLabel { color: orange; font-size: 12px; font-weight: bold; padding: 5px; }");
  
  RCLCPP_INFO(node_->get_logger(), "Published command: scan_all");
  addResultMessage("→ Sent command: scan_all");
}

void ArmControlPanel::onScanAndGraspClicked()
{
  auto msg = std_msgs::msg::String();
  msg.data = "scan_and_grasp";
  command_pub_->publish(msg);
  
  status_label_->setText("Status: Executing scan and grasp...");
  status_label_->setStyleSheet("QLabel { color: orange; font-size: 12px; font-weight: bold; padding: 5px; }");
  
  RCLCPP_INFO(node_->get_logger(), "Published command: scan_and_grasp");
  addResultMessage("→ Sent command: scan_and_grasp");
}

void ArmControlPanel::onGraspLiftClicked()
{
  auto msg = std_msgs::msg::String();
  msg.data = "grasp_lift";
  command_pub_->publish(msg);
  
  status_label_->setText("Status: Executing grasp & lift...");
  status_label_->setStyleSheet("QLabel { color: #FF5722; font-size: 12px; font-weight: bold; padding: 5px; }");
  
  RCLCPP_INFO(node_->get_logger(), "Published command: grasp_lift");
  addResultMessage("→ Sent command: grasp_lift");
}

void ArmControlPanel::onDeliverPoseClicked()
{
  auto msg = std_msgs::msg::String();
  msg.data = "deliver_pose";
  command_pub_->publish(msg);
  
  status_label_->setText("Status: Executing deliver to pose...");
  status_label_->setStyleSheet("QLabel { color: #00BCD4; font-size: 12px; font-weight: bold; padding: 5px; }");
  
  RCLCPP_INFO(node_->get_logger(), "Published command: deliver_pose");
  addResultMessage("→ Sent command: deliver_pose");
}

void ArmControlPanel::onVoiceControlToggled(bool checked)
{
  voice_control_enabled_ = checked;
  
  if (checked) {
    status_label_->setText("Status: Voice control enabled");
    status_label_->setStyleSheet("QLabel { color: blue; font-size: 12px; font-weight: bold; padding: 5px; }");
    RCLCPP_INFO(node_->get_logger(), "Voice control enabled");
    addResultMessage("🎤 Voice control enabled");
    
    // Note: The arm_voice_node should already be running separately
    // This checkbox just indicates the user wants to use voice control
    addResultMessage("Note: Please ensure arm_voice_node is running");
  } else {
    status_label_->setText("Status: Voice control disabled");
    status_label_->setStyleSheet("QLabel { color: gray; font-size: 12px; font-weight: bold; padding: 5px; }");
    RCLCPP_INFO(node_->get_logger(), "Voice control disabled");
    addResultMessage("🔇 Voice control disabled");
  }
}

void ArmControlPanel::onShowDetailsToggled(bool checked)
{
  show_details_enabled_ = checked;
  
  if (checked) {
    RCLCPP_INFO(node_->get_logger(), "Show details enabled");
    addResultMessage("👁️ Details display enabled");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Show details disabled");
    addResultMessage("🔒 Details display disabled (showing summaries only)");
  }
}

void ArmControlPanel::spinNode()
{
  // Process callbacks for this node
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void ArmControlPanel::commandResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
  QString result = QString::fromStdString(msg->data);
  
  // 提取phase和status来更新状态栏
  QString phase = "unknown";
  QString status = "unknown";
  
  int phaseStart = result.indexOf("\"phase\"");
  if (phaseStart != -1) {
    int colonPos = result.indexOf(":", phaseStart);
    int quoteStart = result.indexOf("\"", colonPos);
    int quoteEnd = result.indexOf("\"", quoteStart + 1);
    if (quoteStart != -1 && quoteEnd != -1) {
      phase = result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
    }
  }
  
  int statusStart = result.indexOf("\"status\"");
  if (statusStart != -1) {
    int colonPos = result.indexOf(":", statusStart);
    int quoteStart = result.indexOf("\"", colonPos);
    int quoteEnd = result.indexOf("\"", quoteStart + 1);
    if (quoteStart != -1 && quoteEnd != -1) {
      status = result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
    }
  }
  
  // 根据状态更新状态栏
  if (status == "error") {
    status_label_->setText(QString("Status: Error - %1").arg(phase));
    status_label_->setStyleSheet("QLabel { color: red; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else if (status == "ok") {
    // 检测成功 (用于 scan_planar 等命令)
    status_label_->setText("Status: Detection completed");
    status_label_->setStyleSheet("QLabel { color: green; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else if (status == "no_detection") {
    status_label_->setText("Status: No objects detected");
    status_label_->setStyleSheet("QLabel { color: orange; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else if (status == "scan_complete") {
    status_label_->setText("Status: Scan completed, preparing to grasp...");
    status_label_->setStyleSheet("QLabel { color: blue; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else if (status == "grasp_complete") {
    status_label_->setText("Status: Grasp completed, waiting for hand...");
    status_label_->setStyleSheet("QLabel { color: blue; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else if (status == "success") {
    status_label_->setText(QString("Status: Command completed successfully - %1").arg(phase));
    status_label_->setStyleSheet("QLabel { color: green; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else if (phase != "unknown") {
    // 有phase但status未知的情况
    status_label_->setText(QString("Status: %1 - %2").arg(status).arg(phase));
    status_label_->setStyleSheet("QLabel { color: orange; font-size: 12px; font-weight: bold; padding: 5px; }");
  } else {
    // status和phase都未识别
    status_label_->setText(QString("Status: %1").arg(status));
    status_label_->setStyleSheet("QLabel { color: orange; font-size: 12px; font-weight: bold; padding: 5px; }");
  }
  
  RCLCPP_INFO(node_->get_logger(), "Received result: %s", msg->data.c_str());
  
  // 尝试解析JSON并生成简洁摘要
  QString summary = generateResultSummary(result);
  addResultMessage("← " + summary);
  
  // 添加详细信息(仅当勾选了 Show Details 时显示)
  if (show_details_enabled_) {
    addResultMessage("   Details: " + result);
  }
}

void ArmControlPanel::addResultMessage(const QString& message)
{
  // Add timestamp
  QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
  QString formatted_message = QString("[%1] %2").arg(timestamp).arg(message);
  
  // Add to history
  message_history_.push_front(formatted_message);
  
  // Keep only last 10 messages
  while (message_history_.size() > max_messages_) {
    message_history_.pop_back();
  }
  
  // Update display
  QString display_text;
  for (const auto& msg : message_history_) {
    display_text += msg + "\n";
  }
  result_display_->setPlainText(display_text);
  
  // Scroll to top to show latest message
  QTextCursor cursor = result_display_->textCursor();
  cursor.movePosition(QTextCursor::Start);
  result_display_->setTextCursor(cursor);
}

void ArmControlPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  
  bool voice_enabled;
  if (config.mapGetBool("voice_control_enabled", &voice_enabled)) {
    voice_control_checkbox_->setChecked(voice_enabled);
  }
  
  bool show_details;
  if (config.mapGetBool("show_details_enabled", &show_details)) {
    show_details_checkbox_->setChecked(show_details);
  }
}

void ArmControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("voice_control_enabled", voice_control_enabled_);
  config.mapSetValue("show_details_enabled", show_details_enabled_);
}

QString ArmControlPanel::generateResultSummary(const QString& json_result)
{
  // 简单的JSON解析来提取关键信息
  // 格式: 📊 Command: scan_front | Status: ok | Phase: scan | Success: true | Objects: 2 (mouse, bottle)
  
  QString summary;
  
  // 提取command
  QString command = "unknown";
  int cmdStart = json_result.indexOf("\"command\"");
  if (cmdStart != -1) {
    int colonPos = json_result.indexOf(":", cmdStart);
    int quoteStart = json_result.indexOf("\"", colonPos);
    int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
    if (quoteStart != -1 && quoteEnd != -1) {
      command = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
    }
  }
  
  // 提取status
  QString status = "unknown";
  int statusStart = json_result.indexOf("\"status\"");
  if (statusStart != -1) {
    int colonPos = json_result.indexOf(":", statusStart);
    int quoteStart = json_result.indexOf("\"", colonPos);
    int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
    if (quoteStart != -1 && quoteEnd != -1) {
      status = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
    }
  }
  
  // 提取phase
  QString phase = "";
  int phaseStart = json_result.indexOf("\"phase\"");
  if (phaseStart != -1) {
    int colonPos = json_result.indexOf(":", phaseStart);
    int quoteStart = json_result.indexOf("\"", colonPos);
    int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
    if (quoteStart != -1 && quoteEnd != -1) {
      phase = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
    }
  }
  
  // 提取success (优先使用顶层的 "success" 字段)
  // 注意: success 可能是布尔值 (true/false) 或字符串 ("true"/"false")
  QString success = "unknown";
  int successStart = json_result.indexOf("\"success\"");
  if (successStart != -1) {
    int colonPos = json_result.indexOf(":", successStart);
    // 跳过冒号后的空格，找到实际值
    int valueStart = colonPos + 1;
    while (valueStart < json_result.length() && (json_result[valueStart] == ' ' || json_result[valueStart] == '\n')) {
      valueStart++;
    }
    QString remaining = json_result.mid(valueStart, 10).trimmed();
    
    // 检查布尔值 true/false 或字符串 "true"/"false"
    if (remaining.startsWith("true") || remaining.startsWith("\"true\"")) {
      success = "✓";
    } else if (remaining.startsWith("false") || remaining.startsWith("\"false\"")) {
      success = "✗";
    }
  }
  
  // 统计检测到的物体
  int objectCount = 0;
  QStringList objectLabels;
  
  // 如果顶层没有 success 字段，尝试提取 grasp_success 字段
  int graspSuccessStart = json_result.indexOf("\"grasp_success\"");
  if (graspSuccessStart != -1 && success == "unknown") {
    int colonPos = json_result.indexOf(":", graspSuccessStart);
    int valueStart = colonPos + 1;
    while (valueStart < json_result.length() && (json_result[valueStart] == ' ' || json_result[valueStart] == '\n')) {
      valueStart++;
    }
    QString remaining = json_result.mid(valueStart, 10).trimmed();
    
    if (remaining.startsWith("true") || remaining.startsWith("\"true\"")) {
      success = "✓";
    } else if (remaining.startsWith("false") || remaining.startsWith("\"false\"")) {
      success = "✗";
    }
  }
  
  // 检查 "object" 字段 (用于 grasp_lift, hand_delivery 等命令)
  int objectStart = json_result.indexOf("\"object\"");
  bool hasObjectField = false;
  
  if (objectStart != -1) {
    // 确保不是 "object_pose" 字段
    int colonPos = json_result.indexOf(":", objectStart);
    if (colonPos != -1 && json_result.mid(objectStart, colonPos - objectStart).contains("\"object\"")) {
      hasObjectField = true;
      
      // 在 object 字段后查找 label
      int labelPos = json_result.indexOf("\"label\"", objectStart);
      // 确保 label 在 object 字段内（检查是否在下一个顶级字段之前）
      int nextFieldPos = json_result.indexOf("\",", objectStart + 20);  // 跳过 "object": {
      
      if (labelPos != -1 && (nextFieldPos == -1 || labelPos < nextFieldPos)) {
        int labelColonPos = json_result.indexOf(":", labelPos);
        int quoteStart = json_result.indexOf("\"", labelColonPos);
        int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
        if (quoteStart != -1 && quoteEnd != -1) {
          QString label = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
          objectLabels.append(label);
          objectCount = 1;
        }
      }
      
      // 对于 scan_complete 和 success 状态，如果找到了物体，success 应该是 ✓
      if (objectCount > 0 && success == "unknown") {
        if (status == "scan_complete" || status == "grasp_complete" || status == "success") {
          success = "✓";
        }
      }
    }
  }
  
  // 如果有 object 字段但为空（如 deliver_pose），且状态是 success，使用 grasp_success
  if (hasObjectField && objectCount == 0) {
    // 对于 deliver_pose 等命令，如果有 grasp_success=true 且状态是 success，显示为成功
    if (success == "✓" && status == "success") {
      // 不需要计数物体，直接保持 success 状态
    }
  } 
  // 其次检查 "found" 字段 (用于 scan_and_grasp 命令)
  else {
    int foundStart = json_result.indexOf("\"found\"");
    if (foundStart != -1) {
      // 提取 found 中的 label
      int labelPos = json_result.indexOf("\"label\"", foundStart);
      if (labelPos != -1) {
        int colonPos = json_result.indexOf(":", labelPos);
        int quoteStart = json_result.indexOf("\"", colonPos);
        int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
        if (quoteStart != -1 && quoteEnd != -1) {
          QString label = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
          objectLabels.append(label);
          objectCount = 1;
        }
      }
      
      // 提取 grasp_success 字段作为 success 状态
      int graspSuccessStart = json_result.indexOf("\"grasp_success\"");
      if (graspSuccessStart != -1 && success == "unknown") {
        int colonPos = json_result.indexOf(":", graspSuccessStart);
        QString remaining = json_result.mid(colonPos + 1, 10);
        if (remaining.contains("true")) {
          success = "✓";
        } else if (remaining.contains("false")) {
          success = "✗";
        }
      }
    }
  }
  
  // 如果以上都没找到，检查 "results" 或 "detections" 字段
  if (objectCount == 0) {
    // 先尝试 "results" (用于 scan_tri, scan_front 等命令)
    int resultsStart = json_result.indexOf("\"results\"");
    if (resultsStart == -1) {
      // 如果没有 "results",尝试 "detections"
      resultsStart = json_result.indexOf("\"detections\"");
    }
    
    if (resultsStart != -1) {
      // 简单计数: 查找所有 "label" 出现次数
      int pos = resultsStart;
      // 找到 results/detections 数组的结束位置 (避免读取到其他字段的label)
      int arrayEnd = json_result.indexOf("]", pos);
      if (arrayEnd == -1) arrayEnd = json_result.length();
      
      while ((pos = json_result.indexOf("\"label\"", pos + 1)) != -1 && pos < arrayEnd) {
        objectCount++;
        // 提取label值
        int colonPos = json_result.indexOf(":", pos);
        int quoteStart = json_result.indexOf("\"", colonPos);
        int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
        if (quoteStart != -1 && quoteEnd != -1) {
          QString label = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
          objectLabels.append(label);
        }
      }
    }
  }
  
  // 提取 smallest_target (如果存在且不是null)
  QString smallestTarget;
  int smallestStart = json_result.indexOf("\"smallest_target\"");
  if (smallestStart != -1) {
    int colonPos = json_result.indexOf(":", smallestStart);
    // 检查值是否为null
    int nullPos = json_result.indexOf("null", colonPos);
    int quoteStart = json_result.indexOf("\"", colonPos);
    
    // 只有当值不是null且有引号时才提取
    if (quoteStart != -1 && (nullPos == -1 || quoteStart < nullPos)) {
      int quoteEnd = json_result.indexOf("\"", quoteStart + 1);
      if (quoteEnd != -1) {
        smallestTarget = json_result.mid(quoteStart + 1, quoteEnd - quoteStart - 1);
      }
    }
  }
  
  // 组装摘要
  if (!phase.isEmpty()) {
    summary = QString("📊 Command: %1 | Phase: %2 | Status: %3 | Success: %4 | Objects: %5")
              .arg(command)
              .arg(phase)
              .arg(status)
              .arg(success)
              .arg(objectCount);
  } else {
    summary = QString("📊 Command: %1 | Status: %2 | Success: %3 | Objects: %4")
              .arg(command)
              .arg(status)
              .arg(success)
              .arg(objectCount);
  }
  
  if (objectCount > 0) {
    summary += " (" + objectLabels.join(", ") + ")";
    
    // 只有在有物体且smallest_target不为空时才显示
    if (!smallestTarget.isEmpty()) {
      summary += QString(" | Smallest: %1").arg(smallestTarget);
    }
  }
  
  return summary;
}

}  // namespace arm_rviz_plugin

// Tell pluginlib about this class
PLUGINLIB_EXPORT_CLASS(arm_rviz_plugin::ArmControlPanel, rviz_common::Panel)
