#include "mav_planning_rviz/planning_panel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <functional>
#include <thread>

#include <mavros_msgs/srv/set_mode.hpp>
#include <planner_msgs/srv/set_planner_state.hpp>
#include <planner_msgs/srv/set_service.hpp>
#include <planner_msgs/srv/set_string.hpp>
#include <planner_msgs/srv/set_vector3.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "mav_planning_rviz/goal_marker.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace mav_planning_rviz {

namespace {

FLIGHT_STACK to_flight_stack(const std::string& str) {
  if (str == "px4") {
    return PX4;
  } else if (str == "ardupilot") {
    return ARDUPILOT;
  }
  return FLIGHT_STACK_NONE;
}

std::string to_string(FLIGHT_STACK flight_stack) {
  switch (flight_stack) {
    case PX4:
      return "px4";
    case ARDUPILOT:
      return "ardupilot";
    case FLIGHT_STACK_NONE:
    default:
      return "none";
  }
}

}  // namespace

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      node_(std::make_shared<rclcpp::Node>("mav_planning_rviz")) {
  createLayout();
}

void PlanningPanel::onInitialize() {
  auto rviz_ros_node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  goal_marker_ = std::make_shared<GoalMarker>(rviz_ros_node);

  // Set up menu callbacks on the goal marker
  goal_marker_->setGoalCallback(std::bind(&PlanningPanel::setGoalFromMarker, this, _1));
  goal_marker_->setStartCallback(std::bind(&PlanningPanel::setStartFromMarker, this, _1));

  planner_state_sub_ = rviz_ros_node->create_subscription<planner_msgs::msg::NavigationStatus>(
      "/planner_status", 1, std::bind(&PlanningPanel::plannerstateCallback, this, _1));
}

void PlanningPanel::createLayout() {
  QGridLayout* service_layout = new QGridLayout;

  service_layout->addWidget(createTerrainLoaderGroup(), 0, 0, 1, 1);
  service_layout->addWidget(createPlannerCommandGroup(), 1, 0, 1, 1);
  service_layout->addWidget(createPlannerModeGroup(), 2, 0, 4, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(service_layout);
  setLayout(layout);
}

QGroupBox* PlanningPanel::createPlannerModeGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Planner Actions"));
  QGridLayout* service_layout = new QGridLayout;

  set_planner_state_buttons_.push_back(new QPushButton("NAVIGATE"));
  set_planner_state_buttons_.push_back(new QPushButton("ROLLOUT"));
  set_planner_state_buttons_.push_back(new QPushButton("ABORT"));
  set_planner_state_buttons_.push_back(new QPushButton("RETURN"));

  service_layout->addWidget(set_planner_state_buttons_[0], 0, 0, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[1], 0, 1, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[3], 0, 2, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[2], 0, 3, 1, 1);
  groupBox->setLayout(service_layout);

  connect(set_planner_state_buttons_[0], SIGNAL(released()), this, SLOT(setPlannerModeServiceNavigate()));
  connect(set_planner_state_buttons_[1], SIGNAL(released()), this, SLOT(setPlannerModeServiceRollout()));
  connect(set_planner_state_buttons_[2], SIGNAL(released()), this, SLOT(setPlannerModeServiceAbort()));
  connect(set_planner_state_buttons_[3], SIGNAL(released()), this, SLOT(setPlannerModeServiceReturn()));

  return groupBox;
}

QGroupBox* PlanningPanel::createPlannerCommandGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Set Planner Problem"));
  QGridLayout* service_layout = new QGridLayout;

  planner_service_button_ = new QPushButton("Engage Planner");
  set_current_loiter_button_ = new QPushButton("Loiter Start");
  set_current_segment_button_ = new QPushButton("Current Segment");
  trigger_planning_button_ = new QPushButton("Plan");
  planning_budget_editor_ = new QLineEdit;
  max_altitude_button_enable_ = new QPushButton("Enable Max altitude");
  max_altitude_button_disable_ = new QPushButton("Disable Max altitude");
  waypoint_button_ = new QPushButton("Disengage Planner");

  service_layout->addWidget(set_current_loiter_button_, 0, 0, 1, 1);
  service_layout->addWidget(set_current_segment_button_, 0, 1, 1, 1);

  service_layout->addWidget(new QLabel("Planning budget:"), 2, 0, 1, 1);
  service_layout->addWidget(planning_budget_editor_, 2, 1, 1, 1);
  service_layout->addWidget(trigger_planning_button_, 2, 2, 1, 2);

  service_layout->addWidget(new QLabel("Max Altitude Constraints:"), 3, 0, 1, 1);
  service_layout->addWidget(max_altitude_button_enable_, 3, 1, 1, 1);
  service_layout->addWidget(max_altitude_button_disable_, 3, 2, 1, 1);

  service_layout->addWidget(planner_service_button_, 4, 0, 1, 2);
  service_layout->addWidget(waypoint_button_, 4, 2, 1, 2);

  groupBox->setLayout(service_layout);

  connect(planner_service_button_, SIGNAL(released()), this, SLOT(callPlannerService()));
  connect(set_current_loiter_button_, SIGNAL(released()), this, SLOT(setStartLoiterService()));
  connect(set_current_segment_button_, SIGNAL(released()), this, SLOT(setCurrentSegmentService()));
  connect(waypoint_button_, SIGNAL(released()), this, SLOT(publishWaypoint()));
  connect(planning_budget_editor_, SIGNAL(editingFinished()), this, SLOT(updatePlanningBudget()));
  connect(trigger_planning_button_, SIGNAL(released()), this, SLOT(setPlanningBudgetService()));
  connect(max_altitude_button_enable_, SIGNAL(released()), this, SLOT(EnableMaxAltitude()));
  connect(max_altitude_button_disable_, SIGNAL(released()), this, SLOT(DisableMaxAltitude()));

  return groupBox;
}

QGroupBox* PlanningPanel::createTerrainLoaderGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Terrain Loader"));
  QGridLayout* service_layout = new QGridLayout;

  service_layout->addWidget(new QLabel("Terrain Location:"), 0, 0, 1, 1);
  planner_name_editor_ = new QLineEdit;
  service_layout->addWidget(planner_name_editor_, 0, 1, 1, 1);
  terrain_align_checkbox_ = new QCheckBox("Virtual Terrain");
  service_layout->addWidget(terrain_align_checkbox_, 0, 2, 1, 1);
  load_terrain_button_ = new QPushButton("Load Terrain");
  service_layout->addWidget(load_terrain_button_, 0, 3, 1, 1);

  service_layout->addWidget(new QLabel("Flight Stack:"), 0, 4, 1, 1);
  flight_stack_combobox_ = new QComboBox;
  flight_stack_combobox_->addItems(flight_stack_names_);
  service_layout->addWidget(flight_stack_combobox_, 0, 5, 1, 1);

  connect(planner_name_editor_, SIGNAL(editingFinished()), this, SLOT(updatePlannerName()));
  connect(load_terrain_button_, SIGNAL(released()), this, SLOT(setPlannerName()));
  connect(flight_stack_combobox_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFlightStack()));
  connect(terrain_align_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(terrainAlignmentStateChanged(int)));

  groupBox->setLayout(service_layout);
  return groupBox;
}

void PlanningPanel::terrainAlignmentStateChanged(int state) {
  align_terrain_on_load_ = (state == 0);
}

void PlanningPanel::updatePlannerName() {
  QString new_planner_name = planner_name_editor_->text();
  if (new_planner_name != planner_name_) {
    planner_name_ = new_planner_name;
    Q_EMIT configChanged();
  }
}

void PlanningPanel::updateFlightStack() {
  std::string flight_stack_name = flight_stack_combobox_->currentText().toStdString();
  FLIGHT_STACK new_flight_stack = to_flight_stack(flight_stack_name);
  if (new_flight_stack != flight_stack_) {
    flight_stack_ = new_flight_stack;
    Q_EMIT configChanged();
  }
}

void PlanningPanel::setPlannerName() {
  std::string service_name = "/terrain_planner/set_location";
  std::string new_planner_name = planner_name_.toStdString();
  bool align_terrain = align_terrain_on_load_;

  std::thread t([this, service_name, new_planner_name, align_terrain] {
    auto client = node_->create_client<planner_msgs::srv::SetString>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetString::Request>();
    req->string = new_planner_name;
    req->align = align_terrain;

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::updatePlanningBudget() {
  setPlanningBudget(planning_budget_editor_->text());
}

void PlanningPanel::setPlanningBudget(const QString& new_planning_budget) {
  if (new_planning_budget != planning_budget_value_) {
    planning_budget_value_ = new_planning_budget;
    Q_EMIT configChanged();
  }
}

void PlanningPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("planning_budget", planning_budget_value_);
  config.mapSetValue("flight_stack", QString::fromStdString(to_string(flight_stack_)));
}

void PlanningPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  if (config.mapGetString("planner_name", &planner_name_)) {
    planner_name_editor_->setText(planner_name_);
  }
  if (config.mapGetString("planning_budget", &planning_budget_value_)) {
    planning_budget_editor_->setText(planning_budget_value_);
  }

  QString flight_stack_name;
  if (config.mapGetString("flight_stack", &flight_stack_name)) {
    int index = flight_stack_combobox_->findText(flight_stack_name);
    flight_stack_combobox_->setCurrentIndex(index);
  }
}

void PlanningPanel::callPlannerService() {
  std::string service_name = "/mavros/set_mode";

  std::thread t([this, service_name] {
    auto client = node_->create_client<mavros_msgs::srv::SetMode>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    switch (flight_stack_) {
      case PX4:
        req->custom_mode = "OFFBOARD";
        break;
      case ARDUPILOT:
        req->custom_mode = "GUIDED";
        break;
      case FLIGHT_STACK_NONE:
      default:
        req->custom_mode = "NONE";
        break;
    }

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::publishWaypoint() {
  std::string service_name = "/mavros/set_mode";

  std::thread t([this, service_name] {
    auto client = node_->create_client<mavros_msgs::srv::SetMode>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    switch (flight_stack_) {
      case PX4:
        req->custom_mode = "AUTO.RTL";
        break;
      case ARDUPILOT:
        req->custom_mode = "RTL";
        break;
      case FLIGHT_STACK_NONE:
      default:
        req->custom_mode = "NONE";
        break;
    }

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::EnableMaxAltitude() { setMaxAltitudeConstrant(true); }

void PlanningPanel::DisableMaxAltitude() { setMaxAltitudeConstrant(false); }

void PlanningPanel::setMaxAltitudeConstrant(bool set_constraint) {
  std::string service_name = "/terrain_planner/set_max_altitude";

  std::thread t([this, service_name, set_constraint] {
    auto client = node_->create_client<planner_msgs::srv::SetString>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetString::Request>();
    req->string = "";
    req->align = set_constraint;

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::setPlanningBudgetService() {
  std::string service_name = "/terrain_planner/trigger_planning";
  double planning_budget = -1.0;

  try {
    planning_budget = std::stod(planning_budget_value_.toStdString());
  } catch (const std::exception& e) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Invalid planning budget: " << e.what());
    return;
  }

  std::thread t([this, service_name, planning_budget] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.z = planning_budget;

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::setPlannerModeServiceNavigate() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", NAVIGATE);
}

void PlanningPanel::setPlannerModeServiceAbort() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", ABORT);
}

void PlanningPanel::setPlannerModeServiceReturn() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", RETURN);
}

void PlanningPanel::setPlannerModeServiceRollout() {
  callSetPlannerStateService("/terrain_planner/set_planner_state", ROLLOUT);
}

void PlanningPanel::callSetPlannerStateService(std::string service_name, const int mode) {
  std::thread t([this, service_name, mode] {
    auto client = node_->create_client<planner_msgs::srv::SetPlannerState>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetPlannerState::Request>();
    req->state = mode;

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::setGoalFromMarker(const geometry_msgs::msg::Pose& pose) {
  std::string service_name = "/terrain_planner/set_goal";

  std::thread t([this, service_name, pose] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = pose.position.x;
    req->vector.y = pose.position.y;
    req->vector.z = -1.0;  // Negative altitude invalidates the altitude setpoint

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::setStartFromMarker(const geometry_msgs::msg::Pose& pose) {
  std::string service_name = "/terrain_planner/set_start";

  std::thread t([this, service_name, pose] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = pose.position.x;
    req->vector.y = pose.position.y;
    req->vector.z = -1.0;  // Negative altitude invalidates the altitude setpoint

    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::setStartLoiterService() {
  std::string service_name = "/terrain_planner/set_start_loiter";

  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();
    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::setCurrentSegmentService() {
  std::string service_name = "/terrain_planner/set_current_segment";

  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();
    auto result = client->async_send_request(req);

    const std::lock_guard<std::mutex> lock(node_mutex_);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    }
  });
  t.detach();
}

void PlanningPanel::odometryCallback(const nav_msgs::msg::Odometry& /*msg*/) {
  // Currently unused - placeholder for odometry updates
}

void PlanningPanel::plannerstateCallback(const planner_msgs::msg::NavigationStatus& msg) {
  switch (msg.state) {
    case PLANNER_STATE::HOLD:
      set_planner_state_buttons_[0]->setDisabled(false);  // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(false);  // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(true);   // ABORT
      set_planner_state_buttons_[3]->setDisabled(false);  // RETURN
      break;
    case PLANNER_STATE::NAVIGATE:
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(false);  // RETURN
      break;
    case PLANNER_STATE::ROLLOUT:
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);   // RETURN
      break;
    case PLANNER_STATE::ABORT:
      set_planner_state_buttons_[0]->setDisabled(true);  // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);  // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(true);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);  // RETURN
      break;
    case PLANNER_STATE::RETURN:
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);   // RETURN
      break;
  }
}

}  // namespace mav_planning_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanel, rviz_common::Panel)
