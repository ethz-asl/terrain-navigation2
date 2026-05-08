#ifndef MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
#define MAV_PLANNING_RVIZ_PLANNING_PANEL_H_

#ifndef Q_MOC_RUN
#include <QGroupBox>
#include <mutex>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/navigation_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "mav_planning_rviz/goal_marker.h"
#endif

class QLineEdit;
class QCheckBox;
class QComboBox;
class QPushButton;

namespace mav_planning_rviz {

enum PLANNER_STATE { HOLD = 1, NAVIGATE = 2, ROLLOUT = 3, ABORT = 4, RETURN = 5 };
enum FLIGHT_STACK { FLIGHT_STACK_NONE = 0, PX4 = 1, ARDUPILOT = 2 };

class PlanningPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit PlanningPanel(QWidget* parent = 0);

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;
  virtual void onInitialize();

  // And when we get robot odometry:
  void odometryCallback(const nav_msgs::msg::Odometry& msg);
  void plannerstateCallback(const planner_msgs::msg::NavigationStatus& msg);

  // Callbacks for interactive marker menu actions:
  void setGoalFromMarker(const geometry_msgs::msg::Pose& pose);
  void setStartFromMarker(const geometry_msgs::msg::Pose& pose);

 public Q_SLOTS:
  void updatePlannerName();
  void updateFlightStack();
  void updatePlanningBudget();
  void setPlannerName();
  void callPlannerService();
  void setPlanningBudgetService();
  void setStartLoiterService();
  void setCurrentSegmentService();
  void publishWaypoint();
  void terrainAlignmentStateChanged(int state);
  void EnableMaxAltitude();
  void DisableMaxAltitude();
  void setPathService();
  void setPlannerModeServiceNavigate();
  void setPlannerModeServiceRollout();
  void setPlannerModeServiceAbort();
  void setPlannerModeServiceReturn();

 protected:
  void createLayout();
  void setPlanningBudget(const QString& new_planning_budget);
  void setMaxAltitudeConstrant(bool set_constraint);
  void callSetPlannerStateService(std::string service_name, const int mode);
  QGroupBox* createPlannerModeGroup();
  QGroupBox* createPlannerCommandGroup();
  QGroupBox* createTerrainLoaderGroup();

  // ROS Stuff:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<planner_msgs::msg::NavigationStatus>::SharedPtr planner_state_sub_;

  std::shared_ptr<GoalMarker> goal_marker_;

  std::mutex node_mutex_;

  // QT stuff:
  QLineEdit* planner_name_editor_;
  QComboBox* flight_stack_combobox_;
  QLineEdit* planning_budget_editor_;
  QCheckBox* terrain_align_checkbox_;
  QPushButton* planner_service_button_;
  QPushButton* set_current_loiter_button_;
  QPushButton* set_current_segment_button_;
  QPushButton* trigger_planning_button_;
  QPushButton* waypoint_button_;
  QPushButton* max_altitude_button_enable_;
  QPushButton* max_altitude_button_disable_;
  QPushButton* load_terrain_button_;
  QPushButton* update_path_button_;
  std::vector<QPushButton*> set_planner_state_buttons_;

  // QT state:
  QString planner_name_;
  QString planning_budget_value_{"100.0"};
  bool align_terrain_on_load_{true};

  // Select flight stack (affects offboard/guided and RTL commands)
  QStringList flight_stack_names_ = {"px4", "ardupilot"};
  FLIGHT_STACK flight_stack_{PX4};
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
