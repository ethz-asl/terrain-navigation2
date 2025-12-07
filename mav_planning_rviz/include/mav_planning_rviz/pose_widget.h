#ifndef MAV_PLANNING_RVIZ_POSE_WIDGET_H_
#define MAV_PLANNING_RVIZ_POSE_WIDGET_H_

#ifndef Q_MOC_RUN
#include <QItemDelegate>
#include <QLineEdit>
#include <QStringList>
#include <QTableWidget>
#include <geometry_msgs/msg/pose.hpp>
#endif

class QLineEdit;
namespace mav_planning_rviz {

// This is a little widget that allows pose input.
class PoseWidget : public QWidget {
  Q_OBJECT
 public:
  explicit PoseWidget(const std::string& id, QWidget* parent = 0);

  std::string id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }

  void getPose(geometry_msgs::msg::Pose* pose) const;
  void setPose(const geometry_msgs::msg::Pose& pose);

  virtual QSize sizeHint() const { return table_widget_->sizeHint(); }

 Q_SIGNALS:
  void poseUpdated(const std::string& id, geometry_msgs::msg::Pose& pose);

 public Q_SLOTS:
  void itemChanged(QTableWidgetItem* item);

 protected:
  // Set up the layout, only called by the constructor.
  void createTable();

  // QT stuff:
  QTableWidget* table_widget_;

  // QT state:
  QStringList table_headers_;

  // Other state:
  // This is the ID that binds the button to the pose widget.
  std::string id_;
};

class DoubleTableDelegate : public QItemDelegate {
 public:
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_POSE_WIDGET_H_
