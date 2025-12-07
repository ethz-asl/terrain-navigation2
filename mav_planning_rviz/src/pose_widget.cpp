#include "mav_planning_rviz/pose_widget.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>
#include <cmath>

namespace mav_planning_rviz {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

namespace {

// Helper to extract yaw from quaternion
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Helper to set quaternion from yaw
void setQuaternionFromYaw(double yaw, geometry_msgs::msg::Quaternion* q) {
  q->w = std::cos(yaw / 2.0);
  q->x = 0.0;
  q->y = 0.0;
  q->z = std::sin(yaw / 2.0);
}

}  // namespace

PoseWidget::PoseWidget(const std::string& id, QWidget* parent) : QWidget(parent), id_(id) { createTable(); }

void PoseWidget::createTable() {
  table_widget_ = new QTableWidget(this);
  table_widget_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  table_widget_->setRowCount(1);
  table_widget_->setColumnCount(4);
  table_headers_ << "x [m]"
                 << "y [m]"
                 << "z [m]" << QString::fromUtf8("yaw [°]");
  table_widget_->setHorizontalHeaderLabels(table_headers_);
  table_widget_->verticalHeader()->setVisible(false);
  table_widget_->setShowGrid(true);
  table_widget_->setItemDelegate(new DoubleTableDelegate);

  table_widget_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table_widget_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table_widget_->resizeColumnsToContents();
  // From:
  // https://stackoverflow.com/questions/8766633/how-to-determine-the-correct-size-of-a-qtablewidget
  table_widget_->setFixedSize(table_widget_->horizontalHeader()->length(),
                              table_widget_->verticalHeader()->length() + table_widget_->horizontalHeader()->height());

  for (int i = 0; i < 4; i++) {
    table_widget_->setItem(0, i, new QTableWidgetItem("0.00"));
    table_widget_->item(0, i)->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }

  connect(table_widget_, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(itemChanged(QTableWidgetItem*)));
}

void PoseWidget::getPose(geometry_msgs::msg::Pose* pose) const {
  pose->position.x = table_widget_->item(0, 0)->text().toDouble();
  pose->position.y = table_widget_->item(0, 1)->text().toDouble();
  pose->position.z = table_widget_->item(0, 2)->text().toDouble();
  double yaw = table_widget_->item(0, 3)->text().toDouble() * kDegToRad;
  setQuaternionFromYaw(yaw, &pose->orientation);
}

void PoseWidget::setPose(const geometry_msgs::msg::Pose& pose) {
  table_widget_->blockSignals(true);
  table_widget_->item(0, 0)->setText(QString::number(pose.position.x, 'f', 2));
  table_widget_->item(0, 1)->setText(QString::number(pose.position.y, 'f', 2));
  table_widget_->item(0, 2)->setText(QString::number(pose.position.z, 'f', 2));
  double yaw = getYawFromQuaternion(pose.orientation);
  table_widget_->item(0, 3)->setText(QString::number(yaw * kRadToDeg, 'f', 2));
  table_widget_->blockSignals(false);
}

void PoseWidget::itemChanged(QTableWidgetItem* /*item*/) {
  geometry_msgs::msg::Pose pose;
  getPose(&pose);
  Q_EMIT poseUpdated(id_, pose);
}

QWidget* DoubleTableDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& /*option*/,
                                           const QModelIndex& /*index*/) const {
  // From:
  // https://stackoverflow.com/questions/22708623/qtablewidget-only-numbers-permitted
  QLineEdit* line_edit = new QLineEdit(parent);
  // Set validator
  QDoubleValidator* validator = new QDoubleValidator(line_edit);
  line_edit->setValidator(validator);
  line_edit->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return line_edit;
}

}  // namespace mav_planning_rviz
