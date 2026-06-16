/****************************************************************************
 *
 *   Copyright (c) 2025 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief PX4 ROS 2 external flight mode for terrain navigation.
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TERRAIN_NAVIGATION_MODE_H
#define TERRAIN_NAVIGATION_MODE_H

#include <Eigen/Dense>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/global_path_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief PX4 ROS 2 external flight mode that publishes GlobalPathSetpoint.
 *
 * Registered with PX4 as "Terrain Navigation" mode. The TerrainPlanner drives
 * setpoint publishing directly via publishSetpoint(); the mode's built-in
 * update timer is disabled to avoid conflicts.
 */
class TerrainNavigationMode : public px4_ros2::ModeBase {
 public:
  explicit TerrainNavigationMode(rclcpp::Node& node, const std::string& topic_prefix = "")
      : px4_ros2::ModeBase(node, px4_ros2::ModeBase::Settings{"Terrain Navigation"}, topic_prefix) {
    _path_setpoint = std::make_shared<px4_ros2::FwGlobalPathSetpointType>(*this);
    disableWatchdogTimer();
    setSetpointUpdateRate(0.f);
  }

  ~TerrainNavigationMode() override = default;

  void onActivate() override {}
  void onDeactivate() override {}

  /**
   * @brief Publish a global path setpoint to PX4.
   * @param lat Latitude [deg]
   * @param lon Longitude [deg]
   * @param alt Altitude AMSL [m]
   * @param tangent_ned Unit tangent vector in NED frame
   * @param height_rate Height rate in NED convention [m/s]
   * @param curvature Path curvature in NED convention [1/m]
   */
  void publishSetpoint(double lat, double lon, float alt, const Eigen::Vector3f& tangent_ned, float height_rate,
                       float curvature) {
    _path_setpoint->update(px4_ros2::FwGlobalPathSetpoint{}
                               .withLatLon(lat, lon)
                               .withAltitude(alt)
                               .withTangent(tangent_ned)
                               .withHeightRate(height_rate)
                               .withCurvature(curvature));
  }

 private:
  std::shared_ptr<px4_ros2::FwGlobalPathSetpointType> _path_setpoint;
};

#endif  // TERRAIN_NAVIGATION_MODE_H
