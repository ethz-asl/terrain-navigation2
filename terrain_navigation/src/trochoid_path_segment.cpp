/****************************************************************************
 *
 *   Copyright (c) 2021-2023 Jaeyoung Lim, Autonomous Systems Lab,
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

#include "terrain_navigation/trochoid_path_segment.h"

#include <cmath>
#include <limits>

void TrochoidPathSegment::discretize(const Eigen::Vector3d &start_position, double start_heading, double duration) {
  states.clear();

  if (dt <= 0.0 || duration <= 0.0) return;

  // Turn rate: omega = airspeed * curvature (rad/s)
  // For straight segments curvature ≈ 0, treat as wind-displaced straight line.
  const double omega = airspeed * curvature;
  const double climb_rate = airspeed * std::sin(flightpath_angle);

  const int n = static_cast<int>(std::ceil(duration / dt));
  states.reserve(n + 1);

  for (int i = 0; i <= n; i++) {
    const double t = std::min(static_cast<double>(i) * dt, duration);
    const double psi = start_heading + omega * t;  // instantaneous heading

    State s;
    if (std::abs(omega) < 1e-6) {
      // Straight flight with wind drift
      s.position = start_position + t * Eigen::Vector3d(airspeed * std::cos(start_heading) + wind_velocity(0),
                                                        airspeed * std::sin(start_heading) + wind_velocity(1),
                                                        -climb_rate);
    } else {
      // Trochoid equations: integrating ground velocity = airspeed*(cos psi, sin psi) + wind
      const double R = 1.0 / std::abs(curvature);
      s.position(0) = start_position(0) + (airspeed / omega) * (std::sin(psi) - std::sin(start_heading)) +
                      wind_velocity(0) * t;
      s.position(1) = start_position(1) - (airspeed / omega) * (std::cos(psi) - std::cos(start_heading)) +
                      wind_velocity(1) * t;
      s.position(2) = start_position(2) - climb_rate * t;
      (void)R;  // turn radius available for reference
    }

    // Ground velocity = airspeed heading vector + wind
    s.velocity = Eigen::Vector3d(airspeed * std::cos(psi) + wind_velocity(0),
                                 airspeed * std::sin(psi) + wind_velocity(1), -climb_rate);

    // Attitude stored as identity quaternion placeholder — fill with actual
    // roll/pitch/yaw when integrating into the controller.
    s.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);

    states.push_back(s);
  }
}

double TrochoidPathSegment::getLength(double /*epsilon*/) const {
  double length = 0.0;
  for (size_t i = 1; i < states.size(); i++) {
    length += (states[i].position - states[i - 1].position).norm();
  }
  return length;
}

double TrochoidPathSegment::getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point,
                                             Eigen::Vector3d &tangent, double &segment_curvature, double /*epsilon*/) {
  segment_curvature = curvature;

  if (states.empty()) {
    closest_point = position;
    tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
    return 0.0;
  }
  if (states.size() == 1) {
    closest_point = states.front().position;
    tangent = states.front().velocity.normalized();
    return 1.0;
  }

  // Find the index of the closest discretised state.
  double min_dist = std::numeric_limits<double>::infinity();
  size_t closest_idx = 0;
  for (size_t i = 0; i < states.size(); i++) {
    const double dist = (position - states[i].position).norm();
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  closest_point = states[closest_idx].position;
  tangent = states[closest_idx].velocity.normalized();

  // Apply flightpath_angle to tangent (consistent with DubinsPathSegment)
  tangent(0) = std::cos(flightpath_angle) * tangent(0);
  tangent(1) = std::cos(flightpath_angle) * tangent(1);
  tangent(2) = std::sin(flightpath_angle);

  return static_cast<double>(closest_idx) / static_cast<double>(states.size() - 1);
}
