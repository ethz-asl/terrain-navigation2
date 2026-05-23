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

#include "terrain_navigation/dubins_path_segment.h"

double DubinsPathSegment::getLength(double /*epsilon*/) const {
  double length{0.0};
  if (states.size() <= 1) return 0.0;

  const Eigen::Vector3d segment_start = states.front().position;
  const Eigen::Vector3d segment_end = states.back().position;

  if (std::abs(curvature) < 0.0001) {
    length = (segment_end - segment_start).norm();
  } else {
    Eigen::Vector2d segment_start_2d = segment_start.head(2);
    Eigen::Vector2d segment_end_2d = segment_end.head(2);
    if (is_periodic) {
      length = 2 * M_PI * (1 / std::abs(curvature));
    } else {
      Eigen::Vector2d segment_start_tangent_2d = (states.front().velocity).head(2).normalized();
      Eigen::Vector2d arc_center_2d = getArcCenter(segment_start_2d, segment_start_tangent_2d, curvature);
      Eigen::Vector2d start_vector = (segment_start_2d - arc_center_2d).normalized();
      Eigen::Vector2d end_vector = (segment_end_2d - arc_center_2d).normalized();

      double psi = std::atan2(end_vector(1), end_vector(0)) - std::atan2(start_vector(1), start_vector(0));
      if (psi * curvature < 0) {
        psi += std::copysign(2.0 * M_PI, curvature);
      }
      length = (1 / std::abs(curvature)) * std::abs(psi);
    }
  }
  return length;
}

double DubinsPathSegment::getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point,
                                          Eigen::Vector3d &tangent, double &segment_curvature, double epsilon) {
  double theta{-std::numeric_limits<double>::infinity()};
  segment_curvature = curvature;

  const Eigen::Vector3d segment_start = states.front().position;
  const Eigen::Vector3d segment_start_tangent = (states.front().velocity).normalized();
  const Eigen::Vector3d segment_end = states.back().position;

  if (states.size() == 1) {
    theta = 1.0;
  } else if (std::abs(curvature) < epsilon) {
    theta = getLineProgress(position, segment_start, segment_end);
    tangent = (segment_end - segment_start).normalized();
    closest_point = std::max(std::min(1.0, theta), 0.0) * (segment_end - segment_start) + segment_start;
  } else {
    Eigen::Vector2d position_2d(position(0), position(1));
    Eigen::Vector2d segment_start_2d = segment_start.head(2);
    Eigen::Vector2d segment_start_tangent_2d = segment_start_tangent.head(2).normalized();
    Eigen::Vector2d segment_end_2d = segment_end.head(2);
    Eigen::Vector2d arc_center{Eigen::Vector2d::Zero()};

    if (is_periodic) {
      arc_center = getArcCenter(segment_start_2d, segment_start_tangent_2d, curvature);
      Eigen::Vector2d start_vector = (segment_start_2d - arc_center).normalized();
      Eigen::Vector2d position_vector = position_2d - arc_center;
      double angle_pos =
          std::atan2(position_vector(1), position_vector(0)) - std::atan2(start_vector(1), start_vector(0));
      wrap_2pi(angle_pos);
      theta = angle_pos / (2 * M_PI);
    } else {
      arc_center = getArcCenter(curvature, segment_start_2d, segment_start_tangent_2d, segment_end_2d);
      theta = getArcProgress(arc_center, position_2d, segment_start_2d, segment_end_2d, curvature);
    }
    Eigen::Vector2d closest_point_2d = std::abs(1 / curvature) * (position_2d - arc_center).normalized() + arc_center;
    closest_point = Eigen::Vector3d(closest_point_2d(0), closest_point_2d(1),
                                    theta * segment_end(2) + (1 - theta) * segment_start(2));
    Eigen::Vector2d error_vector = (closest_point_2d - arc_center).normalized();
    tangent = Eigen::Vector3d((curvature / std::abs(curvature)) * -error_vector(1),
                              (curvature / std::abs(curvature)) * error_vector(0), 0.0);
  }

  tangent(0) = std::cos(flightpath_angle) * tangent(0);
  tangent(1) = std::cos(flightpath_angle) * tangent(1);
  tangent(2) = std::sin(flightpath_angle);
  return theta;
}
