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

#ifndef TROCHOID_PATH_SEGMENT_H
#define TROCHOID_PATH_SEGMENT_H

#include "terrain_navigation/path_segment.h"

/**
 * @brief Path segment for trochoid (wind-aware) geometry.
 *
 * A trochoid path is traced by a UAV flying at constant airspeed while the
 * heading rotates at a fixed rate and a constant wind vector displaces the
 * ground track.  Unlike a Dubins arc the curvature of the ground-relative
 * path is not constant, so getLength() and getClosestPoint() operate on the
 * discretised states stored in PathSegment::states.
 *
 * The inherited `curvature` field stores the air-relative curvature
 * (1 / turn_radius), which the control law uses to compute the bank angle
 * command.  The ground-relative path curvature varies and is not stored
 * explicitly.
 *
 * To create a segment:
 *   1. Set wind_velocity, airspeed, curvature, flightpath_angle, dt.
 *   2. Call discretize() to populate states, or populate states manually.
 */
class TrochoidPathSegment : public PathSegment {
 public:
  TrochoidPathSegment() = default;
  ~TrochoidPathSegment() override = default;

  /**
   * @brief Populate states by integrating the trochoid equations.
   *
   * @param start_position  Initial ground position (m, NED or ENU)
   * @param start_heading   Initial heading (rad, same convention as velocity)
   * @param duration        Total segment duration (s)
   */
  void discretize(const Eigen::Vector3d &start_position, double start_heading, double duration);

  double getLength(double epsilon = 1.0E-3) const override;

  double getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent,
                         double &segment_curvature, double epsilon = 1.0E-4) override;

  std::shared_ptr<PathSegment> clone() const override { return std::make_shared<TrochoidPathSegment>(*this); }

  // Wind velocity in the world frame (m/s)
  Eigen::Vector3d wind_velocity{Eigen::Vector3d::Zero()};
  // True airspeed (m/s)
  double airspeed{15.0};
};

#endif
