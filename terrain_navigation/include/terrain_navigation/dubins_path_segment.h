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

#ifndef DUBINS_PATH_SEGMENT_H
#define DUBINS_PATH_SEGMENT_H

#include "terrain_navigation/path_segment.h"

/**
 * @brief Path segment for Dubins (arc or straight-line) geometry.
 *
 * Each segment is one of the three primitives that compose a Dubins path:
 * a left arc (curvature > 0), a right arc (curvature < 0), or a straight
 * line (curvature ≈ 0). The flightpath_angle extends this to 3-D helical
 * and climbing/descending variants.
 */
class DubinsPathSegment : public PathSegment {
 public:
  DubinsPathSegment() = default;
  ~DubinsPathSegment() override = default;

  double getLength(double epsilon = 1.0E-3) const override;

  double getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent,
                         double &segment_curvature, double epsilon = 1.0E-4) override;

  std::shared_ptr<PathSegment> clone() const override { return std::make_shared<DubinsPathSegment>(*this); }
};

#endif
