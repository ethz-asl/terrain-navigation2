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

#ifndef PATH_H
#define PATH_H

#include "terrain_navigation/path_segment.h"

class Path {
 public:
  Path() = default;
  virtual ~Path() = default;

  // Deep-clone copy: each segment is independently owned so that tracking
  // state (reached, viewed) in one Path does not affect copies of it.
  Path(const Path &other) : utility(other.utility), validity(other.validity) {
    segments.reserve(other.segments.size());
    for (const auto &seg : other.segments) {
      segments.push_back(seg->clone());
    }
  }
  Path &operator=(const Path &other) {
    if (this != &other) {
      segments.clear();
      segments.reserve(other.segments.size());
      for (const auto &seg : other.segments) {
        segments.push_back(seg->clone());
      }
      utility = other.utility;
      validity = other.validity;
    }
    return *this;
  }

  Path(Path &&) = default;
  Path &operator=(Path &&) = default;

  std::vector<Eigen::Vector3d> position() {
    std::vector<Eigen::Vector3d> pos_vector;
    for (const auto &segment : segments) {
      std::vector<Eigen::Vector3d> segment_pos = segment->position();
      pos_vector.insert(pos_vector.end(), segment_pos.begin(), segment_pos.end());
    }
    return pos_vector;
  }
  std::vector<Eigen::Vector3d> velocity() {
    std::vector<Eigen::Vector3d> vel_vector;
    for (const auto &segment : segments) {
      std::vector<Eigen::Vector3d> segment_vel = segment->velocity();
      vel_vector.insert(vel_vector.end(), segment_vel.begin(), segment_vel.end());
    }
    return vel_vector;
  }
  std::vector<Eigen::Vector4d> attitude() {
    std::vector<Eigen::Vector4d> attitude_vector;
    for (const auto &segment : segments) {
      std::vector<Eigen::Vector4d> segment_att = segment->attitude();
      attitude_vector.insert(attitude_vector.end(), segment_att.begin(), segment_att.end());
    }
    return attitude_vector;
  }

  void resetSegments() { segments.clear(); };

  void appendSegment(std::shared_ptr<PathSegment> segment) { segments.push_back(std::move(segment)); }
  void appendSegment(const Path &other) {
    for (const auto &seg : other.segments) {
      segments.push_back(seg->clone());
    }
  }

  void prependSegment(std::shared_ptr<PathSegment> segment) { segments.insert(segments.begin(), std::move(segment)); }
  void prependSegment(const Path &other) {
    for (int i = static_cast<int>(other.segments.size()) - 1; i >= 0; i--) {
      prependSegment(other.segments[i]->clone());
    }
  }

  PathSegment &firstSegment() { return *segments.front(); }
  PathSegment &lastSegment() { return *segments.back(); }

  /**
   * @brief Return the shared_ptr to the currently active segment.
   *
   * The returned pointer is the same object stored in this Path, so callers
   * can append it to another Path via appendSegment() directly.  Marking
   * reached on a copy-assigned Path is safe because copy-assign deep-clones
   * all segments.
   */
  std::shared_ptr<PathSegment> getCurrentSegment(const Eigen::Vector3d &position) {
    Eigen::Vector3d closest_point;
    Eigen::Vector3d tangent;
    double curvature;
    int segment_idx{-1};
    for (auto &segment : segments) {
      segment_idx++;
      if (segment->reached && (segment != segments.back())) continue;
      auto theta = segment->getClosestPoint(position, closest_point, tangent, curvature);

      // If current segment is a full circle with a following segment, escape
      // when the aircraft is close to the next segment's start.
      if (segment->is_periodic && (segment != segments.back())) {
        Eigen::Vector3d next_segment_start = segments[segment_idx + 1]->states.front().position;
        if ((closest_point - next_segment_start).norm() < epsilon_) {
          segment->reached = true;
          return segment;
        }
      }

      if (theta <= 0.0) {
        return segment;
      } else if (theta < 1.0) {
        return segment;
      } else {
        segment->reached = true;
      }
    }
    return segments.back();
  }

  void getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent,
                       double &curvature) {
    closest_point = segments.front()->states.front().position;

    for (auto &segment : segments) {
      if (segment->reached && (segment != segments.back())) continue;
      auto theta = segment->getClosestPoint(position, closest_point, tangent, curvature);

      curvature = segment->curvature;
      if (theta <= 0.0) {
        closest_point = segment->states.front().position;
        tangent = (segment->states.front().velocity).normalized();
        return;
      } else if (theta < 1.0) {
        return;
      } else {
        closest_point = segment->states.back().position;
        tangent = (segment->states.back().velocity).normalized();
        segment->reached = true;
      }
    }
  }

  Eigen::Vector3d getEndofCurrentSegment(const Eigen::Vector3d &position) {
    if (segments.empty()) return position;
    return getCurrentSegment(position)->states.back().position;
  }

  int getCurrentSegmentIndex(const Eigen::Vector3d &position) {
    Eigen::Vector3d closest_point;
    Eigen::Vector3d tangent;
    double curvature;
    int segment_idx{-1};
    for (auto &segment : segments) {
      segment_idx++;
      if (segment->reached && (segment != segments.back())) continue;
      auto theta = segment->getClosestPoint(position, closest_point, tangent, curvature);
      if (theta <= 0.0) {
        return segment_idx;
      } else if (theta < 1.0) {
        return segment_idx;
      }
    }
    return segment_idx;
  }

  double getLength(const int start_idx = 0) const {
    double length{0.0};
    for (size_t i = static_cast<size_t>(start_idx); i < segments.size(); i++) {
      length += segments[i]->getLength();
    }
    return length;
  }

  bool valid() { return validity; }

  double utility{0.0};
  bool validity{false};
  std::vector<std::shared_ptr<PathSegment>> segments;

 private:
  double epsilon_{15.0 * 0.2};
};

#endif
