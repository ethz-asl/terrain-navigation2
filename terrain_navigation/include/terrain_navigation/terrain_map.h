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

#ifndef TERRAIN_MAP_H
#define TERRAIN_MAP_H

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_geo/grid_map_geo.hpp>
#include <grid_map_geo_wavelet_quadtree/wavelet_terrain_map.hpp>
#include <iostream>
#include <memory>

#if __APPLE__
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_p.h>
#include <ogr_spatialref.h>
#else
#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>
#endif

class TerrainMap : public WaveletTerrainMap {
 public:
  TerrainMap();
  virtual ~TerrainMap();
  void addLayerSafety(const std::string& layer, const std::string& layer_name_lowerbound,
                      const std::string& layer_name_upperbound);
  void AddLayerNormals(const std::string reference_layer);
  bool isInCollision(const std::string& layer, const Eigen::Vector3d& position, bool is_above = true);
  double getCollisionDepth(const std::string& layer, const Eigen::Vector3d& position, bool is_above = true);

  /**
   * @brief Compression-error-bounded override of
   * GridMapGeo::AddLayerDistanceTransform, for maps loaded via
   * LoadFromWaveletQuadtree with a nonzero max-error tolerance.
   *
   * The reconstructed "elevation" layer of such a map is only known to
   * within WaveletTerrainMap::getCompressionErrorBound() of the true source
   * value. Building "distance_surface" (a floor the vehicle must stay
   * above -- see ompl::TerrainValidityChecker::checkCollision) or
   * "max_elevation" (a ceiling it must stay below) directly from that
   * layer would only guarantee clearance from the *reconstructed*
   * terrain, not the true terrain. This inflates/deflates the elevation
   * reference by the compression error bound first, in whichever
   * direction keeps the resulting surface a genuine, conservative bound
   * on the true terrain -- so an existing call site needs no changes to
   * benefit. Any layer name other than these two falls back to the
   * exact, unmodified transform, since the direction of the needed
   * correction is specific to which side of the map's flight corridor
   * the layer represents.
   */
  bool AddLayerDistanceTransform(const double surface_distance, const std::string& layer_name);

  /**
   * @brief Compression-error-bounded override of
   * GridMapGeo::AddLayerOffsetTransform -- a literal Above-Ground-Level
   * (AGL) surface, i.e. elevation directly below plus a fixed offset, with
   * no horizontal clearance radius. Use this instead of
   * AddLayerDistanceTransform when collision limits are meant to enforce an
   * AGL ceiling/floor (e.g. Part 107's altitude limits) rather than a true
   * 3D clearance from nearby terrain. Same floor-raised/ceiling-lowered
   * compression-error handling as AddLayerDistanceTransform above.
   */
  bool AddLayerOffsetTransform(const double surface_distance, const std::string& layer_name);

 protected:
 private:
  /**
   * @brief Shifts an already-computed "distance_surface"/"max_elevation"
   * layer by +-getCompressionErrorBound(), in whichever direction keeps it
   * a genuine, conservative bound on the true (uncompressed) terrain. Both
   * GridMapGeo::AddLayerDistanceTransform and ::AddLayerOffsetTransform
   * compute their output as reference_layer(cell) plus a purely geometric
   * term, so shifting the *output* by a constant here is exactly equivalent
   * to (but far cheaper than) pre-shifting the whole "elevation" layer
   * before running the transform -- and unlike pre-shifting, it never adds
   * a scratch layer to grid_map_, which is published wholesale (see
   * TerrainPlanner::MapPublishOnce) and must not carry internal
   * bookkeeping layers. Any layer name other than these two, or a zero
   * bound, is left untouched.
   */
  void applyCompressionErrorMargin(const std::string& layer_name);
};
#endif
