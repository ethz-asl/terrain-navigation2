/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
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
 * @brief ROS Node to test ompl
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <any>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "terrain_navigation/data_logger.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

#include <iostream>
#include <cmath>
#include <ctime>

using namespace std::chrono_literals;


class SolarCalculator {
private:
    static constexpr double PI = 3.14159265359;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;
    
    // WGS84 ellipsoid parameters
    static constexpr double EQUATORIAL_RADIUS = 6378137.0;
    static constexpr double FLATTENING = 1.0 / 298.257223563;
    static constexpr double ECCENTRICITY_SQUARED = 2 * FLATTENING - FLATTENING * FLATTENING;

public:
    struct SolarAngles {
        double elevation;  // Solar elevation angle (degrees)
        double azimuth;    // Solar azimuth angle (degrees)
        double zenith;     // Solar zenith angle (degrees)
    };
    
    struct GeographicCoords {
        double latitude;   // degrees
        double longitude;  // degrees
    };
    
    // Convert UTM coordinates to Geographic (Lat/Lon)
    static GeographicCoords utmToGeographic(double easting, double northing, int zone, bool isNorthern = true) {
        // UTM parameters
        double k0 = 0.9996;  // Scale factor
        double e1 = (1 - sqrt(1 - ECCENTRICITY_SQUARED)) / (1 + sqrt(1 - ECCENTRICITY_SQUARED));
        
        // Remove false easting and northing
        double x = easting - 500000.0;
        double y = isNorthern ? northing : northing - 10000000.0;
        
        // Central meridian for the zone
        double lonOrigin = (zone - 1) * 6 - 180 + 3;  // in degrees
        
        // Calculate footprint latitude
        double M = y / k0;
        double mu = M / (EQUATORIAL_RADIUS * (1 - ECCENTRICITY_SQUARED/4 - 3*ECCENTRICITY_SQUARED*ECCENTRICITY_SQUARED/64 - 5*pow(ECCENTRICITY_SQUARED, 3)/256));
        
        double phi1Rad = mu + (3*e1/2 - 27*pow(e1,3)/32) * sin(2*mu) + (21*e1*e1/16 - 55*pow(e1,4)/32) * sin(4*mu) + (151*pow(e1,3)/96) * sin(6*mu);
        
        // Calculate latitude and longitude
        double N1 = EQUATORIAL_RADIUS / sqrt(1 - ECCENTRICITY_SQUARED * sin(phi1Rad) * sin(phi1Rad));
        double T1 = tan(phi1Rad) * tan(phi1Rad);
        double C1 = ECCENTRICITY_SQUARED * cos(phi1Rad) * cos(phi1Rad) / (1 - ECCENTRICITY_SQUARED);
        double R1 = EQUATORIAL_RADIUS * (1 - ECCENTRICITY_SQUARED) / pow(1 - ECCENTRICITY_SQUARED * sin(phi1Rad) * sin(phi1Rad), 1.5);
        double D = x / (N1 * k0);
        
        double lat = phi1Rad - (N1 * tan(phi1Rad) / R1) * (D*D/2 - (5 + 3*T1 + 10*C1 - 4*C1*C1 - 9*ECCENTRICITY_SQUARED) * pow(D,4)/24 + (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*ECCENTRICITY_SQUARED - 3*C1*C1) * pow(D,6)/720);
        
        double lon = (D - (1 + 2*T1 + C1) * pow(D,3)/6 + (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*ECCENTRICITY_SQUARED + 24*T1*T1) * pow(D,5)/120) / cos(phi1Rad);
        
        GeographicCoords coords;
        coords.latitude = lat * RAD_TO_DEG;
        coords.longitude = lonOrigin + lon * RAD_TO_DEG;
        
        return coords;
    }

    // Calculate Julian Day Number
    static double julianDay(int year, int month, int day, int hour, int minute, int second) {
        if (month <= 2) {
            year -= 1;
            month += 12;
        }
        
        int a = year / 100;
        int b = 2 - a + (a / 4);
        
        double jd = floor(365.25 * (year + 4716)) + 
                   floor(30.6001 * (month + 1)) + 
                   day + b - 1524.5;
        
        // Add time of day
        jd += (hour + minute/60.0 + second/3600.0) / 24.0;
        
        return jd;
    }

    // Calculate solar declination angle
    static double solarDeclination(double julianDay) {
        double n = julianDay - 2451545.0;
        double L = fmod(280.460 + 0.9856474 * n, 360.0);
        double g = (357.528 + 0.9856003 * n) * DEG_TO_RAD;
        double lambda = (L + 1.915 * sin(g) + 0.020 * sin(2 * g)) * DEG_TO_RAD;
        
        double declination = asin(sin(23.439 * DEG_TO_RAD) * sin(lambda));
        return declination;
    }

    // Calculate equation of time
    static double equationOfTime(double julianDay) {
        double n = julianDay - 2451545.0;
        double L = fmod(280.460 + 0.9856474 * n, 360.0) * DEG_TO_RAD;
        double g = fmod(357.528 + 0.9856003 * n, 360.0) * DEG_TO_RAD;
        double lambda = L + 1.915 * DEG_TO_RAD * sin(g) + 0.020 * DEG_TO_RAD * sin(2 * g);
        
        double alpha = atan2(cos(23.439 * DEG_TO_RAD) * sin(lambda), cos(lambda));
        double E = L - alpha;
        
        // Normalize to [-PI, PI]
        while (E > PI) E -= 2 * PI;
        while (E < -PI) E += 2 * PI;
        
        return E * RAD_TO_DEG * 4; // Convert to minutes
    }

    // Main function to calculate solar angles
    static SolarAngles calculateSolarAngles(double latitude, double longitude, 
                                          int year, int month, int day, 
                                          int hour, int minute, int second) {
        // Convert inputs
        latitude *= DEG_TO_RAD;
        
        // Calculate Julian Day
        double jd = julianDay(year, month, day, hour, minute, second);
        
        // Solar declination
        double declination = solarDeclination(jd);
        
        // Equation of time (in minutes)
        double eot = equationOfTime(jd);
        
        // Hour angle
        double timeOffset = eot + 4 * longitude; // longitude correction in minutes
        double tst = hour * 60 + minute + second/60.0 + timeOffset; // True solar time in minutes
        double hourAngle = (tst / 4.0 - 180.0) * DEG_TO_RAD; // Hour angle in radians
        
        // Solar elevation angle
        double elevation = asin(sin(declination) * sin(latitude) + 
                               cos(declination) * cos(latitude) * cos(hourAngle));
        
        // Solar azimuth angle
        double azimuth = atan2(sin(hourAngle), 
                              cos(hourAngle) * sin(latitude) - 
                              tan(declination) * cos(latitude));
        
        // Convert to degrees and normalize azimuth
        SolarAngles angles;
        angles.elevation = elevation * RAD_TO_DEG;
        angles.zenith = 90.0 - angles.elevation;
        angles.azimuth = fmod(azimuth * RAD_TO_DEG + 180.0, 360.0);
        
        return angles;
    }
    
    // Calculate solar angles from UTM coordinates
    static SolarAngles calculateSolarAnglesUTM(double easting, double northing, int utmZone, bool isNorthern,
                                              int year, int month, int day, 
                                              int hour, int minute, int second) {
        // Convert UTM to Geographic coordinates
        GeographicCoords geoCoords = utmToGeographic(easting, northing, utmZone, isNorthern);
        
        // Use existing solar calculation with converted coordinates
        return calculateSolarAngles(geoCoords.latitude, geoCoords.longitude, 
                                  year, month, day, hour, minute, second);
    }
    
    // Convenience function for current time with UTM input
    static SolarAngles calculateCurrentSolarAnglesUTM(double easting, double northing, int utmZone, bool isNorthern = true) {
        time_t now = time(0);
        tm* ltm = localtime(&now);
        
        return calculateSolarAnglesUTM(easting, northing, utmZone, isNorthern,
                                     1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday,
                                     ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    }
};

void publishCircleSetpoints(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                            const Eigen::Vector3d& position, const double radius) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.header.stamp = rclcpp::Clock().now();
  std::vector<geometry_msgs::msg::Point> points;
  for (double t = 0.0; t <= 1.0; t += 0.02) {
    geometry_msgs::msg::Point point;
    point.x = position.x() + radius * std::cos(t * 2 * M_PI);
    point.y = position.y() + radius * std::sin(t * 2 * M_PI);
    point.z = position.z();
    points.push_back(point);
  }
  geometry_msgs::msg::Point start_point;
  start_point.x = position.x() + radius * std::cos(0.0);
  start_point.y = position.y() + radius * std::sin(0.0);
  start_point.z = position.z();
  points.push_back(start_point);

  marker.points = points;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  pub->publish(marker);
}

void getDubinsShortestPath(std::shared_ptr<ompl::base::OwenStateSpace>& dubins_ss, const Eigen::Vector3d start_pos,
                           const double start_yaw, const Eigen::Vector3d goal_pos, const double goal_yaw,
                           std::vector<Eigen::Vector3d>& path) {
  ompl::base::State* from = dubins_ss->allocState();
  from->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] =
      start_pos.x();
  from->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] =
      start_pos.y();
  from->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] =
      start_pos.z();
  from->as<ompl::base::OwenStateSpace::StateType>()->yaw() = start_yaw;

  ompl::base::State* to = dubins_ss->allocState();
  to->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] =
      goal_pos.x();
  to->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] =
      goal_pos.y();
  to->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] =
      goal_pos.z();
  to->as<ompl::base::OwenStateSpace::StateType>()->yaw() = goal_yaw;

  ompl::base::State* state = dubins_ss->allocState();
  for (double t = 0.0; t < 1.0; t += 0.02) {
    dubins_ss->interpolate(from, to, t, state);
    auto interpolated_state = Eigen::Vector3d(state->as<ompl::base::OwenStateSpace::StateType>()
                                                  ->as<ompl::base::RealVectorStateSpace::StateType>(0)
                                                  ->values[0],
                                              state->as<ompl::base::OwenStateSpace::StateType>()
                                                  ->as<ompl::base::RealVectorStateSpace::StateType>(0)
                                                  ->values[1],
                                              state->as<ompl::base::OwenStateSpace::StateType>()
                                                  ->as<ompl::base::RealVectorStateSpace::StateType>(0)
                                                  ->values[2]);
    path.push_back(interpolated_state);
  }
}

bool validatePosition(std::shared_ptr<TerrainMap> map, const Eigen::Vector3d goal, Eigen::Vector3d& valid_goal) {
  double upper_surface = map->getGridMap().atPosition("ics_+", goal.head(2));
  double lower_surface = map->getGridMap().atPosition("ics_-", goal.head(2));
  const bool is_goal_valid = (upper_surface < lower_surface) ? true : false;
  valid_goal(0) = goal(0);
  valid_goal(1) = goal(1);
  valid_goal(2) = (upper_surface + lower_surface) / 2.0;
  return is_goal_valid;
}

double mod2pi(double x) { return x - 2 * M_PI * floor(x * (0.5 / M_PI)); }

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

PathSegment generateArcTrajectory(Eigen::Vector3d rate, const double horizon, Eigen::Vector3d current_pos,
                                  Eigen::Vector3d current_vel, const double dt = 0.1) {
  PathSegment trajectory;
  trajectory.states.clear();

  double cruise_speed_{20.0};

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.flightpath_angle = std::asin(climb_rate / cruise_speed_);
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = -rate(2) / cruise_speed_;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    double yaw = rate(2) * time + current_yaw;

    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = Eigen::Vector3d(cruise_speed_ * std::cos(yaw), -cruise_speed_ * std::sin(yaw), -climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);

    time = time + dt;
  }
  return trajectory;
}

PathSegment getLoiterPath(Eigen::Vector3d end_position, Eigen::Vector3d end_velocity, Eigen::Vector3d center_pos) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  PathSegment loiter_trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  return loiter_trajectory;
}

class ThermalGenerator : public rclcpp::Node {
 public:
 ThermalGenerator() : Node("terrain_generator") {
    grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 1);

    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ThermalGenerator::timer_callback, this));

    std::string map_path = this->declare_parameter("map_path", "");
    std::string color_file_path = this->declare_parameter("color_file_path", "");
    std::string location = this->declare_parameter("location", "");
    std::string output_directory = this->declare_parameter("output_directory", "");

    // Load terrain map from defined tif paths
    terrain_map = std::make_shared<TerrainMap>();
    terrain_map->initializeFromGeotiff(map_path);
    if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
      terrain_map->addColorFromGeotiff(color_file_path);
    }
    terrain_map->AddLayerNormals("elevation");
  }

  void timer_callback() {
    std::cout << "Publishing results" << std::endl;

    start_min = start_min + 5;
    if (start_min %60 == 0) {
      start_min = 0;
      start_time = (start_time + 1)%24;
    }

    // Berkeley, CA in UTM Zone 10N coordinates
    // UC Berkeley campus center approximately:
    double easting = 551455.0;   // UTM Easting (meters)
    double northing = 4185540.0; // UTM Northing (meters)
    int utmZone = 10;           // UTM Zone 10N for California Bay Area
    bool isNorthern = true;     // Northern hemisphere
        
    SolarCalculator::SolarAngles ggAngles = 
        SolarCalculator::calculateSolarAnglesUTM(easting, northing, utmZone, isNorthern,
                                               2024, 6, 21, start_time, 0, 0); // 3 PM
    
    std::cout << "Golden Gate Bridge area (UTM: " << easting << "E, " << northing << "N):" << std::endl;
    std::cout << "June 21, " << start_time << ":" << start_min << " - Elevation: " << ggAngles.elevation 
              << "°, Azimuth: " << ggAngles.azimuth << "°" << std::endl;
    Eigen::Vector3d solar_angle = Eigen::Vector3d(std::cos(ggAngles.azimuth/180.0*M_PI), std::sin(ggAngles.azimuth/180.0*M_PI), std::tan(ggAngles.elevation/180.0*M_PI));
    solar_angle = solar_angle.normalized();
    terrain_map->AddHeatFluxLayer("heatflux", solar_angle);

    // Repeatedly publish results
    auto message = grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap());
    grid_map_pub->publish(*message);
  }

 private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub;
  rclcpp::TimerBase::SharedPtr timer;

  std::shared_ptr<TerrainMap> terrain_map;

  int start_time = 9;
  int start_min = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto thermal_generator = std::make_shared<ThermalGenerator>();
    
  rclcpp::spin(thermal_generator);
  rclcpp::shutdown();
  return 0;
}

