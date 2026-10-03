#include "jr_perception/opponent_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace jr_perception::opponent_core {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;
using Clock = std::chrono::steady_clock;

double ms(Clock::time_point start, Clock::time_point end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double sq(double v) { return v * v; }
double clamp(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }
double norm2(double x, double y) { return std::sqrt(sq(x) + sq(y)); }
bool finite(double v) { return std::isfinite(v); }

Vec3 transform_point(const Transform & t, const Point & p)
{
  // q * v * conjugate(q), using the equivalent cross-product expression.
  const double qnorm = std::sqrt(sq(t.qx) + sq(t.qy) + sq(t.qz) + sq(t.qw));
  const double qx = t.qx / qnorm;
  const double qy = t.qy / qnorm;
  const double qz = t.qz / qnorm;
  const double qw = t.qw / qnorm;
  const double ux = qy * p.z - qz * p.y;
  const double uy = qz * p.x - qx * p.z;
  const double uz = qx * p.y - qy * p.x;
  const double vx = qy * uz - qz * uy;
  const double vy = qz * ux - qx * uz;
  const double vz = qx * uy - qy * ux;
  return {
    t.translation.x + p.x + 2.0 * (qw * ux + vx),
    t.translation.y + p.y + 2.0 * (qw * uy + vy),
    t.translation.z + p.z + 2.0 * (qw * uz + vz)};
}

bool valid_transform(const Transform & t)
{
  return finite(t.translation.x) && finite(t.translation.y) && finite(t.translation.z) &&
         finite(t.qx) && finite(t.qy) && finite(t.qz) && finite(t.qw) &&
         sq(t.qx) + sq(t.qy) + sq(t.qz) + sq(t.qw) > 0.5;
}

bool valid_map(const Map * map)
{
  return map && map->width > 0 && map->height > 0 && map->resolution > 0.0 &&
         finite(map->origin_x) && finite(map->origin_y) && finite(map->origin_yaw) &&
         map->occupancy.size() == map->width * map->height;
}

struct WorldPoint {
  Vec3 position;
  double range = 0.0;
  bool mapped = false;
};

struct GridIndex {
  int x = 0;
  int y = 0;
};

GridIndex grid_index(const Map & map, double x, double y)
{
  const double dx = x - map.origin_x;
  const double dy = y - map.origin_y;
  const double c = std::cos(map.origin_yaw);
  const double s = std::sin(map.origin_yaw);
  return {
    static_cast<int>(std::floor((c * dx + s * dy) / map.resolution)),
    static_cast<int>(std::floor((-s * dx + c * dy) / map.resolution))};
}

int occupancy_at(const Map & map, int ix, int iy)
{
  if (ix < 0 || iy < 0 || static_cast<std::size_t>(ix) >= map.width ||
    static_cast<std::size_t>(iy) >= map.height)
  {
    return -1;
  }
  return map.occupancy[static_cast<std::size_t>(iy) * map.width +
                       static_cast<std::size_t>(ix)];
}

bool occupied_near(const Map & map, double x, double y, double margin)
{
  const auto center = grid_index(map, x, y);
  const int cells = static_cast<int>(std::ceil(margin / map.resolution)) + 1;
  const double c = std::cos(map.origin_yaw);
  const double s = std::sin(map.origin_yaw);
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const int ix = center.x + dx;
      const int iy = center.y + dy;
      if (occupancy_at(map, ix, iy) < 50) {
        continue;
      }
      // Distance to the occupied cell rectangle, not just its center.
      const double lx = c * (x - map.origin_x) + s * (y - map.origin_y);
      const double ly = -s * (x - map.origin_x) + c * (y - map.origin_y);
      const double near_x = clamp(lx, ix * map.resolution, (ix + 1) * map.resolution);
      const double near_y = clamp(ly, iy * map.resolution, (iy + 1) * map.resolution);
      if (norm2(lx - near_x, ly - near_y) <= margin) {
        return true;
      }
    }
  }
  return false;
}

std::uint64_t key(int x, int y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
         static_cast<std::uint32_t>(y);
}

std::uint64_t key3(int x, int y, int z)
{
  constexpr std::uint64_t mask = (1ULL << 21) - 1;
  return ((static_cast<std::uint64_t>(x) & mask) << 42) |
         ((static_cast<std::uint64_t>(y) & mask) << 21) |
         (static_cast<std::uint64_t>(z) & mask);
}

struct Cell {
  int x = 0;
  int y = 0;
  std::vector<std::size_t> points;
  bool visited = false;
};

struct Features {
  Candidate candidate;
  double line_rms = 0.0;
  double circle_rms = 0.0;
  double ring_fraction = 0.0;
  double shape_width = 0.0;
  double shape_length = 0.0;
  double top = 0.0;
  double bottom = 0.0;
  std::size_t through_ray_bins = 0;
};

std::string reason_join(const std::vector<std::string> & reasons)
{
  if (reasons.empty()) {
    return "robot_geometry_supported";
  }
  std::ostringstream out;
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i) {out << ';';}
    out << reasons[i];
  }
  return out.str();
}

std::uint64_t spatial_id(double x, double y)
{
  const std::uint32_t ix = static_cast<std::uint32_t>(
    static_cast<std::int32_t>(std::floor(x * 8.0)));
  const std::uint32_t iy = static_cast<std::uint32_t>(
    static_cast<std::int32_t>(std::floor(y * 8.0)));
  return (static_cast<std::uint64_t>(ix) << 32) | iy;
}

double robust_residual(double r)
{
  const double a = std::abs(r);
  return a <= 0.055 ? sq(a) : 0.11 * a - sq(0.055);
}

std::pair<Vec3, double> fit_fixed_radius(
  const std::vector<WorldPoint> & points,
  const std::vector<std::size_t> & members,
  const Vec3 & sensor,
  double radius)
{
  double nearest = std::numeric_limits<double>::infinity();
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (std::size_t i : members) {
    nearest = std::min(nearest, points[i].range);
    mean_x += points[i].position.x;
    mean_y += points[i].position.y;
  }
  mean_x /= static_cast<double>(members.size());
  mean_y /= static_cast<double>(members.size());
  const double bearing = std::atan2(mean_y - sensor.y, mean_x - sensor.x);
  const double ux = std::cos(bearing);
  const double uy = std::sin(bearing);
  const double vx = -uy;
  const double vy = ux;
  const double nominal_r = nearest + radius;
  const double nominal_t = (mean_x - sensor.x) * vx + (mean_y - sensor.y) * vy;
  double best_loss = std::numeric_limits<double>::infinity();
  Vec3 best{mean_x, mean_y, 0.0};
  // Only a bounded neighborhood is searched. The score always includes every
  // point of the local object; no convenient subset can define a robot.
  for (int ri = -4; ri <= 4; ++ri) {
    for (int ti = -4; ti <= 4; ++ti) {
      const double cr = nominal_r + ri * 0.035;
      const double ct = nominal_t + ti * 0.035;
      const double cx = sensor.x + ux * cr + vx * ct;
      const double cy = sensor.y + uy * cr + vy * ct;
      double loss = 0.0;
      double weight = 0.0;
      for (std::size_t i : members) {
        const auto & p = points[i].position;
        const double residual = norm2(p.x - cx, p.y - cy) - radius;
        const double w = p.z >= 0.055 && p.z <= 0.28 ? 1.0 : 0.55;
        loss += w * robust_residual(residual);
        weight += w;
      }
      loss = loss / std::max(1.0, weight) +
        0.0004 * (sq(ri / 4.0) + sq(ti / 4.0));
      if (loss < best_loss) {
        best_loss = loss;
        best = {cx, cy, 0.0};
      }
    }
  }
  return {best, std::sqrt(best_loss)};
}

std::size_t count_through_rays(
  const std::vector<WorldPoint> & all,
  const Vec3 & sensor,
  const Vec3 & center,
  double radius,
  std::vector<Vec3> * contradictory,
  std::size_t limit)
{
  std::unordered_set<int> independent;
  const double ox = sensor.x - center.x;
  const double oy = sensor.y - center.y;
  for (const auto & point : all) {
    const auto & p = point.position;
    const double dx = p.x - sensor.x;
    const double dy = p.y - sensor.y;
    const double ray_length = norm2(dx, dy);
    if (ray_length < 0.01) {continue;}
    const double ux = dx / ray_length;
    const double uy = dy / ray_length;
    const double b = ox * ux + oy * uy;
    const double c = sq(ox) + sq(oy) - sq(radius);
    const double discriminant = sq(b) - c;
    if (discriminant <= 0.0) {continue;}
    const double entry = -b - std::sqrt(discriminant);
    const double exit = -b + std::sqrt(discriminant);
    if (entry < 0.05 || ray_length <= exit + 0.07) {continue;}
    const double crossing_z = sensor.z +
      (p.z - sensor.z) * entry / ray_length;
    // The low Kobuki base is opaque. Shelves and supports above it are not.
    if (crossing_z < 0.045 || crossing_z > 0.14) {continue;}
    const double angle = std::atan2(dy, dx);
    const int angular_bin = static_cast<int>(std::floor((angle + kPi) * 180.0 / kPi));
    const int height_bin = crossing_z < 0.085 ? 0 : 1;
    if (independent.insert(angular_bin * 2 + height_bin).second &&
      contradictory && contradictory->size() < limit)
    {
      contradictory->push_back(p);
    }
  }
  return independent.size();
}

Features describe_component(
  const std::vector<WorldPoint> & all,
  const std::vector<std::size_t> & members,
  const Vec3 & sensor,
  const Map * map,
  const Config & config,
  double pose_sigma)
{
  Features features;
  auto & out = features.candidate;
  out.point_count = members.size();
  double mean_x = 0.0;
  double mean_y = 0.0;
  double mean_z = 0.0;
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  std::size_t middle_layer = 0;
  std::size_t upper_layer = 0;
  for (std::size_t i : members) {
    const auto & p = all[i].position;
    mean_x += p.x;
    mean_y += p.y;
    mean_z += p.z;
    min_z = std::min(min_z, p.z);
    max_z = std::max(max_z, p.z);
    if (p.z >= 0.115 && p.z < 0.23) {++middle_layer;}
    if (p.z >= 0.23 && p.z < 0.50) {++upper_layer;}
  }
  const double n = static_cast<double>(members.size());
  out.middle_layer_fraction = static_cast<double>(middle_layer) / n;
  out.upper_layer_fraction = static_cast<double>(upper_layer) / n;
  out.middle_to_upper_ratio = static_cast<double>(middle_layer) /
    std::max<std::size_t>(1, middle_layer + upper_layer);
  mean_x /= n;
  mean_y /= n;
  mean_z /= n;
  features.top = max_z;
  features.bottom = min_z;
  out.height = max_z - min_z;
  const double bearing = std::atan2(mean_y - sensor.y, mean_x - sensor.x);
  const double ux = std::cos(bearing);
  const double uy = std::sin(bearing);
  const double vx = -uy;
  const double vy = ux;
  double min_long = std::numeric_limits<double>::infinity();
  double max_long = -std::numeric_limits<double>::infinity();
  double min_lat = std::numeric_limits<double>::infinity();
  double max_lat = -std::numeric_limits<double>::infinity();
  double cxx = 0.0;
  double cxy = 0.0;
  double cyy = 0.0;
  for (std::size_t i : members) {
    const auto & p = all[i].position;
    const double dx = p.x - mean_x;
    const double dy = p.y - mean_y;
    const double axial = dx * ux + dy * uy;
    const double lateral = dx * vx + dy * vy;
    min_long = std::min(min_long, axial);
    max_long = std::max(max_long, axial);
    min_lat = std::min(min_lat, lateral);
    max_lat = std::max(max_lat, lateral);
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }
  out.depth = max_long - min_long;
  out.width = max_lat - min_lat;
  const double eigen_angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
  const double px = std::cos(eigen_angle);
  const double py = std::sin(eigen_angle);
  double min_major = std::numeric_limits<double>::infinity();
  double max_major = -std::numeric_limits<double>::infinity();
  double min_minor = std::numeric_limits<double>::infinity();
  double max_minor = -std::numeric_limits<double>::infinity();
  double line_sq = 0.0;
  for (std::size_t i : members) {
    const auto & p = all[i].position;
    const double dx = p.x - mean_x;
    const double dy = p.y - mean_y;
    const double major = dx * px + dy * py;
    const double minor = -dx * py + dy * px;
    min_major = std::min(min_major, major);
    max_major = std::max(max_major, major);
    min_minor = std::min(min_minor, minor);
    max_minor = std::max(max_minor, minor);
    line_sq += minor * minor;
  }
  features.shape_length = max_major - min_major;
  features.shape_width = max_minor - min_minor;
  features.line_rms = std::sqrt(line_sq / n);

  const auto fit = fit_fixed_radius(all, members, sensor, config.robot_radius);
  out.center_map = fit.first;
  features.circle_rms = fit.second;
  out.id = spatial_id(out.center_map.x, out.center_map.y);
  std::unordered_set<int> angular;
  std::unordered_set<int> heights;
  std::unordered_set<std::uint64_t> unique_support;
  std::size_t support = 0;
  std::size_t interior = 0;
  std::size_t mapped = 0;
  const std::size_t stride = std::max<std::size_t>(
    1, members.size() / std::max<std::size_t>(1, config.max_marker_points));
  for (std::size_t m = 0; m < members.size(); ++m) {
    const auto & point = all[members[m]];
    const auto & p = point.position;
    const double distance = norm2(p.x - out.center_map.x, p.y - out.center_map.y);
    const double residual = std::abs(distance - config.robot_radius);
    const bool on_surface = residual < 0.07;
    if (m % stride == 0 && out.object_map.size() < config.max_marker_points) {
      out.object_map.push_back(p);
    }
    mapped += map && occupied_near(*map, p.x, p.y, 0.04) ? 1 : 0;
    if (distance < config.robot_radius - 0.09 && p.z >= 0.055 && p.z < 0.16) {
      ++interior;
      if (out.contradictory_map.size() < config.max_marker_points) {
        out.contradictory_map.push_back(p);
      }
    }
    if (on_surface) {
      ++support;
      const int a = static_cast<int>(std::floor(
        (std::atan2(p.y - out.center_map.y, p.x - out.center_map.x) + kPi) /
        (kPi / 12.0)));
      const int h = p.z < 0.15 ? 0 : (p.z < 0.30 ? 1 : 2);
      angular.insert(a);
      heights.insert(h);
      unique_support.insert(key(
        static_cast<int>(std::floor(p.x / 0.055)),
        static_cast<int>(std::floor(p.y / 0.055))));
      if (m % stride == 0 && out.support_map.size() < config.max_marker_points) {
        out.support_map.push_back(p);
      }
    }
  }
  out.support_count = support;
  out.angular_bins = angular.size();
  out.height_bins = heights.size();
  out.map_occupied_count = mapped;
  features.ring_fraction = static_cast<double>(support) / n;
  features.through_ray_bins = count_through_rays(
    all, sensor, out.center_map, config.robot_radius,
    &out.contradictory_map, config.max_marker_points);
  out.contradictory_count = interior + features.through_ray_bins;

  const bool mapped_center = map && occupied_near(*map, out.center_map.x,
      out.center_map.y, 0.02);
  const double static_share = static_cast<double>(mapped) / n;
  const double planar_margin = features.circle_rms - features.line_rms;
  const bool too_small = out.width < 0.16 && features.shape_length < 0.24;
  const bool too_large = out.width > 0.58 || out.depth > 0.60 ||
    features.shape_length > 0.68;
  const bool too_low = max_z < 0.24 || out.height < 0.13;
  const bool thin_face = features.shape_length > 0.24 &&
    features.line_rms < 0.026 && planar_margin > 0.010;
  const bool weak_cover = out.angular_bins < 3 || unique_support.size() < 5;
  const bool many_through = features.through_ray_bins >= 5 &&
    features.through_ray_bins > out.angular_bins;
  const bool interior_conflict = interior > std::max<std::size_t>(4, members.size() / 6);
  // The known low box is broad but short; the tall box is narrow. A tall,
  // broad, independently covered object is a second body explanation when an
  // elevated robot shelf makes the middle/upper density ratio misleading.
  // It must still pass the complete-object and free-ray checks.
  out.tall_broad_body = out.height >= 0.27 && out.height < 0.43 &&
    out.width >= 0.26 && out.width < 0.43 &&
    out.depth >= 0.35 && out.depth < 0.52 && members.size() >= 40 &&
    out.angular_bins >= 7 && out.contradictory_count <= 6 &&
    static_share < 0.25 && !mapped_center && !many_through &&
    !interior_conflict && !thin_face && features.circle_rms <= 0.08;

  // The model comparison is on the same complete connected object. These
  // normalized values are heuristic class scores, not calibrated posteriors.
  double robot_logit = 0.2;
  robot_logit += clamp((out.width - 0.19) / 0.065, -2.0, 1.4);
  robot_logit += clamp((max_z - 0.26) / 0.07, -2.0, 1.1);
  robot_logit += clamp((out.height - 0.15) / 0.09, -1.5, 0.9);
  robot_logit += clamp((features.ring_fraction - 0.58) * 2.7, -1.6, 1.0);
  robot_logit += clamp((out.angular_bins - 3.0) * 0.25, -0.8, 1.0);
  robot_logit -= clamp(features.circle_rms / 0.055, 0.0, 2.0);
  robot_logit -= too_large ? 4.0 : 0.0;
  robot_logit -= too_small ? 3.0 : 0.0;
  robot_logit -= too_low ? 3.0 : 0.0;
  robot_logit -= thin_face ? 2.3 : 0.0;
  robot_logit -= many_through ? 2.5 : 0.0;
  robot_logit -= interior_conflict ? 2.0 : 0.0;
  robot_logit -= mapped_center ? 3.0 : 0.0;
  robot_logit -= clamp(static_share * 3.0, 0.0, 2.0);
  robot_logit -= weak_cover ? 1.2 : 0.0;
  robot_logit -= members.size() < 12 ? 1.8 : 0.0;
  robot_logit += clamp(8.0 * (out.middle_to_upper_ratio - 0.65), -4.0, 2.0);
  robot_logit += out.tall_broad_body ? 1.8 : 0.0;
  const double box_logit = (too_small ? 2.2 : 0.0) +
    (too_low && features.shape_length > 0.28 ? 2.4 : 0.0) +
    (thin_face ? 1.8 : 0.0) +
    (features.shape_length > 0.39 && features.shape_width > 0.25 ? 0.8 : 0.0) +
    clamp(5.0 * (0.68 - out.middle_to_upper_ratio), -1.0, 2.0);
  const double wall_logit = (too_large ? 3.0 : 0.0) +
    (mapped_center ? 2.6 : 0.0) + 2.0 * static_share +
    (features.shape_length > 0.5 && features.line_rms < 0.04 ? 1.5 : 0.0);
  const double artifact_logit = (weak_cover ? 1.4 : 0.0) +
    (members.size() < 12 ? 1.8 : 0.0) +
    (many_through ? 1.5 : 0.0) +
    (interior_conflict ? 1.5 : 0.0);
  const double high = std::max({robot_logit, box_logit, wall_logit, artifact_logit});
  const double robot_exp = std::exp(robot_logit - high);
  const double box_exp = std::exp(box_logit - high);
  const double wall_exp = std::exp(wall_logit - high);
  const double artifact_exp = std::exp(artifact_logit - high);
  const double total = robot_exp + box_exp + wall_exp + artifact_exp;
  out.robot_probability = robot_exp / total;
  out.box_probability = box_exp / total;
  out.wall_probability = wall_exp / total;
  out.artifact_probability = artifact_exp / total;

  std::vector<std::string> reasons;
  if (members.size() < 12) {reasons.emplace_back("too_few_points");}
  if (too_small) {reasons.emplace_back("object_too_narrow");}
  if (too_large) {reasons.emplace_back("complete_object_too_large");}
  if (too_low) {reasons.emplace_back("insufficient_vertical_structure");}
  if (thin_face) {reasons.emplace_back("planar_face_beats_body");}
  if (weak_cover) {reasons.emplace_back("insufficient_independent_surface_coverage");}
  if (many_through) {reasons.emplace_back("free_rays_through_opaque_base");}
  if (interior_conflict) {reasons.emplace_back("returns_inside_opaque_base");}
  if (mapped_center || static_share > 0.25) {
    reasons.emplace_back("occupied_static_map");
  }
  if (features.circle_rms > 0.08) {reasons.emplace_back("poor_complete_body_fit");}
  if (out.middle_to_upper_ratio < 0.70) {
    reasons.emplace_back("upper_layer_dominates_body");
  }
  if (out.tall_broad_body) {reasons.emplace_back("tall_broad_body_support");}
  out.reason = reason_join(reasons);
  // A large pose covariance softens geometry but cannot silently shift the map.
  const double localization_var = sq(pose_sigma) + 0.0004;
  const double shape_var = sq(clamp(features.circle_rms, 0.025, 0.11));
  const double cover_var = sq(out.angular_bins < 5 ? 0.075 : 0.038);
  const double v = localization_var + shape_var + cover_var;
  out.position_covariance = {v, 0.0, 0.0, v};
  const bool body_geometry_viable = !too_large && !too_small && !too_low &&
    !mapped_center && !many_through && !interior_conflict &&
    out.angular_bins >= 3 && unique_support.size() >= 5;
  out.complete_body_viable = body_geometry_viable && !thin_face &&
    features.circle_rms <= 0.08 && static_share < 0.25;
  out.weak_body_observation = body_geometry_viable &&
    out.robot_probability >= 0.50 && out.middle_to_upper_ratio >= 0.40;
  // The upper shell can disappear behind an occluder. Such a fragment cannot
  // create a robot track; it may only update a recently confirmed one.
  out.continuation_body_observation = !too_large && !mapped_center &&
    !many_through && !interior_conflict && members.size() >= 18 &&
    out.width >= 0.18 && out.angular_bins >= 4 &&
    unique_support.size() >= 5 && features.circle_rms <= 0.08 &&
    out.middle_layer_fraction >= 0.50 &&
    out.middle_to_upper_ratio >= 0.75;
  if (((out.robot_probability >= 0.62 &&
    out.middle_to_upper_ratio >= 0.70) ||
    (out.robot_probability >= 0.40 && out.tall_broad_body)) &&
    body_geometry_viable)
  {
    out.decision = Decision::accepted;
  } else if (out.weak_body_observation)
  {
    out.decision = Decision::ambiguous;
  } else {
    out.decision = Decision::rejected;
  }
  return features;
}

}  // namespace

struct Detector::InternalTrack {
  std::uint64_t id = 0;
  Vec3 center;
  Vec3 velocity;
  double position_variance = 0.02;
  double velocity_variance = 1.0;
  double probability = 0.5;
  double last_stamp = 0.0;
  double last_measurement_stamp = 0.0;
  double last_strong_stamp = 0.0;
  int measured_hits = 0;
  int velocity_hits = 0;
  int consistent_velocity_hits = 0;
  bool confirmed = false;
  bool prediction_velocity_valid = false;
  bool velocity_valid = false;
  bool measured_this_frame = false;
  bool associated = false;
  std::deque<std::pair<double, Vec3>> measurements;
};

// This track is independent of the complete-body, static-capable tracker.
// It only confirms coherent target motion while the sensor itself is still.
// Its observations never count as strong hits for InternalTrack.
struct Detector::MotionTrack {
  struct Observation {
    double stamp = 0.0;
    Vec3 center;
    Vec3 sensor_pose;  // x, y, yaw
  };
  std::uint64_t id = 0;
  double last_stamp = 0.0;
  bool assigned = false;
  bool confirmed = false;
  std::deque<Observation> history;
};

Detector::Detector(Config config) : config_(std::move(config)) {}
Detector::~Detector() = default;
Detector::Detector(Detector &&) noexcept = default;
Detector & Detector::operator=(Detector &&) noexcept = default;

void Detector::reset()
{
  tracks_.clear();
  next_track_id_ = 1;
  motion_tracks_.clear();
  next_motion_id_ = 1;
  temporal_frames_.clear();
  temporal_point_count_ = 0;
  last_stamp_ = -1.0;
}

Result Detector::process(const Frame & frame)
{
  const auto start = Clock::now();
  Result result;
  result.stamp = frame.stamp;
  if (!finite(frame.stamp) || frame.stamp < 0.0) {
    result.status = "invalid_timestamp";
    return result;
  }
  if (!valid_transform(frame.map_from_sensor)) {
    result.status = "invalid_transform";
    return result;
  }
  if ((frame.map && !valid_map(frame.map)) ||
    (!frame.map && !config_.allow_mapless))
  {
    result.status = "invalid_map";
    return result;
  }
  if (!finite(frame.pose_var_x) || !finite(frame.pose_var_y) ||
    !finite(frame.pose_var_yaw) || frame.pose_var_x < 0.0 ||
    frame.pose_var_y < 0.0 || frame.pose_var_yaw < 0.0)
  {
    result.status = "invalid_pose_covariance";
    return result;
  }
  if (last_stamp_ >= 0.0 && frame.stamp <= last_stamp_) {
    result.status = "non_monotonic_scan";
    return result;
  }
  if (frame.points.empty()) {
    result.status = "empty_cloud";
    return result;
  }
  const double pose_sigma = std::sqrt(std::max(frame.pose_var_x, frame.pose_var_y));
  if (pose_sigma > 0.5 || std::sqrt(frame.pose_var_yaw) > 0.7) {
    result.status = "localization_too_uncertain";
    return result;
  }
  const Map * map = frame.map;
  const Vec3 sensor = frame.map_from_sensor.translation;
  std::vector<WorldPoint> all;
  all.reserve(frame.points.size());
  std::vector<std::size_t> foreground;
  foreground.reserve(frame.points.size() / 3);
  for (const auto & p : frame.points) {
    if (!finite(p.x) || !finite(p.y) || !finite(p.z)) {continue;}
    const Vec3 world = transform_point(frame.map_from_sensor, p);
    const double range = norm2(world.x - sensor.x, world.y - sensor.y);
    if (range < config_.min_range || range > config_.max_range ||
      world.z < config_.min_height || world.z > config_.max_height)
    {
      continue;
    }
    const bool mapped = map && occupied_near(*map, world.x, world.y, 0.055);
    all.push_back({world, range, mapped});
    if (!mapped) {foreground.push_back(all.size() - 1);}
  }
  const auto transformed = Clock::now();
  result.timing.transform_ms = ms(start, transformed);
  if (all.empty()) {
    result.status = "no_points_in_search_volume";
    last_stamp_ = frame.stamp;
    result.timing.total_ms = ms(start, Clock::now());
    return result;
  }
  if (map) {
    const auto mapped = std::count_if(all.begin(), all.end(),
      [](const WorldPoint & point) {return point.mapped;});
    result.map_alignment_share = static_cast<double>(mapped) / all.size();
    if (all.size() >= config_.min_map_alignment_points &&
      result.map_alignment_share < config_.min_map_alignment_share)
    {
      tracks_.clear();
      motion_tracks_.clear();
      temporal_frames_.clear();
      temporal_point_count_ = 0;
      result.status = "map_alignment_inconsistent";
      last_stamp_ = frame.stamp;
      result.timing.total_ms = ms(start, Clock::now());
      return result;
    }
  }

  const auto & q = frame.map_from_sensor;
  const double qnorm2 = sq(q.qx) + sq(q.qy) + sq(q.qz) + sq(q.qw);
  const double sensor_yaw = std::atan2(
    2.0 * (q.qw * q.qz + q.qx * q.qy) / qnorm2,
    1.0 - 2.0 * (sq(q.qy) + sq(q.qz)) / qnorm2);
  const Vec3 sensor_pose{sensor.x, sensor.y, sensor_yaw};

  std::vector<std::size_t> novel_foreground;
  const TemporalFrame * reference = nullptr;
  double nearest_lag_error = 0.20;
  for (const auto & prior : temporal_frames_) {
    const double lag = frame.stamp - prior.stamp;
    if (lag < 1.0 || lag > 1.4) {continue;}
    const double error = std::abs(lag - 1.2);
    if (error < nearest_lag_error) {
      nearest_lag_error = error;
      reference = &prior;
    }
  }
  if (reference &&
    norm2(sensor_pose.x - reference->sensor_pose.x,
      sensor_pose.y - reference->sensor_pose.y) <= 0.035 &&
    std::abs(std::remainder(sensor_pose.z - reference->sensor_pose.z,
      2.0 * kPi)) <= 0.06)
  {
    constexpr double cell_size = 0.08;
    std::unordered_map<std::uint64_t, std::vector<Vec3>> old_cells;
    old_cells.reserve(reference->points.size());
    for (const auto & p : reference->points) {
      old_cells[key3(static_cast<int>(std::floor(p.x / cell_size)),
        static_cast<int>(std::floor(p.y / cell_size)),
        static_cast<int>(std::floor(p.z / cell_size)))].push_back(p);
    }
    for (const std::size_t index : foreground) {
      const auto & p = all[index].position;
      const int ix = static_cast<int>(std::floor(p.x / cell_size));
      const int iy = static_cast<int>(std::floor(p.y / cell_size));
      const int iz = static_cast<int>(std::floor(p.z / cell_size));
      bool old_return = false;
      for (int dz = -1; dz <= 1 && !old_return; ++dz) {
        for (int dy = -1; dy <= 1 && !old_return; ++dy) {
          for (int dx = -1; dx <= 1 && !old_return; ++dx) {
            const auto found = old_cells.find(key3(ix + dx, iy + dy, iz + dz));
            if (found == old_cells.end()) {continue;}
            for (const auto & old : found->second) {
              if (sq(p.x - old.x) + sq(p.y - old.y) + sq(p.z - old.z) <= 0.0036) {
                old_return = true;
                break;
              }
            }
          }
        }
      }
      if (!old_return) {novel_foreground.push_back(index);}
    }
  }
  // A time bound alone is insufficient for dense or high-rate clouds. Missing
  // temporal history disables this optional proposal branch, never body gates.
  constexpr std::size_t max_temporal_points = 200000;
  constexpr std::size_t max_temporal_frames = 32;
  if (all.size() <= max_temporal_points) {
    TemporalFrame current_temporal;
    current_temporal.stamp = frame.stamp;
    current_temporal.sensor_pose = sensor_pose;
    current_temporal.points.reserve(all.size());
    for (const auto & p : all) {current_temporal.points.push_back(p.position);}
    temporal_point_count_ += current_temporal.points.size();
    temporal_frames_.push_back(std::move(current_temporal));
  }
  while (!temporal_frames_.empty() &&
    (frame.stamp - temporal_frames_.front().stamp > 1.5 ||
    temporal_frames_.size() > max_temporal_frames ||
    temporal_point_count_ > max_temporal_points))
  {
    temporal_point_count_ -= temporal_frames_.front().points.size();
    temporal_frames_.pop_front();
  }

  const auto segment = [&](const std::vector<std::size_t> & indices, double gap) {
      std::vector<Cell> cells;
      cells.reserve(indices.size());
      std::unordered_map<std::uint64_t, std::size_t> lookup;
      lookup.reserve(indices.size());
      for (std::size_t point_index : indices) {
        const auto & p = all[point_index].position;
        const int ix = static_cast<int>(std::floor(p.x / config_.voxel_size));
        const int iy = static_cast<int>(std::floor(p.y / config_.voxel_size));
        const auto k = key(ix, iy);
        auto found = lookup.find(k);
        if (found == lookup.end()) {
          found = lookup.emplace(k, cells.size()).first;
          cells.push_back({ix, iy, {}, false});
        }
        cells[found->second].points.push_back(point_index);
      }
      std::vector<std::vector<std::size_t>> output;
      for (std::size_t ci = 0; ci < cells.size(); ++ci) {
        if (cells[ci].visited) {continue;}
        cells[ci].visited = true;
        std::queue<std::size_t> pending;
        pending.push(ci);
        std::vector<std::size_t> members;
        while (!pending.empty()) {
          const std::size_t current = pending.front();
          pending.pop();
          const auto & cell = cells[current];
          members.insert(members.end(), cell.points.begin(), cell.points.end());
          for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
              if (dx == 0 && dy == 0) {continue;}
              if (norm2(dx * config_.voxel_size, dy * config_.voxel_size) >
                gap + config_.voxel_size * 0.5)
              {
                continue;
              }
              const auto neighbor = lookup.find(key(cell.x + dx, cell.y + dy));
              if (neighbor == lookup.end() || cells[neighbor->second].visited) {continue;}
              cells[neighbor->second].visited = true;
              pending.push(neighbor->second);
            }
          }
        }
        if (members.size() >= 7) {output.push_back(std::move(members));}
      }
      return output;
    };
  auto components = segment(foreground, config_.cluster_gap);
  auto changed_components = segment(novel_foreground, 0.07);
  // Changed returns propose a location. Score the whole current connected
  // object at the same finer connectivity, including unchanged and mapped
  // returns. A convenient changed arc cannot stand in for a larger wall/box.
  std::vector<std::vector<std::size_t>> complete_components;
  std::vector<std::size_t> complete_membership(all.size(), all.size());
  if (!changed_components.empty()) {
    std::vector<std::size_t> indices(all.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {indices[i] = i;}
    complete_components = segment(indices, 0.07);
    for (std::size_t ci = 0; ci < complete_components.size(); ++ci) {
      for (const auto index : complete_components[ci]) {complete_membership[index] = ci;}
    }
  }
  const auto segmented = Clock::now();
  result.timing.segmentation_ms = ms(transformed, segmented);

  for (const auto & members : components) {
    auto feature = describe_component(all, members, sensor, map, config_, pose_sigma);
    result.candidates.push_back(std::move(feature.candidate));
  }
  std::sort(result.candidates.begin(), result.candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      return a.robot_probability > b.robot_probability;
    });
  // Debug messages must stay bounded even if a map mismatch creates hundreds
  // of components. Keep all plausible objects and the most explanatory rejects.
  if (result.candidates.size() > 32) {result.candidates.resize(32);}
  const auto scored = Clock::now();
  result.timing.scoring_ms = ms(segmented, scored);

  for (auto & track : tracks_) {
    track.associated = false;
    track.measured_this_frame = false;
    const double dt = clamp(frame.stamp - track.last_stamp, 0.0, 1.0);
    const double predict_dt = track.prediction_velocity_valid ? std::min(dt, 0.25) : 0.0;
    track.center.x += track.velocity.x * predict_dt;
    track.center.y += track.velocity.y * predict_dt;
    track.position_variance += 0.006 * dt + track.velocity_variance * sq(predict_dt);
    track.last_stamp = frame.stamp;
    if (frame.stamp - track.last_measurement_stamp > config_.max_track_gap_s) {
      track.confirmed = false;
    }
  }

  Track best_previous_prediction;
  bool had_previous_prediction = false;
  for (const auto & track : tracks_) {
    if (track.confirmed &&
      (!had_previous_prediction || track.probability > best_previous_prediction.probability))
    {
      best_previous_prediction = {
        track.id, track.center, track.velocity,
        {track.position_variance, 0.0, 0.0, track.position_variance},
        track.velocity_variance, track.probability,
        frame.stamp - track.last_measurement_stamp, track.measured_hits,
        track.confirmed, track.velocity_valid, false};
      had_previous_prediction = true;
    }
  }

  for (auto & candidate : result.candidates) {
    if ((candidate.decision == Decision::rejected &&
      !candidate.continuation_body_observation) ||
      (candidate.robot_probability < 0.35 &&
      !candidate.continuation_body_observation))
    {
      continue;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    InternalTrack * match = nullptr;
    for (auto & track : tracks_) {
      if (track.associated ||
        frame.stamp - track.last_measurement_stamp > config_.max_track_age_s)
      {
        continue;
      }
      if (candidate.decision == Decision::rejected &&
        (!track.confirmed || frame.stamp - track.last_strong_stamp > 1.00))
      {
        continue;
      }
      const double distance = norm2(
        candidate.center_map.x - track.center.x,
        candidate.center_map.y - track.center.y);
      const double gate = clamp(
        0.25 + 2.0 * std::sqrt(track.position_variance +
          candidate.position_covariance[0]), 0.32, 0.80);
      if (distance < gate && distance < best_distance) {
        match = &track;
        best_distance = distance;
      }
    }
    if (!match) {
      if (candidate.decision != Decision::accepted) {continue;}
      InternalTrack track;
      track.id = next_track_id_++;
      track.center = candidate.center_map;
      track.position_variance = candidate.position_covariance[0];
      track.probability = candidate.robot_probability;
      track.last_stamp = frame.stamp;
      track.last_measurement_stamp = frame.stamp;
      track.last_strong_stamp = frame.stamp;
      track.measured_hits = 1;
      track.measured_this_frame = true;
      track.associated = true;
      track.measurements.emplace_back(frame.stamp, candidate.center_map);
      tracks_.push_back(track);
      candidate.track_id = track.id;
      continue;
    }
    match->associated = true;
    candidate.track_id = match->id;
    // An unconfirmed object still needs strong body evidence. Once confirmed,
    // a weak but geometrically compatible return is a current measurement.
    // It is never emitted as a coasted prediction.
    if (candidate.decision != Decision::accepted &&
      !(match->confirmed && frame.stamp - match->last_strong_stamp <= 1.00 &&
      (candidate.weak_body_observation ||
      candidate.continuation_body_observation)))
    {
      match->probability *= 0.90;
      continue;
    }
    const double observation_dt = frame.stamp - match->last_measurement_stamp;
    const double measurement_var = candidate.position_covariance[0];
    const double gain = clamp(
      match->position_variance / (match->position_variance + measurement_var),
      0.25, 0.80);
    match->center.x += gain * (candidate.center_map.x - match->center.x);
    match->center.y += gain * (candidate.center_map.y - match->center.y);
    match->position_variance =
      (1.0 - gain) * match->position_variance + 0.001;
    if (observation_dt > config_.max_track_gap_s) {
      match->measurements.clear();
      match->velocity_hits = 0;
      match->consistent_velocity_hits = 0;
      match->prediction_velocity_valid = false;
      match->velocity_valid = false;
      match->velocity = {};
    }
    match->measurements.emplace_back(frame.stamp, candidate.center_map);
    while (!match->measurements.empty() &&
      frame.stamp - match->measurements.front().first > 1.20)
    {
      match->measurements.pop_front();
    }
    // Keep the existing short-window velocity estimate, but do not publish it
    // merely because a regression can be fitted. A static body's observed
    // centroid may slide during partial occlusion or a change of viewpoint.
    // Sustained motion must be significant against the observed scatter and
    // have compatible slopes in both halves of a longer window.
    struct VelocityFit {
      double vx = 0.0;
      double vy = 0.0;
      double tt = 0.0;
      double residual_rms = 0.0;
    };
    const auto fit_velocity = [&](std::size_t begin, std::size_t end) {
        VelocityFit fit;
        const double n = static_cast<double>(end - begin);
        double mean_t = 0.0;
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
          const auto & m = match->measurements[i];
          mean_t += m.first;
          mean_x += m.second.x;
          mean_y += m.second.y;
        }
        mean_t /= n;
        mean_x /= n;
        mean_y /= n;
        double tx = 0.0;
        double ty = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
          const auto & m = match->measurements[i];
          const double dt = m.first - mean_t;
          fit.tt += dt * dt;
          tx += dt * (m.second.x - mean_x);
          ty += dt * (m.second.y - mean_y);
        }
        if (fit.tt <= 0.0) {return fit;}
        fit.vx = tx / fit.tt;
        fit.vy = ty / fit.tt;
        double residual_sum = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
          const auto & m = match->measurements[i];
          const double dt = m.first - mean_t;
          residual_sum += sq(m.second.x - mean_x - fit.vx * dt) +
            sq(m.second.y - mean_y - fit.vy * dt);
        }
        fit.residual_rms = std::sqrt(residual_sum / std::max(n - 2.0, 1.0));
        return fit;
      };
    const auto & history = match->measurements;
    std::size_t short_begin = 0;
    while (short_begin < history.size() &&
      frame.stamp - history[short_begin].first > 0.85)
    {
      ++short_begin;
    }
    if (history.size() - short_begin >= 4 &&
      frame.stamp - history[short_begin].first >= 0.28)
    {
      const auto fit = fit_velocity(short_begin, history.size());
      if (fit.tt > 0.03 && norm2(fit.vx, fit.vy) <= 3.5) {
        const double alpha = match->velocity_hits == 0 ? 1.0 : 0.4;
        match->velocity.x = (1.0 - alpha) * match->velocity.x + alpha * fit.vx;
        match->velocity.y = (1.0 - alpha) * match->velocity.y + alpha * fit.vy;
        match->velocity_variance = clamp(measurement_var / fit.tt, 0.02, 9.0);
        ++match->velocity_hits;
        match->prediction_velocity_valid = match->velocity_hits >= 2 &&
          match->velocity_variance < 0.6;
      }
    }
    bool consistent_motion = false;
    if (history.size() >= 6 && frame.stamp - history.front().first >= 0.90) {
      const auto fit = fit_velocity(0, history.size());
      const double midpoint = 0.5 * (history.front().first + history.back().first);
      std::size_t split = 0;
      while (split < history.size() && history[split].first <= midpoint) {++split;}
      if (fit.tt > 0.10 && split >= 3 && history.size() - split >= 3) {
        const auto first = fit_velocity(0, split);
        const auto second = fit_velocity(split, history.size());
        const double speed = norm2(fit.vx, fit.vy);
        const double first_speed = norm2(first.vx, first.vy);
        const double second_speed = norm2(second.vx, second.vy);
        // The 2.5 cm floor is below a 4.5 cm map voxel and prevents a nearly
        // exact fit through very short stationary centroid fluctuations.
        const double slope_error = std::max(fit.residual_rms, 0.025) /
          std::sqrt(fit.tt);
        consistent_motion = speed > 3.0 * slope_error &&
          first.vx * second.vx + first.vy * second.vy >
          0.80 * first_speed * second_speed &&
          std::min(first_speed, second_speed) >
          0.45 * std::max(first_speed, second_speed);
      }
    }
    match->consistent_velocity_hits = consistent_motion ?
      match->consistent_velocity_hits + 1 : 0;
    match->velocity_valid = match->prediction_velocity_valid &&
      match->consistent_velocity_hits >= 2;
    match->last_measurement_stamp = frame.stamp;
    match->measured_this_frame = true;
    if (candidate.decision == Decision::accepted) {
      match->last_strong_stamp = frame.stamp;
      match->measured_hits = observation_dt <= config_.max_track_gap_s ?
        match->measured_hits + 1 : 1;
    }
    const double score_weight = candidate.decision == Decision::accepted ?
      0.55 : 0.08;
    match->probability = clamp(
      (1.0 - score_weight) * match->probability +
      score_weight * candidate.robot_probability, 0.0, 1.0);
    match->confirmed = match->measured_hits >= config_.confirmation_hits &&
      match->probability >= 0.56 &&
      frame.stamp - match->last_strong_stamp <= 1.00;
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
    [&](const InternalTrack & track) {
      return frame.stamp - track.last_measurement_stamp > config_.max_track_age_s;
    }), tracks_.end());
  const InternalTrack * best = nullptr;
  for (const auto & track : tracks_) {
    Track public_track{
      track.id, track.center, track.velocity,
      {track.position_variance, 0.0, 0.0, track.position_variance},
      track.velocity_variance, track.probability,
      frame.stamp - track.last_measurement_stamp, track.measured_hits,
      track.confirmed, track.velocity_valid, track.measured_this_frame};
    result.tracks.push_back(public_track);
    if (track.confirmed && track.measured_this_frame &&
      (!best || track.probability > best->probability))
    {
      best = &track;
    }
  }
  if (best) {
    result.has_measurement = true;
    result.measurement = {
      best->id, best->center, best->velocity,
      {best->position_variance, 0.0, 0.0, best->position_variance},
      best->velocity_variance, best->probability,
      0.0, best->measured_hits, best->confirmed, best->velocity_valid, true};
  }
  if (had_previous_prediction) {
    result.has_prediction = true;
    result.prediction = best_previous_prediction;
  }

  std::vector<Candidate> temporal_candidates;
  std::unordered_set<std::size_t> scored_complete;
  for (const auto & members : changed_components) {
    if (members.size() < 18 || members.size() > 300) {continue;}
    const auto ci = complete_membership[members.front()];
    if (ci >= complete_components.size() || !scored_complete.insert(ci).second) {continue;}
    auto candidate = describe_component(
      all, complete_components[ci], sensor, map, config_, pose_sigma).candidate;
    if (candidate.width < 0.26 || candidate.width > 0.55 ||
      candidate.depth < 0.14 || candidate.depth > 0.55 ||
      candidate.height < 0.19 || candidate.height > 0.43 ||
      candidate.support_count < 10 || candidate.angular_bins < 4 ||
      candidate.robot_probability < 0.35 || !candidate.complete_body_viable ||
      candidate.robot_probability < std::max({candidate.box_probability,
      candidate.wall_probability, candidate.artifact_probability}))
    {
      continue;
    }
    bool already_represented = false;
    for (const auto & existing : result.candidates) {
      if (existing.robot_probability >= 0.65 &&
        (existing.decision == Decision::accepted ||
        existing.decision == Decision::ambiguous) &&
        existing.width >= 0.26 && existing.width <= 0.39 &&
        existing.depth >= 0.15 && existing.depth <= 0.53 &&
        existing.height >= 0.25 && existing.height <= 0.43 &&
        norm2(existing.center_map.x - candidate.center_map.x,
          existing.center_map.y - candidate.center_map.y) < 0.25)
      {
        already_represented = true;
        break;
      }
    }
    if (already_represented) {continue;}
    candidate.temporal_change = true;
    candidate.reason = "temporal_change;" + candidate.reason;
    temporal_candidates.push_back(std::move(candidate));
  }
  std::sort(temporal_candidates.begin(), temporal_candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      return a.robot_probability > b.robot_probability;
    });
  for (std::size_t i = 0; i < std::min<std::size_t>(16, temporal_candidates.size()); ++i) {
    result.candidates.push_back(std::move(temporal_candidates[i]));
  }

  motion_tracks_.erase(std::remove_if(motion_tracks_.begin(), motion_tracks_.end(),
    [&](const MotionTrack & track) {
      return frame.stamp - track.last_stamp > config_.max_track_age_s;
    }), motion_tracks_.end());
  for (auto & track : motion_tracks_) {track.assigned = false;}
  Track motion_measurement;
  bool has_motion_measurement = false;
  Candidate * selected_motion_candidate = nullptr;
  for (auto & candidate : result.candidates) {
    // Upper returns can dominate the moving robot's body, but that geometry
    // also appears on boxes. Do not weaken the ordinary static-body gate.
    const bool temporal_shape = candidate.temporal_change &&
      candidate.complete_body_viable &&
      candidate.robot_probability >= std::max({candidate.box_probability,
      candidate.wall_probability, candidate.artifact_probability}) &&
      candidate.robot_probability >= 0.35 &&
      candidate.width >= 0.26 && candidate.width <= 0.55 &&
      candidate.depth >= 0.14 && candidate.depth <= 0.55 &&
      candidate.height >= 0.19 && candidate.height <= 0.43 &&
      candidate.angular_bins >= 4 && candidate.support_count >= 10;
    const bool ordinary_shape =
      (candidate.decision == Decision::accepted ||
      candidate.decision == Decision::ambiguous) &&
      candidate.robot_probability >= 0.65 &&
      (candidate.weak_body_observation || candidate.decision == Decision::accepted) &&
      candidate.width >= 0.26 && candidate.width <= 0.39 &&
      candidate.depth >= 0.15 && candidate.depth <= 0.53 &&
      candidate.height >= 0.25 && candidate.height <= 0.43 &&
      candidate.angular_bins >= 5 && candidate.support_count >= 20;
    if (!temporal_shape && !ordinary_shape)
    {
      continue;
    }
    MotionTrack * match = nullptr;
    // Keep the gate wide enough for a fast target, while rechecking its full
    // motion fit on every observation to reject a jump onto a nearby box.
    double best_distance = 0.28;
    for (auto & track : motion_tracks_) {
      if (track.assigned || frame.stamp - track.last_stamp > 0.55) {continue;}
      const auto & last = track.history.back().center;
      const double distance = norm2(
        candidate.center_map.x - last.x, candidate.center_map.y - last.y);
      if (distance < best_distance) {
        best_distance = distance;
        match = &track;
      }
    }
    if (!match) {
      MotionTrack track;
      // Keep motion-only IDs distinct from the ordinary track IDs and within
      // the range used by the ROS marker publisher.
      track.id = 1000000000ULL + next_motion_id_++;
      motion_tracks_.push_back(std::move(track));
      match = &motion_tracks_.back();
    }
    match->assigned = true;
    match->last_stamp = frame.stamp;
    match->history.push_back({frame.stamp, candidate.center_map, sensor_pose});
    while (!match->history.empty() &&
      frame.stamp - match->history.front().stamp > 1.65)
    {
      match->history.pop_front();
    }
    const auto & history = match->history;
    const double span = history.back().stamp - history.front().stamp;
    // Revalidate on every current observation: a previously moving object
    // cannot keep the motion-only confirmation after ego motion, a changed
    // candidate, or a trajectory that no longer supports motion.
    match->confirmed = false;
    if (history.size() >= 8 && span >= 0.80) {
      const auto & initial_sensor = history.front().sensor_pose;
      bool sensor_stationary = true;
      for (const auto & observation : history) {
        const auto & ego = observation.sensor_pose;
        if (norm2(ego.x - initial_sensor.x, ego.y - initial_sensor.y) > 0.035 ||
          std::abs(std::remainder(ego.z - initial_sensor.z, 2.0 * kPi)) > 0.06)
        {
          sensor_stationary = false;
          break;
        }
      }
      if (sensor_stationary) {
        double mean_t = 0.0;
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto & observation : history) {
          mean_t += observation.stamp - history.front().stamp;
          mean_x += observation.center.x;
          mean_y += observation.center.y;
        }
        const double count = static_cast<double>(history.size());
        mean_t /= count;
        mean_x /= count;
        mean_y /= count;
        double tt = 0.0;
        double tx = 0.0;
        double ty = 0.0;
        double total_variance = 0.0;
        for (const auto & observation : history) {
          const double dt = observation.stamp - history.front().stamp - mean_t;
          const double dx = observation.center.x - mean_x;
          const double dy = observation.center.y - mean_y;
          tt += dt * dt;
          tx += dt * dx;
          ty += dt * dy;
          total_variance += dx * dx + dy * dy;
        }
        if (tt > 0.0 && total_variance > 0.0) {
          const double vx = tx / tt;
          const double vy = ty / tt;
          double residual = 0.0;
          for (const auto & observation : history) {
            const double dt = observation.stamp - history.front().stamp - mean_t;
            residual += sq(observation.center.x - mean_x - vx * dt) +
              sq(observation.center.y - mean_y - vy * dt);
          }
          const double explained_fraction = 1.0 - residual / total_variance;
          match->confirmed = norm2(vx, vy) * span >= 0.10 &&
            explained_fraction >= 0.60;
        }
      }
    }
    if (match->confirmed && !result.has_measurement &&
      (!has_motion_measurement ||
      candidate.robot_probability > motion_measurement.probability))
    {
      motion_measurement = {
        match->id, candidate.center_map, {}, candidate.position_covariance,
        9.0, candidate.robot_probability, 0.0,
        static_cast<int>(history.size()), true, false, true};
      has_motion_measurement = true;
      selected_motion_candidate = &candidate;
    }
  }
  if (has_motion_measurement) {
    // Link the selected current-cloud evidence to its motion-only track in
    // diagnostics without changing the ordinary track association.
    selected_motion_candidate->track_id = motion_measurement.id;
    result.has_measurement = true;
    result.measurement = motion_measurement;
    result.tracks.push_back(motion_measurement);
  }
  result.status = result.has_measurement ? "measured_robot" :
    (result.candidates.empty() ? "no_foreground_objects" : "no_confirmed_robot");
  last_stamp_ = frame.stamp;
  const auto done = Clock::now();
  result.timing.tracking_ms = ms(scored, done);
  result.timing.total_ms = ms(start, done);
  return result;
}

}  // namespace jr_perception::opponent_core
