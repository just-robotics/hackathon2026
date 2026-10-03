// Standalone real-bag cache replay. No ROS, DDS, simulator, or generated clouds.
// Build with g++ -std=c++17 -O3 -Isrc/jr_perception/include, both this file
// and src/jr_perception/src/opponent_core.cpp, plus -lzstd.
// Usage: /tmp/opponent_offline CACHE_SESSION_DIR OUTPUT.jsonl
//        [filtered|raw] [--mapless] [--deskew] [--confirmation-hits N]
//        [--cluster-gap METERS]
#include "jr_perception/opponent_core.hpp"

#include <zstd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace core = jr_perception::opponent_core;

namespace {

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (true) {
    const auto end = line.find('\t', start);
    if (end == std::string::npos) {
      parts.push_back(line.substr(start));
      return parts;
    }
    parts.push_back(line.substr(start, end - start));
    start = end + 1;
  }
}

struct Tsv {
  std::ifstream file;
  std::unordered_map<std::string, std::size_t> columns;

  explicit Tsv(const std::string & path) : file(path)
  {
    if (!file) {throw std::runtime_error("cannot open " + path);}
    std::string header;
    std::getline(file, header);
    const auto names = split(header);
    for (std::size_t i = 0; i < names.size(); ++i) {columns[names[i]] = i;}
  }

  std::string get(const std::vector<std::string> & row, const std::string & name) const
  {
    const auto it = columns.find(name);
    if (it == columns.end() || it->second >= row.size()) {
      throw std::runtime_error("TSV column missing: " + name);
    }
    return row[it->second];
  }
};

double number(const std::string & value)
{
  if (value.empty()) {throw std::runtime_error("empty numeric TSV value");}
  return std::stod(value);
}

std::uint64_t integer(const std::string & value)
{
  if (value.empty()) {throw std::runtime_error("empty integer TSV value");}
  return std::stoull(value);
}

void quoted(std::ostream & out, const std::string & value)
{
  out << '"';
  for (const char ch : value) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;}
    else if (ch == '\n') {out << "\\n";}
    else if (ch == '\r') {out << "\\r";}
    else if (ch == '\t') {out << "\\t";}
    else if (static_cast<unsigned char>(ch) < 0x20) {out << ' ';}
    else {out << ch;}
  }
  out << '"';
}

float read_float(const std::uint8_t * bytes)
{
  float value;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

double read_double(const std::uint8_t * bytes)
{
  double value;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

struct PoseSample {
  std::uint64_t stamp_ns = 0;
  core::Transform pose;
};

core::Vec3 rotate(const core::Transform & t, const core::Vec3 & p)
{
  const double ux = t.qy * p.z - t.qz * p.y;
  const double uy = t.qz * p.x - t.qx * p.z;
  const double uz = t.qx * p.y - t.qy * p.x;
  const double vx = t.qy * uz - t.qz * uy;
  const double vy = t.qz * ux - t.qx * uz;
  const double vz = t.qx * uy - t.qy * ux;
  return {p.x + 2.0 * (t.qw * ux + vx),
    p.y + 2.0 * (t.qw * uy + vy), p.z + 2.0 * (t.qw * uz + vz)};
}

core::Transform interpolate_transform(
  const core::Transform & a, const core::Transform & b, double part)
{
  core::Transform out;
  out.translation = {
    a.translation.x + part * (b.translation.x - a.translation.x),
    a.translation.y + part * (b.translation.y - a.translation.y),
    a.translation.z + part * (b.translation.z - a.translation.z)};
  std::array<double, 4> qa{a.qx, a.qy, a.qz, a.qw};
  std::array<double, 4> qb{b.qx, b.qy, b.qz, b.qw};
  double dot = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {dot += qa[i] * qb[i];}
  if (dot < 0.0) {
    for (double & v : qb) {v = -v;}
    dot = -dot;
  }
  double left = 1.0 - part;
  double right = part;
  if (dot < 0.9995) {
    const double angle = std::acos(std::clamp(dot, -1.0, 1.0));
    const double scale = 1.0 / std::sin(angle);
    left = std::sin((1.0 - part) * angle) * scale;
    right = std::sin(part * angle) * scale;
  }
  std::array<double, 4> q{};
  double norm_sq = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    q[i] = left * qa[i] + right * qb[i];
    norm_sq += q[i] * q[i];
  }
  const double inv = 1.0 / std::sqrt(norm_sq);
  out.qx = q[0] * inv; out.qy = q[1] * inv;
  out.qz = q[2] * inv; out.qw = q[3] * inv;
  return out;
}

std::optional<core::Transform> pose_at(
  const std::vector<PoseSample> & poses, std::uint64_t stamp_ns)
{
  const auto after = std::lower_bound(poses.begin(), poses.end(), stamp_ns,
    [](const PoseSample & item, std::uint64_t ns) {return item.stamp_ns < ns;});
  if (after != poses.end() && after->stamp_ns == stamp_ns) {return after->pose;}
  if (after == poses.begin() || after == poses.end()) {return std::nullopt;}
  const auto & before = *(after - 1);
  const auto gap = after->stamp_ns - before.stamp_ns;
  if (gap == 0 || gap > 200000000ULL) {return std::nullopt;}
  const double part = static_cast<double>(stamp_ns - before.stamp_ns) /
    static_cast<double>(gap);
  return interpolate_transform(before.pose, after->pose, part);
}

core::Point point_at_header(
  const core::Point & point, const core::Transform & acquisition,
  const core::Transform & header)
{
  const auto world_delta = rotate(acquisition, {point.x, point.y, point.z});
  core::Transform inverse = header;
  inverse.qx = -header.qx; inverse.qy = -header.qy; inverse.qz = -header.qz;
  const auto at_header = rotate(inverse, {
    acquisition.translation.x + world_delta.x - header.translation.x,
    acquisition.translation.y + world_delta.y - header.translation.y,
    acquisition.translation.z + world_delta.z - header.translation.z});
  core::Point out = point;
  out.x = static_cast<float>(at_header.x);
  out.y = static_cast<float>(at_header.y);
  out.z = static_cast<float>(at_header.z);
  return out;
}

void xy(std::ostream & out, const core::Vec3 & point)
{
  out << '[' << point.x << ',' << point.y << ']';
}

void sample_xyz(std::ostream & out, const std::vector<core::Vec3> & sample)
{
  out << '[';
  for (std::size_t i = 0; i < sample.size(); ++i) {
    if (i) {out << ',';}
    out << '[' << sample[i].x << ',' << sample[i].y << ',' << sample[i].z << ']';
  }
  out << ']';
}

void write_result(
  std::ostream & out,
  const std::string & bag,
  const std::string & topic,
  const std::string & world_frame,
  bool deskew,
  std::uint64_t record_index,
  std::uint64_t stamp_ns,
  std::size_t input_points,
  const core::Result & result)
{
  out << std::setprecision(17);
  out << "{\"bag\":";
  quoted(out, bag);
  out << ",\"topic\":";
  quoted(out, topic);
  out << ",\"world_frame\":";
  quoted(out, world_frame);
  out << ",\"point_time_mode\":";
  quoted(out, deskew ? "bucketed_15ms_from_cached_header_poses" : "cloud_header_pose");
  out << ",\"record_index\":" << record_index;
  out << ",\"stamp_s\":" << result.stamp;
  out << ",\"stamp_ns\":" << stamp_ns;
  out << ",\"status\":";
  quoted(out, result.status);
  out << ",\"map_alignment_share\":" << result.map_alignment_share;
  out << ",\"input_points\":" << input_points;
  out << ",\"has_measurement\":" << (result.has_measurement ? "true" : "false");
  out << ",\"measurement_xy\":";
  if (result.has_measurement) {xy(out, result.measurement.center_map);}
  else {out << "null";}
  out << ",\"track_id\":" << (result.has_measurement ? result.measurement.id : 0);
  out << ",\"velocity_xy\":";
  if (result.has_measurement && result.measurement.velocity_valid) {
    xy(out, result.measurement.velocity_map);
  } else {out << "null";}
  out << ",\"velocity_valid\":" <<
    (result.has_measurement && result.measurement.velocity_valid ? "true" : "false");
  out << ",\"position_variance\":" <<
    (result.has_measurement ? result.measurement.position_covariance[0] : 0.0);
  out << ",\"has_prediction\":" << (result.has_prediction ? "true" : "false");
  out << ",\"prediction_xy\":";
  if (result.has_prediction) {xy(out, result.prediction.center_map);}
  else {out << "null";}
  out << ",\"prediction_age_s\":" <<
    (result.has_prediction ? result.prediction.age_since_measurement : 0.0);
  out << ",\"timing_ms\":" << result.timing.total_ms;
  out << ",\"score_semantics\":\"normalized_heuristic_not_calibrated\"";
  out << ",\"timing\":{\"transform_ms\":" << result.timing.transform_ms <<
    ",\"segmentation_ms\":" << result.timing.segmentation_ms <<
    ",\"scoring_ms\":" << result.timing.scoring_ms <<
    ",\"tracking_ms\":" << result.timing.tracking_ms << '}';
  out << ",\"candidates\":[";
  for (std::size_t i = 0; i < result.candidates.size(); ++i) {
    const auto & candidate = result.candidates[i];
    if (i) {out << ',';}
    out << "{\"id\":" << candidate.id << ",\"track_id\":" << candidate.track_id;
    out << ",\"decision\":";
    quoted(out, candidate.decision == core::Decision::accepted ? "accepted" :
      candidate.decision == core::Decision::ambiguous ? "ambiguous" : "rejected");
    out << ",\"reason\":";
    quoted(out, candidate.reason);
    out << ",\"center_xy\":";
    xy(out, candidate.center_map);
    out << ",\"robot_probability\":" << candidate.robot_probability <<
      ",\"box_probability\":" << candidate.box_probability <<
      ",\"wall_probability\":" << candidate.wall_probability <<
      ",\"artifact_probability\":" << candidate.artifact_probability <<
      ",\"width\":" << candidate.width <<
      ",\"depth\":" << candidate.depth <<
      ",\"height\":" << candidate.height <<
      ",\"point_count\":" << candidate.point_count <<
      ",\"support_count\":" << candidate.support_count <<
      ",\"contradictory_count\":" << candidate.contradictory_count <<
      ",\"map_occupied_count\":" << candidate.map_occupied_count <<
      ",\"angular_bins\":" << candidate.angular_bins <<
      ",\"height_bins\":" << candidate.height_bins <<
      ",\"middle_layer_fraction\":" << candidate.middle_layer_fraction <<
      ",\"upper_layer_fraction\":" << candidate.upper_layer_fraction <<
      ",\"middle_to_upper_ratio\":" << candidate.middle_to_upper_ratio <<
      ",\"weak_body_observation\":" <<
      (candidate.weak_body_observation ? "true" : "false") <<
      ",\"continuation_body_observation\":" <<
      (candidate.continuation_body_observation ? "true" : "false") <<
      ",\"tall_broad_body\":" <<
      (candidate.tall_broad_body ? "true" : "false") <<
      ",\"temporal_change\":" <<
      (candidate.temporal_change ? "true" : "false") <<
      ",\"complete_body_viable\":" <<
      (candidate.complete_body_viable ? "true" : "false") <<
      ",\"position_variance\":" << candidate.position_covariance[0];
    out << ",\"object_sample_xyz\":";
    sample_xyz(out, candidate.object_map);
    out << ",\"support_sample_xyz\":";
    sample_xyz(out, candidate.support_map);
    out << ",\"contradictory_sample_xyz\":";
    sample_xyz(out, candidate.contradictory_map);
    out << '}';
  }
  out << "],\"tracks\":[";
  for (std::size_t i = 0; i < result.tracks.size(); ++i) {
    const auto & track = result.tracks[i];
    if (i) {out << ',';}
    out << "{\"id\":" << track.id << ",\"center_xy\":";
    xy(out, track.center_map);
    out << ",\"probability\":" << track.probability <<
      ",\"age_s\":" << track.age_since_measurement <<
      ",\"measured_hits\":" << track.measured_hits <<
      ",\"confirmed\":" << (track.confirmed ? "true" : "false") <<
      ",\"measured_this_frame\":" << (track.measured_this_frame ? "true" : "false") <<
      ",\"velocity_valid\":" << (track.velocity_valid ? "true" : "false") << '}';
  }
  out << "]}\n";
}

core::Map read_map(const std::string & dir)
{
  Tsv tsv(dir + "/map.tsv");
  std::string line;
  if (!std::getline(tsv.file, line)) {throw std::runtime_error("map.tsv has no row");}
  const auto row = split(line);
  core::Map map;
  map.width = integer(tsv.get(row, "width"));
  map.height = integer(tsv.get(row, "height"));
  map.resolution = number(tsv.get(row, "resolution"));
  map.origin_x = number(tsv.get(row, "origin_x"));
  map.origin_y = number(tsv.get(row, "origin_y"));
  const double qx = number(tsv.get(row, "origin_qx"));
  const double qy = number(tsv.get(row, "origin_qy"));
  const double qz = number(tsv.get(row, "origin_qz"));
  const double qw = number(tsv.get(row, "origin_qw"));
  map.origin_yaw = std::atan2(2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));
  std::ifstream binary(dir + "/map.bin", std::ios::binary);
  if (!binary) {throw std::runtime_error("cannot open map.bin");}
  map.occupancy.resize(map.width * map.height);
  binary.read(reinterpret_cast<char *>(map.occupancy.data()),
    static_cast<std::streamsize>(map.occupancy.size()));
  if (static_cast<std::size_t>(binary.gcount()) != map.occupancy.size()) {
    throw std::runtime_error("map.bin size mismatch");
  }
  return map;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: opponent_offline CACHE_SESSION_DIR OUTPUT.jsonl "
      "[filtered|raw] [--mapless] [--deskew] [--confirmation-hits N] "
      "[--cluster-gap METERS]\n";
    return 2;
  }
  try {
    const std::string dir = argv[1];
    const std::string output_path = argv[2];
    std::string selected_topic = "filtered";
    bool mapless = false;
    bool deskew = false;
    int confirmation_hits = 5;
    double cluster_gap = 0.105;
    for (int i = 3; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "filtered" || option == "raw") {selected_topic = option;}
      else if (option == "--mapless") {mapless = true;}
      else if (option == "--deskew") {deskew = true;}
      else if (option == "--confirmation-hits" && i + 1 < argc) {
        confirmation_hits = std::stoi(argv[++i]);
        if (confirmation_hits < 1 || confirmation_hits > 20) {
          throw std::runtime_error("confirmation hits out of range");
        }
      }
      else if (option == "--cluster-gap" && i + 1 < argc) {
        cluster_gap = std::stod(argv[++i]);
        if (!(cluster_gap > 0.0 && cluster_gap < 0.25)) {
          throw std::runtime_error("cluster gap out of range");
        }
      }
      else {throw std::runtime_error("unknown option: " + option);}
    }
    const auto slash = dir.find_last_of('/');
    const std::string bag = dir.substr(slash == std::string::npos ? 0 : slash + 1);
    core::Map map;
    if (!mapless) {map = read_map(dir);}
    const std::string world_frame = mapless ? "odom_local" : "map";
    std::vector<PoseSample> pose_samples;
    if (deskew) {
      Tsv pose_rows(dir + "/frames.tsv");
      std::string pose_line;
      while (std::getline(pose_rows.file, pose_line)) {
        if (pose_line.empty()) {continue;}
        const auto row = split(pose_line);
        if (pose_rows.get(row, "topic") != selected_topic ||
          pose_rows.get(row, "pose_status") != "ok")
        {
          continue;
        }
        PoseSample sample;
        sample.stamp_ns = integer(pose_rows.get(row, "header_ns"));
        sample.pose.translation = {
          number(pose_rows.get(row, "map_sensor_x")),
          number(pose_rows.get(row, "map_sensor_y")),
          number(pose_rows.get(row, "map_sensor_z"))};
        sample.pose.qx = number(pose_rows.get(row, "map_sensor_qx"));
        sample.pose.qy = number(pose_rows.get(row, "map_sensor_qy"));
        sample.pose.qz = number(pose_rows.get(row, "map_sensor_qz"));
        sample.pose.qw = number(pose_rows.get(row, "map_sensor_qw"));
        pose_samples.push_back(sample);
      }
      std::sort(pose_samples.begin(), pose_samples.end(),
        [](const PoseSample & a, const PoseSample & b) {
          return a.stamp_ns < b.stamp_ns;
        });
      pose_samples.erase(std::unique(pose_samples.begin(), pose_samples.end(),
        [](const PoseSample & a, const PoseSample & b) {
          return a.stamp_ns == b.stamp_ns;
        }), pose_samples.end());
    }
    Tsv frames(dir + "/frames.tsv");
    std::ifstream clouds(dir + "/clouds.zstbin", std::ios::binary);
    if (!clouds) {throw std::runtime_error("cannot open clouds.zstbin");}
    std::ofstream output(output_path);
    if (!output) {throw std::runtime_error("cannot open output " + output_path);}
    core::Config config;
    config.allow_mapless = mapless;
    config.confirmation_hits = confirmation_hits;
    config.cluster_gap = cluster_gap;
    core::Detector detector(config);
    std::size_t scans = 0;
    std::size_t valid_pose = 0;
    std::size_t measurements = 0;
    std::string line;
    while (std::getline(frames.file, line)) {
      if (line.empty()) {continue;}
      const auto row = split(line);
      const std::string topic = frames.get(row, "topic");
      if (topic != selected_topic) {continue;}
      ++scans;
      const auto record_index = integer(frames.get(row, "record_index"));
      const auto stamp_ns = integer(frames.get(row, "header_ns"));
      const auto point_count = integer(frames.get(row, "width")) *
        integer(frames.get(row, "height"));
      core::Result result;
      result.stamp = static_cast<double>(stamp_ns) * 1e-9;
      if (frames.get(row, "pose_status") != "ok") {
        result.status = "pose_" + frames.get(row, "pose_status");
        write_result(output, bag, topic, world_frame, deskew,
          record_index, stamp_ns, point_count, result);
        continue;
      }
      ++valid_pose;
      if (frames.get(row, "is_bigendian") != "0") {
        throw std::runtime_error("big endian PointCloud2 not supported");
      }
      const std::size_t point_step = integer(frames.get(row, "point_step"));
      const std::size_t row_step = integer(frames.get(row, "row_step"));
      const std::size_t width = integer(frames.get(row, "width"));
      const std::size_t height = integer(frames.get(row, "height"));
      const std::size_t raw_size = integer(frames.get(row, "uncompressed_size"));
      const std::size_t compressed_size = integer(frames.get(row, "compressed_size"));
      const auto offset = integer(frames.get(row, "offset"));
      const std::size_t x_offset = integer(frames.get(row, "x_offset"));
      const std::size_t y_offset = integer(frames.get(row, "y_offset"));
      const std::size_t z_offset = integer(frames.get(row, "z_offset"));
      const auto intensity_string = frames.get(row, "intensity_offset");
      const auto tag_string = frames.get(row, "tag_offset");
      const auto line_string = frames.get(row, "line_offset");
      const auto timestamp_string = frames.get(row, "timestamp_offset");
      const std::size_t intensity_offset = intensity_string.empty() ?
        point_step : integer(intensity_string);
      const std::size_t tag_offset = tag_string.empty() ? point_step : integer(tag_string);
      const std::size_t line_offset = line_string.empty() ? point_step : integer(line_string);
      const std::size_t timestamp_offset = timestamp_string.empty() ?
        point_step : integer(timestamp_string);
      if (x_offset + 4 > point_step || y_offset + 4 > point_step ||
        z_offset + 4 > point_step || row_step * height != raw_size)
      {
        throw std::runtime_error("invalid PointCloud2 layout");
      }
      std::vector<char> compressed(compressed_size);
      std::vector<std::uint8_t> decompressed(raw_size);
      clouds.seekg(static_cast<std::streamoff>(offset));
      clouds.read(compressed.data(), static_cast<std::streamsize>(compressed_size));
      if (static_cast<std::size_t>(clouds.gcount()) != compressed_size) {
        throw std::runtime_error("truncated clouds.zstbin");
      }
      const std::size_t decoded = ZSTD_decompress(
        decompressed.data(), raw_size, compressed.data(), compressed_size);
      if (ZSTD_isError(decoded) || decoded != raw_size) {
        throw std::runtime_error("zstd frame decode failed");
      }
      core::Frame frame;
      frame.stamp = result.stamp;
      frame.map = mapless ? nullptr : &map;
      frame.map_from_sensor.translation = {
        number(frames.get(row, "map_sensor_x")),
        number(frames.get(row, "map_sensor_y")),
        number(frames.get(row, "map_sensor_z"))};
      frame.map_from_sensor.qx = number(frames.get(row, "map_sensor_qx"));
      frame.map_from_sensor.qy = number(frames.get(row, "map_sensor_qy"));
      frame.map_from_sensor.qz = number(frames.get(row, "map_sensor_qz"));
      frame.map_from_sensor.qw = number(frames.get(row, "map_sensor_qw"));
      frame.pose_var_x = number(frames.get(row, "pose_var_x"));
      frame.pose_var_y = number(frames.get(row, "pose_var_y"));
      frame.pose_var_yaw = number(frames.get(row, "pose_var_yaw"));
      frame.points.reserve(width * height);
      std::unordered_map<std::int64_t, std::optional<core::Transform>> bucket_poses;
      std::string deskew_error;
      if (deskew && timestamp_offset + 8 > point_step) {
        deskew_error = "missing_point_timestamps";
      }
      for (std::size_t y = 0; y < height; ++y) {
        for (std::size_t x = 0; x < width; ++x) {
          if (!deskew_error.empty()) {break;}
          const std::uint8_t * ptr = decompressed.data() + y * row_step + x * point_step;
          core::Point point;
          point.x = read_float(ptr + x_offset);
          point.y = read_float(ptr + y_offset);
          point.z = read_float(ptr + z_offset);
          if (intensity_offset + 4 <= point_step) {
            point.intensity = read_float(ptr + intensity_offset);
          }
          if (tag_offset < point_step) {point.tag = ptr[tag_offset];}
          if (line_offset < point_step) {point.line = ptr[line_offset];}
          if (deskew) {
            const double point_ns = read_double(ptr + timestamp_offset);
            if (!std::isfinite(point_ns) || point_ns < 1e15 || point_ns > 1e20) {
              deskew_error = "invalid_point_timestamps";
              break;
            }
            const auto delta_ns = static_cast<std::int64_t>(std::llround(point_ns)) -
              static_cast<std::int64_t>(stamp_ns);
            if (delta_ns < -10000000LL || delta_ns > 160000000LL) {
              deskew_error = "point_time_outside_packet";
              break;
            }
            constexpr std::int64_t bucket_ns = 15000000LL;
            const auto bucket = static_cast<std::int64_t>(
              std::floor(static_cast<double>(delta_ns) / bucket_ns));
            auto found = bucket_poses.find(bucket);
            if (found == bucket_poses.end()) {
              const auto representative_ns = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(stamp_ns) + bucket * bucket_ns +
                bucket_ns / 2);
              found = bucket_poses.emplace(bucket,
                pose_at(pose_samples, representative_ns)).first;
            }
            if (!found->second) {
              deskew_error = "deskew_pose_unavailable";
              break;
            }
            point = point_at_header(point, *found->second, frame.map_from_sensor);
          }
          frame.points.push_back(point);
        }
        if (!deskew_error.empty()) {break;}
      }
      if (!deskew_error.empty()) {
        result.status = deskew_error;
        write_result(output, bag, topic, world_frame, deskew,
          record_index, stamp_ns, point_count, result);
        continue;
      }
      result = detector.process(frame);
      measurements += result.has_measurement ? 1 : 0;
      write_result(output, bag, topic, world_frame, deskew,
        record_index, stamp_ns, frame.points.size(), result);
    }
    std::cerr << bag << ": scans=" << scans << ", valid_pose=" << valid_pose <<
      ", measured=" << measurements << "\n";
  } catch (const std::exception & error) {
    std::cerr << "opponent_offline: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
