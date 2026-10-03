#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace jr_perception::opponent_core {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// T_map_sensor at the acquisition time of this cloud. The caller is responsible
// for selecting/interpolating stamped pose and TF, and for motion compensation
// within the packet if it is needed.
struct Transform {
  Vec3 translation;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

struct Point {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

struct Map {
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::vector<std::int8_t> occupancy;  // row-major; -1 means unknown
};

struct Frame {
  double stamp = 0.0;                 // seconds, same clock as subsequent frames
  std::vector<Point> points;          // points in the sensor frame
  Transform map_from_sensor;
  // Marginal position and yaw variance of the stamped localization estimate.
  double pose_var_x = 0.0;
  double pose_var_y = 0.0;
  double pose_var_yaw = 0.0;
  const Map * map = nullptr;          // must remain alive through process()
};

enum class Decision { accepted, ambiguous, rejected };

struct Candidate {
  std::uint64_t id = 0;
  Decision decision = Decision::rejected;
  std::string reason;
  Vec3 center_map;
  std::array<double, 4> position_covariance{{0.0, 0.0, 0.0, 0.0}};  // xx,xy,yx,yy
  double robot_probability = 0.0;
  double box_probability = 0.0;
  double wall_probability = 0.0;
  double artifact_probability = 0.0;
  double width = 0.0;
  double depth = 0.0;
  double height = 0.0;
  std::size_t point_count = 0;
  std::size_t support_count = 0;
  std::size_t contradictory_count = 0;
  std::size_t map_occupied_count = 0;
  std::size_t angular_bins = 0;
  std::size_t height_bins = 0;
  // Fractions use every point of the connected object, not the marker sample.
  // A Kobuki presents a dense middle body; a box tends to retain its upper face.
  double middle_layer_fraction = 0.0;  // 0.115 <= z < 0.23 m
  double upper_layer_fraction = 0.0;   // 0.23 <= z < 0.50 m
  double middle_to_upper_ratio = 0.0;  // middle / (middle + upper)
  bool weak_body_observation = false; // geometry valid, uncertain layer evidence
  bool continuation_body_observation = false; // partial body, confirmed track only
  bool tall_broad_body = false; // independent size/height explanation
  bool complete_body_viable = false; // complete object passes hard shape/ray/map gates
  bool temporal_change = false; // proposal from a changed real return at fixed sensor pose
  std::vector<Vec3> object_map;       // capped sample of the complete local object
  std::vector<Vec3> support_map;      // capped sample for markers
  std::vector<Vec3> contradictory_map;  // capped sample of incompatible returns
  std::uint64_t track_id = 0;
};

struct Track {
  std::uint64_t id = 0;
  Vec3 center_map;
  Vec3 velocity_map;
  std::array<double, 4> position_covariance{{0.0, 0.0, 0.0, 0.0}};
  double velocity_variance = 1.0;
  double probability = 0.0;
  double age_since_measurement = 0.0;
  int measured_hits = 0;
  bool confirmed = false;
  bool velocity_valid = false;
  bool measured_this_frame = false;
};

struct Timing {
  double transform_ms = 0.0;
  double segmentation_ms = 0.0;
  double scoring_ms = 0.0;
  double tracking_ms = 0.0;
  double total_ms = 0.0;
};

struct Result {
  double stamp = 0.0;
  std::string status;                  // ready / invalid_map / invalid_transform / ...
  std::vector<Candidate> candidates;
  std::vector<Track> tracks;
  bool has_measurement = false;       // publish odom/visible only when true
  Track measurement;                  // current cloud support, selected target
  bool has_prediction = false;        // RViz/debug only, never odom/visible
  Track prediction;
  Timing timing;
  double map_alignment_share = -1.0;  // -1 when no map is available
};

struct Config {
  // Offline local-odom ablation only. The real ROS node requires /map.
  bool allow_mapless = false;
  double min_range = 0.32;
  double max_range = 6.0;
  double min_height = 0.045;
  double max_height = 0.60;
  double voxel_size = 0.045;
  double cluster_gap = 0.105;
  double robot_radius = 0.178;
  double min_map_alignment_share = 0.35;
  std::size_t min_map_alignment_points = 1000;
  double max_track_gap_s = 0.70;
  double max_track_age_s = 1.50;
  int confirmation_hits = 5;
  std::size_t max_marker_points = 64;
};

class Detector {
public:
  explicit Detector(Config config = {});
  ~Detector();
  Detector(const Detector &) = delete;
  Detector & operator=(const Detector &) = delete;
  Detector(Detector &&) noexcept;
  Detector & operator=(Detector &&) noexcept;
  Result process(const Frame & frame);
  void reset();

private:
  Config config_;
  struct InternalTrack;
  std::vector<InternalTrack> tracks_;
  std::uint64_t next_track_id_ = 1;
  struct MotionTrack;
  std::vector<MotionTrack> motion_tracks_;
  std::uint64_t next_motion_id_ = 1;
  struct TemporalFrame {
    double stamp = 0.0;
    Vec3 sensor_pose;  // x, y, yaw
    std::vector<Vec3> points;
  };
  std::deque<TemporalFrame> temporal_frames_;
  std::size_t temporal_point_count_ = 0;
  double last_stamp_ = -1.0;
};

}  // namespace jr_perception::opponent_core
