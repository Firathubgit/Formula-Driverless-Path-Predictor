#pragma once
#include "fd/cone_path.hpp"
#include "fd/local_planner.hpp"
#include "fd/path_following.hpp"
#include "fd/perception.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace fd {

// Driving on cones (TrackWayFastPlan Phase 7.3 and 7.4, decision 0031): the car follows the path it believes from its own
// simulated detections, placed on the map by the pose it believes, and nothing else. The driver below is the whole of
// what decides in that mode, and everything it is ever given is in its interface: the pose the car was placed at, which
// is where its map starts; its pose and motion readings, measured by its instruments or, without them, the true state as
// an explicit ideal-state assumption; and its perception frames. It never receives the track, the course's cones, or the
// truth of any frame; the evaluation reads those, apart from it.

// Where the driver's readings come from.
enum class BeliefSource { measured, ideal };
// "measured" or "ideal".
const char* belief_source_name(BeliefSource source);

// A pose reading in the map frame, and the latest motion readings, each as the car's instruments deliver them.
struct PoseReading { double sampled_at_s{}, x_m{}, y_m{}, yaw_rad{}; };
struct MotionReading { double speed_mps{}, yaw_rate_radps{}, steering_rad{}; };

// How the driver remembers the cones it has placed on the map. FaSTTUBe's planner is meant to see the cones around the
// car, not one frame of a sensor facing forward: in a hairpin the cones it needs stand beside and behind it. Each
// detection placed within merge_radius_m of a remembered cone refines that cone, its position the mean of its placements
// and its colour the one most often reported; any other is a new cone; a cone farther than keep_within_m from the car is
// forgotten. With enabled false the driver plans from each frame alone, as schema 16 recordings did. This is a memory
// of placed detections, not a map estimated together with the pose: nothing here corrects the pose.
struct ConeMemory {
    bool enabled{true};
    double merge_radius_m{1.0};
    double keep_within_m{30.0};
};
void validate_cone_memory(const ConeMemory& memory);

// A cone the driver remembers: where its placements put it on average, how often each colour was reported, and how
// many times it was placed.
struct RememberedCone {
    Vec2 position;
    std::array<int, 5> colour_votes{};  // by ConeType
    int placements{};
};

// A path the driver believed, made from one perception frame.
struct BelievedPath {
    std::uint64_t index{};          // in the order the driver made them, from zero
    std::uint64_t frame{};          // the perception frame it came from, counted from the first the driver was given
    State pose;                     // the pose believed at the frame's sampling, which placed its detections on the map
    ConePath cones;                 // FaSTTUBe's steps on the frame's detections, in the map frame
    OpenPath path;                  // the believed path, the cone path's samples at stations of their own chords
};

// What the driver believed at a decision.
struct Belief {
    State state;                          // position, heading, speed and steering, at the decision's time
    double pose_sampled_at_s{};           // when the pose it was carried forward from was sampled
    std::optional<std::uint64_t> path;    // the believed path it followed; none while it has made none
    std::vector<PlanPoint> plan;          // the speeds planned along that path at this decision; empty without one
};

class ConeDriver {
public:
    explicit ConeDriver(ConePathSettings settings = {}, ConeMemory memory = {});
    // Starts the driver again, placed at rest at this pose: known as a team knows where it put its car, and nothing else.
    void reset(const State& start);
    void read_pose(const PoseReading& reading);
    void read_motion(const MotionReading& reading);
    // Plans from a frame of detections, placed on the map by the pose believed when it was sampled, with the cones it
    // remembers. Returns whether it made a new believed path: not when the frame came before any pose, nor when FaSTTUBe
    // made its path from its previous one, which the driver already follows.
    bool perceive(const PerceptionFrame& frame);
    // The decision at this time: follow the believed path by the policy's rules, predicted from the believed state; or,
    // with no believed path yet, brake to rest where the car is.
    LocalDecision decide(double time_s, const Config& config, const VehicleModel& model, std::uint64_t ticks_to_next_control,
                         const SteeringTable* steering, const PerformanceEnvelope* envelope);
    const Belief& belief() const noexcept { return belief_; }
    // The believed paths made most recently, oldest first, the last being the one followed: at most kept_paths of them,
    // which is more than a step's frames can make, so a caller looking after every step sees each one.
    static constexpr std::size_t kept_paths = 16;
    const std::vector<BelievedPath>& recent_paths() const noexcept { return paths_; }
    const BelievedPath* path() const noexcept { return paths_.empty() ? nullptr : &paths_.back(); }
    std::uint64_t paths_made() const noexcept { return made_; }
    const ConePathSettings& settings() const noexcept { return planner_.settings(); }
    const ConeMemory& memory_settings() const noexcept { return memory_settings_; }
    const std::vector<RememberedCone>& remembered() const noexcept { return memory_; }
    // Where the driver would believe the car to be at a time, from what it has read so far: between two pose readings
    // interpolated, beyond the latest carried forward at the latest speed and yaw rate. Speed and steering as last read.
    State believed_at(double time_s) const;
private:
    ConePathPlanner planner_;
    ConeMemory memory_settings_;
    std::vector<RememberedCone> memory_;
    std::vector<PoseReading> poses_;
    MotionReading motion_;
    std::vector<BelievedPath> paths_;
    Belief belief_;
    std::uint64_t frames_{}, made_{};
};

} // namespace fd
