#pragma once
#include "fd/cones.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace fd {

// Judging a run as a Formula Student official would (TrackWayFastPlan Phase 7.5, decision 0032), after PacSim's
// competition logic (MIT), read for structure: timekeeping gates that start the clock and time laps and sectors, a cone
// knocked down or out for 2 s, off course for 10 s and disqualification after 8 s of it, and an unsafe stop after the
// finish. The judge reads the car's true pose and the course's cones, ground truth both, and is the evaluation's alone:
// nothing that drives the car reads what it decides.

enum class Discipline { autocross, trackdrive };
// "autocross" or "trackdrive".
const char* discipline_name(Discipline discipline);

struct CompetitionRules {
    Discipline discipline{Discipline::trackdrive};
    int trackdrive_laps{10};                         // an autocross is one lap
    double cone_hit_s{2.0};                          // each cone down or out
    double off_course_s{10.0};                       // each time all four wheels leave the course
    double unsafe_stop_s{10.0};                      // trackdrive; an unsafe stop in autocross is a DNF
    double off_course_limit_s{8.0};                  // off course longer than this is a DNF
    double start_timeout_s{60.0};                    // to cross the start line
    double autocross_timeout_s{300.0};
    double first_lap_timeout_s{300.0};               // trackdrive
    double total_timeout_s{1000.0};                  // trackdrive
    double stop_zone_m{30.0};                        // after the finish the car stops within this of the line
    double stop_within_s{33.0};                      // and within this long
    double stopped_below_mps{0.1};
};
void validate_competition_rules(const CompetitionRules& rules);

// The car as the judge sees it, from the rear axle along its heading: its four wheels' contact points, which decide
// off course, and its body, which knocks cones. PacSim's car is a 1.5 m wheelbase; this one keeps the simulation's
// wheelbase and a Formula Student car's track and overhangs.
struct Footprint {
    double wheelbase_m{2.6};
    double half_track_m{0.6};
    double body_rear_m{0.6};        // behind the rear axle
    double body_front_m{3.2};       // ahead of the rear axle
    double body_half_width_m{0.7};
    double cone_width_m{0.228};     // a cone's base, square, as PacSim takes it
};
void validate_footprint(const Footprint& footprint);

// A timekeeping gate: a line from its left end to its right, seen in the direction of travel. The car crosses it when
// its rear axle passes from behind the line to ahead of it between the ends. The first gate starts and finishes laps;
// any others split them into sectors.
struct Gate { Vec2 left, right; };

// The course a run is judged on: the track, whose corridor is the course's boundary; its cones; and its gates.
struct Course {
    Track track;
    std::vector<Cone> cones;
    std::vector<Gate> gates;
};
// The gates of a closed track: the start and finish line across its first sample, and sector lines across it at a third
// and two thirds of its length, each reaching a metre beyond the corridor either side.
std::vector<Gate> make_gates(const Track& track);

// Personal practice timing, independent of competition penalties/DNF. Reads true
// poses only; twelve ordered corridor gates prevent start-line rocking or reverse
// crossings from producing a lap. The first accelerator input starts the clock.
class PracticeLaps {
public:
    explicit PracticeLaps(const Track& track);
    void reset(bool keep_history = true);
    void observe(const State& before, const State& after, double throttle);
    double current_seconds(double now) const;
    bool started() const { return started_; }
    double previous_seconds() const { return previous_; }
    double best_seconds() const { return best_; }
    int completed() const { return completed_; }
private:
    std::vector<Gate> gates_;
    std::size_t next_{1};
    bool started_{};
    double start_{}, previous_{}, best_{};
    int completed_{};
};
// Rejects a track validate_track rejects, a gate of no length or not finite, and no gate at all.
void validate_course(const Course& course);

// A Formula Student layout as PacSim's track files give one: the left (blue) and right (yellow) cones in order, any
// orange cones, the timekeeping gates as pairs of cones, the start pose, and whether the lanes close into a loop.
struct CourseFile {
    std::string name;
    std::vector<Cone> left, right, orange;
    std::vector<Gate> gates;
    State start;
    bool closed{};
};
// Reads PacSim's YAML track format (track: left, right, time_keeping, unknown, start, lanesFirstWithLastConnected),
// keeping to what that format uses; rejects anything it cannot read, naming the line.
CourseFile load_pacsim_course(const std::filesystem::path& file);
// The closed course such a layout makes: its centreline the midpoints between each left cone and the right cones'
// polyline, conditioned into a smooth closed track that starts beside the start pose, with the distance from each sample
// to either cone line as its corridor edges; its cones as laid; its gates, left end first. Rejects an open layout and
// one that conditioning refuses.
Course course_from_layout(const CourseFile& layout);

enum class TimingKind { start, sector, lap, finish, cone_hit, off_course, back_on_course, stopped, unsafe_stop, dnf };
// "start", "sector", "lap", "finish", "cone hit", "off course", "back on course", "stopped", "unsafe stop" or "dnf".
const char* timing_kind_name(TimingKind kind);

// One thing the judge saw: when, in which lap (counting from zero before the first is complete), a lap's or a sector's
// time or a penalty's seconds, the cone knocked, where the car's rear axle was, and why for a DNF.
struct TimingEvent {
    TimingKind kind{TimingKind::start};
    double time_s{};
    int lap{};
    double seconds{};
    std::optional<std::size_t> cone;
    Vec2 position;
    std::string reason;
};

class Judge {
public:
    Judge(Course course, CompetitionRules rules = {}, Footprint footprint = {});
    // Every fixed tick, the car's true pose. Nothing changes after a DNF or a safe stop.
    void observe(const State& pose, double time_s);
    void reset();
    const std::vector<TimingEvent>& events() const noexcept { return events_; }
    const Course& course() const noexcept { return course_; }
    const CompetitionRules& rules() const noexcept { return rules_; }
    const Footprint& footprint() const noexcept { return footprint_; }
    bool started() const noexcept { return started_; }
    int laps() const noexcept { return static_cast<int>(lap_times_.size()); }
    int laps_required() const noexcept { return rules_.discipline == Discipline::autocross ? 1 : rules_.trackdrive_laps; }
    const std::vector<double>& lap_times() const noexcept { return lap_times_; }
    // The sectors of the lap under way, and of each lap completed.
    const std::vector<std::vector<double>>& sector_times() const noexcept { return sector_times_; }
    const std::vector<double>& current_sectors() const noexcept { return current_sectors_; }
    double penalty_s() const noexcept { return penalty_s_; }
    int cones_hit() const noexcept { return cones_hit_; }
    int off_course_count() const noexcept { return off_course_count_; }
    const std::vector<bool>& hit() const noexcept { return hit_; }
    bool off_course() const noexcept { return off_course_; }
    bool finished() const noexcept { return finished_; }
    bool stopped() const noexcept { return stopped_; }
    bool dnf() const noexcept { return dnf_; }
    const std::string& dnf_reason() const noexcept { return dnf_reason_; }
    // Time since the start line, the laps' sum once finished; zero before the start.
    double elapsed_s(double now_s) const noexcept;
private:
    void add(TimingKind kind, double time_s, double seconds, const State& pose, std::optional<std::size_t> cone = {},
             std::string reason = {});
    void disqualify(double time_s, const State& pose, std::string reason);
    Course course_;
    CompetitionRules rules_;
    Footprint footprint_;
    std::vector<TimingEvent> events_;
    std::vector<double> previous_side_;   // each gate's signed distance at the last tick, NaN outside its ends
    std::vector<bool> hit_;
    std::vector<double> lap_times_, current_sectors_;
    std::vector<std::vector<double>> sector_times_;
    double start_time_s_{}, lap_start_s_{}, last_trigger_s_{}, off_course_since_s_{}, finished_at_s_{};
    double penalty_s_{};
    int cones_hit_{}, off_course_count_{};
    bool started_{}, finished_{}, stopped_{}, dnf_{}, off_course_{}, observed_{};
    std::string dnf_reason_;
};

} // namespace fd
