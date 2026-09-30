// Formula Student judging after PacSim's competitionLogic.cpp (MIT), read for structure and for the rules it encodes;
// written anew here, with the car's footprint, gate crossing and cone contact computed exactly. See decision 0032.
#include "fd/competition.hpp"
#include "fd/track_conditioning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace fd {
namespace {

double cross(Vec2 a, Vec2 b) { return a.x*b.y-a.y*b.x; }
double dot(Vec2 a, Vec2 b) { return a.x*b.x+a.y*b.y; }
Vec2 sub(Vec2 a, Vec2 b) { return {a.x-b.x, a.y-b.y}; }
double norm(Vec2 a) { return std::hypot(a.x, a.y); }
bool finite(Vec2 a) { return std::isfinite(a.x) && std::isfinite(a.y); }

double segment_distance(Vec2 a, Vec2 b, Vec2 p) {
    const Vec2 d = sub(b, a);
    const double l2 = dot(d, d);
    const double f = l2 > 0 ? std::clamp(dot(sub(p, a), d)/l2, 0.0, 1.0) : 0.0;
    return norm(sub(p, {a.x+f*d.x, a.y+f*d.y}));
}

// The nearest point of a polyline, closed when asked, and its distance.
std::pair<Vec2, double> nearest_on(const std::vector<Vec2>& line, bool closed, Vec2 p) {
    Vec2 best{};
    double distance = std::numeric_limits<double>::infinity();
    const std::size_t n = line.size();
    for (std::size_t i = 0; i+1 < n+(closed ? 1 : 0); ++i) {
        const Vec2 a = line[i], b = line[(i+1)%n], d = sub(b, a);
        const double l2 = dot(d, d);
        const double f = l2 > 0 ? std::clamp(dot(sub(p, a), d)/l2, 0.0, 1.0) : 0.0;
        const Vec2 q{a.x+f*d.x, a.y+f*d.y};
        const double here = norm(sub(p, q));
        if (here < distance) { distance = here; best = q; }
    }
    return {best, distance};
}

// Whether two convex quadrilaterals overlap, by the separating axis theorem over both sets of edge normals.
bool overlap(const std::array<Vec2, 4>& a, const std::array<Vec2, 4>& b) {
    for (const auto* shape : {&a, &b})
        for (std::size_t i = 0; i < 4; ++i) {
            const Vec2 edge = sub((*shape)[(i+1)%4], (*shape)[i]);
            const Vec2 axis{-edge.y, edge.x};
            double a_min = std::numeric_limits<double>::infinity(), a_max = -a_min, b_min = a_min, b_max = -a_min;
            for (const auto& p : a) { a_min = std::min(a_min, dot(p, axis)); a_max = std::max(a_max, dot(p, axis)); }
            for (const auto& p : b) { b_min = std::min(b_min, dot(p, axis)); b_max = std::max(b_max, dot(p, axis)); }
            if (a_max < b_min || b_max < a_min) return false;
        }
    return true;
}

// The corridor's edge points across a projection: left and right, reaching margin beyond it.
Gate gate_at(const Track& track, double s, double margin) {
    const auto here = sample(track, s);
    const auto ahead = sample(track, s+0.05);
    const double tx = ahead.x_m-here.x_m, ty = ahead.y_m-here.y_m, n = std::hypot(tx, ty);
    const Vec2 left{-ty/n, tx/n};
    const auto corridor = corridor_at(track, project(track, {here.x_m, here.y_m}));
    return {{here.x_m+left.x*(corridor.left_m+margin), here.y_m+left.y*(corridor.left_m+margin)},
            {here.x_m-left.x*(corridor.right_m+margin), here.y_m-left.y*(corridor.right_m+margin)}};
}

std::string trim(std::string s) {
    const auto first = s.find_first_not_of(" \t\r");
    if (first == std::string::npos) return {};
    return s.substr(first, s.find_last_not_of(" \t\r")-first+1);
}

std::array<double, 3> triple(const std::string& text, const std::string& where) {
    const auto open = text.find('['), close = text.find(']');
    if (open == std::string::npos || close == std::string::npos || close < open) throw std::invalid_argument(where+": expected [x, y, z]");
    std::stringstream in(text.substr(open+1, close-open-1));
    std::array<double, 3> v{};
    for (std::size_t k = 0; k < 3; ++k) {
        std::string item;
        if (!std::getline(in, item, ',')) throw std::invalid_argument(where+": expected three numbers");
        try {
            std::size_t used = 0;
            v[k] = std::stod(trim(item), &used);
            if (used != trim(item).size() || !std::isfinite(v[k])) throw std::invalid_argument("");
        } catch (const std::exception&) { throw std::invalid_argument(where+": '"+trim(item)+"' is not a finite number"); }
    }
    return v;
}

} // namespace

const char* discipline_name(Discipline discipline) { return discipline == Discipline::autocross ? "autocross" : "trackdrive"; }

const char* timing_kind_name(TimingKind kind) {
    switch (kind) {
    case TimingKind::start: return "start";
    case TimingKind::sector: return "sector";
    case TimingKind::lap: return "lap";
    case TimingKind::finish: return "finish";
    case TimingKind::cone_hit: return "cone hit";
    case TimingKind::off_course: return "off course";
    case TimingKind::back_on_course: return "back on course";
    case TimingKind::stopped: return "stopped";
    case TimingKind::unsafe_stop: return "unsafe stop";
    case TimingKind::dnf: return "dnf";
    }
    return "start";
}

void validate_competition_rules(const CompetitionRules& r) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    const auto nonnegative = [](double v) { return std::isfinite(v) && v >= 0; };
    if ((r.discipline != Discipline::autocross && r.discipline != Discipline::trackdrive) || r.trackdrive_laps < 1 ||
        r.trackdrive_laps > 100 || !nonnegative(r.cone_hit_s) || !nonnegative(r.off_course_s) || !nonnegative(r.unsafe_stop_s) ||
        !positive(r.off_course_limit_s) || !positive(r.start_timeout_s) || !positive(r.autocross_timeout_s) ||
        !positive(r.first_lap_timeout_s) || !positive(r.total_timeout_s) || !positive(r.stop_zone_m) || !positive(r.stop_within_s) ||
        !positive(r.stopped_below_mps))
        throw std::invalid_argument("Competition rules are out of range");
}

void validate_footprint(const Footprint& f) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    if (!positive(f.wheelbase_m) || !positive(f.half_track_m) || !positive(f.body_rear_m) || !positive(f.body_front_m) ||
        !positive(f.body_half_width_m) || !positive(f.cone_width_m) || f.body_front_m < f.wheelbase_m)
        throw std::invalid_argument("A car's footprint needs positive dimensions and a body reaching its front axle");
}

std::vector<Gate> make_gates(const Track& track) {
    validate_track(track);
    return {gate_at(track, 0, 1.0), gate_at(track, track.length_m/3, 1.0), gate_at(track, 2*track.length_m/3, 1.0)};
}

PracticeLaps::PracticeLaps(const Track& track) {
    validate_track(track);
    for (int i=0;i<12;++i) gates_.push_back(gate_at(track,track.length_m*i/12,0));
}
void PracticeLaps::reset(bool keep_history) {
    started_=false; start_=0; next_=1;
    if (!keep_history) { previous_=best_=0; completed_=0; }
}
double PracticeLaps::current_seconds(double now) const {
    return started_ ? std::max(0.0,now-start_) : 0;
}
void PracticeLaps::observe(const State& before, const State& after, double throttle) {
    if (!std::isfinite(throttle) || throttle<0 || throttle>1 ||
        !std::isfinite(before.time_s) || !std::isfinite(after.time_s) || after.time_s<=before.time_s ||
        !std::isfinite(before.x_m) || !std::isfinite(before.y_m) || !std::isfinite(after.x_m) || !std::isfinite(after.y_m))
        throw std::invalid_argument("Practice timing needs finite forward-time poses and a valid throttle");
    if (!started_) {
        if (throttle<=0.05) return;
        started_=true; start_=before.time_s;
    }
    const auto& gate=gates_[next_];
    const auto across=sub(gate.right,gate.left);
    const double width=norm(across);
    const Vec2 forward{-across.y/width,across.x/width};
    const Vec2 a{before.x_m,before.y_m}, b{after.x_m,after.y_m};
    const double from=dot(sub(a,gate.left),forward), to=dot(sub(b,gate.left),forward);
    if (!(from<=0 && to>0)) return;
    const double fraction=-from/(to-from);
    const Vec2 crossing{a.x+(b.x-a.x)*fraction,a.y+(b.y-a.y)*fraction};
    const double along=dot(sub(crossing,gate.left),across)/width;
    if (along<0 || along>width) return;
    if (next_==0) {
        const double time=before.time_s+(after.time_s-before.time_s)*fraction;
        previous_=time-start_;
        if (best_==0 || previous_<best_) best_=previous_;
        ++completed_; start_=time;
    }
    next_=(next_+1)%gates_.size();
}

void validate_course(const Course& course) {
    validate_track(course.track);
    if (course.gates.empty()) throw std::invalid_argument("A course needs a start line");
    for (const auto& g : course.gates)
        if (!finite(g.left) || !finite(g.right) || norm(sub(g.right, g.left)) < 1e-6)
            throw std::invalid_argument("A gate needs two distinct finite ends");
    for (const auto& c : course.cones)
        if (!finite(c.position)) throw std::invalid_argument("A cone needs a finite position");
}

CourseFile load_pacsim_course(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw std::invalid_argument("Cannot read course file "+file.string());
    CourseFile out;
    out.name = file.stem().string();
    std::string line, section, subsection;
    int number = 0;
    std::optional<std::array<double, 3>> pending;  // a list item's position, waiting for its class
    std::vector<Vec2> gate_ends;
    const auto take = [&](const std::string& cls, const std::string& where) {
        if (!pending) return;
        const Vec2 p{(*pending)[0], (*pending)[1]};
        pending.reset();
        if (cls == "invisible") return;
        if (section == "time_keeping") { gate_ends.push_back(p); return; }
        ConeColour colour = section == "right" ? ConeColour::yellow : ConeColour::blue;
        if (cls == "blue") colour = ConeColour::blue;
        else if (cls == "yellow") colour = ConeColour::yellow;
        else if (cls == "small" || cls == "small-orange") colour = ConeColour::orange;
        else if (cls == "big" || cls == "big-orange") colour = ConeColour::big_orange;
        else if (cls != "unknown") throw std::invalid_argument(where+": unknown cone class '"+cls+"'");
        if (section == "left") out.left.push_back({p, colour});
        else if (section == "right") out.right.push_back({p, colour});
        else if (colour == ConeColour::orange || colour == ConeColour::big_orange) out.orange.push_back({p, colour});
    };
    bool in_track = false;
    while (std::getline(in, line)) {
        ++number;
        const std::string where = file.filename().string()+" line "+std::to_string(number);
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        if (trim(line).empty()) continue;
        const std::size_t indent = line.find_first_not_of(' ');
        const std::string text = trim(line);
        if (indent == 0) { in_track = text == "track:"; if (!in_track) throw std::invalid_argument(where+": expected 'track:'"); continue; }
        if (!in_track) throw std::invalid_argument(where+": expected 'track:' first");
        if (indent == 2 && text[0] != '-') {
            take("unknown", where);
            const auto colon = text.find(':');
            if (colon == std::string::npos) throw std::invalid_argument(where+": expected a key");
            section = text.substr(0, colon);
            subsection.clear();
            const std::string value = trim(text.substr(colon+1));
            if (section == "lanesFirstWithLastConnected") {
                if (value != "true" && value != "false") throw std::invalid_argument(where+": expected true or false");
                out.closed = value == "true";
            }
            continue;
        }
        if (section == "left" || section == "right" || section == "time_keeping" || section == "unknown") {
            if (text.rfind("- position:", 0) == 0) { take("unknown", where); pending = triple(text, where); continue; }
            if (text.rfind("class:", 0) == 0) { take(trim(text.substr(6)), where); continue; }
            if (text.rfind("- ", 0) == 0 || text.rfind("position:", 0) == 0) throw std::invalid_argument(where+": expected '- position: [x, y, z]'");
            continue;  // other per-cone attributes are not used
        }
        if (section == "start") {
            if (text.rfind("position:", 0) == 0) { const auto p = triple(text, where); out.start.x_m = p[0]; out.start.y_m = p[1]; }
            else if (text.rfind("orientation:", 0) == 0) out.start.yaw_rad = triple(text, where)[2];
            continue;
        }
        if (section == "earthToTrack") {
            // A layout placed elsewhere on the earth would need its transform; PacSim's own layouts all have none.
            if (text.rfind("position:", 0) == 0 || text.rfind("orientation:", 0) == 0) {
                const auto v = triple(text, where);
                if (v[0] != 0 || v[1] != 0 || v[2] != 0) throw std::invalid_argument(where+": a transform to the earth is not supported");
            }
            continue;
        }
    }
    take("unknown", "end of "+file.filename().string());
    if (gate_ends.size()%2) throw std::invalid_argument(file.filename().string()+": timekeeping cones come in pairs");
    for (std::size_t i = 0; i+1 < gate_ends.size(); i += 2) out.gates.push_back({gate_ends[i], gate_ends[i+1]});
    if (out.left.size() < 3 || out.right.size() < 3) throw std::invalid_argument(file.filename().string()+": a course needs both lanes of cones");
    return out;
}

Course course_from_layout(const CourseFile& layout) {
    if (!layout.closed) throw std::invalid_argument(layout.name+": only a closed course, its lanes' last cones joining their first, is driven");
    std::vector<Vec2> left, right;
    for (const auto& c : layout.left) left.push_back(c.position);
    for (const auto& c : layout.right) right.push_back(c.position);
    // The centreline: the midpoint between each left cone and the right lane's nearest point, in the left lane's order.
    std::vector<Vec2> centre;
    std::vector<double> widths;
    for (const auto& p : left) {
        const auto [q, d] = nearest_on(right, true, p);
        centre.push_back({(p.x+q.x)/2, (p.y+q.y)/2});
        widths.push_back(d);
    }
    // Starting beside the start pose.
    const Vec2 start{layout.start.x_m, layout.start.y_m};
    const auto first = std::min_element(centre.begin(), centre.end(), [&](Vec2 a, Vec2 b) { return norm(sub(a, start)) < norm(sub(b, start)); });
    std::rotate(centre.begin(), first, centre.end());
    std::nth_element(widths.begin(), widths.begin()+static_cast<std::ptrdiff_t>(widths.size()/2), widths.end());
    const double width = widths[widths.size()/2];
    ConditioningOptions options;
    options.input_spacing_m = 1.0;
    options.smoothing_rms_m = 0.1;
    auto conditioned = condition_track(centre, width, layout.name, options);
    auto& track = conditioned.track;
    // Its corridor is the cones' own: each sample's distance to either lane.
    track.left_edge_m.clear();
    track.right_edge_m.clear();
    for (const auto& p : track.points) {
        track.left_edge_m.push_back(nearest_on(left, true, {p.x_m, p.y_m}).second);
        track.right_edge_m.push_back(nearest_on(right, true, {p.x_m, p.y_m}).second);
    }
    validate_track(track);
    Course course;
    course.track = track;
    course.cones.insert(course.cones.end(), layout.left.begin(), layout.left.end());
    course.cones.insert(course.cones.end(), layout.right.begin(), layout.right.end());
    course.cones.insert(course.cones.end(), layout.orange.begin(), layout.orange.end());
    // Each gate's left end first, by the direction of travel where it stands.
    for (auto gate : layout.gates) {
        const Vec2 middle{(gate.left.x+gate.right.x)/2, (gate.left.y+gate.right.y)/2};
        const auto at = project(track, middle);
        const auto here = sample(track, at.s_m), ahead = sample(track, at.s_m+0.05);
        const Vec2 tangent{ahead.x_m-here.x_m, ahead.y_m-here.y_m};
        if (cross(tangent, sub(gate.left, middle)) < 0) std::swap(gate.left, gate.right);
        course.gates.push_back(gate);
    }
    if (course.gates.empty()) course.gates = make_gates(track);
    validate_course(course);
    return course;
}

Judge::Judge(Course course, CompetitionRules rules, Footprint footprint)
    : course_(std::move(course)), rules_(rules), footprint_(footprint) {
    validate_course(course_);
    validate_competition_rules(rules_);
    validate_footprint(footprint_);
    reset();
}

void Judge::reset() {
    events_.clear();
    previous_side_.assign(course_.gates.size(), std::numeric_limits<double>::quiet_NaN());
    hit_.assign(course_.cones.size(), false);
    lap_times_.clear();
    current_sectors_.clear();
    sector_times_.clear();
    start_time_s_ = lap_start_s_ = last_trigger_s_ = off_course_since_s_ = finished_at_s_ = penalty_s_ = 0;
    cones_hit_ = off_course_count_ = 0;
    started_ = finished_ = stopped_ = dnf_ = off_course_ = observed_ = false;
    dnf_reason_.clear();
}

double Judge::elapsed_s(double now_s) const noexcept {
    if (!started_) return 0;
    return (finished_ ? finished_at_s_ : now_s)-start_time_s_;
}

void Judge::add(TimingKind kind, double time_s, double seconds, const State& pose, std::optional<std::size_t> cone, std::string reason) {
    events_.push_back({kind, time_s, static_cast<int>(lap_times_.size()), seconds, cone, {pose.x_m, pose.y_m}, std::move(reason)});
}

void Judge::disqualify(double time_s, const State& pose, std::string reason) {
    dnf_ = true;
    dnf_reason_ = reason;
    add(TimingKind::dnf, time_s, 0, pose, {}, std::move(reason));
}

void Judge::observe(const State& pose, double t) {
    if (dnf_ || stopped_) return;
    const Vec2 p{pose.x_m, pose.y_m};
    // Timekeeping: the rear axle passing from behind a gate's line to ahead of it, between its ends.
    for (std::size_t g = 0; g < course_.gates.size(); ++g) {
        const auto& gate = course_.gates[g];
        const Vec2 across = sub(gate.right, gate.left);
        const double length = norm(across);
        const Vec2 forward{-across.y/length, across.x/length};
        const double along = dot(sub(p, gate.left), across)/length;
        const double side = along >= 0 && along <= length ? dot(sub(p, gate.left), forward) : std::numeric_limits<double>::quiet_NaN();
        const double before = previous_side_[g];
        previous_side_[g] = side;
        if (!observed_ || std::isnan(side) || std::isnan(before) || !(before <= 1e-9 && side > 1e-9) || finished_) continue;
        if (g == 0) {
            if (!started_) {
                started_ = true;
                start_time_s_ = lap_start_s_ = last_trigger_s_ = t;
                add(TimingKind::start, t, 0, pose);
                continue;
            }
            if (course_.gates.size() > 1) current_sectors_.push_back(t-last_trigger_s_);
            lap_times_.push_back(t-lap_start_s_);
            sector_times_.push_back(current_sectors_);
            current_sectors_.clear();
            events_.push_back({TimingKind::lap, t, static_cast<int>(lap_times_.size())-1, lap_times_.back(), {}, p, {}});
            lap_start_s_ = last_trigger_s_ = t;
            if (laps() >= laps_required()) {
                finished_ = true;
                finished_at_s_ = t;
                add(TimingKind::finish, t, elapsed_s(t), pose);
            }
        } else if (started_) {
            current_sectors_.push_back(t-last_trigger_s_);
            add(TimingKind::sector, t, current_sectors_.back(), pose);
            last_trigger_s_ = t;
        }
    }
    observed_ = true;
    const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
    const auto at = [&](double ahead, double left) { return Vec2{p.x+c*ahead-s*left, p.y+s*ahead+c*left}; };
    // Off course: all four wheels outside the course's boundary.
    const auto& f = footprint_;
    bool any_on = false;
    for (const Vec2 wheel : {at(0, f.half_track_m), at(0, -f.half_track_m), at(f.wheelbase_m, f.half_track_m), at(f.wheelbase_m, -f.half_track_m)})
        any_on = any_on || within_corridor(course_.track, project(course_.track, wheel), 0);
    if (!any_on && !off_course_) {
        ++off_course_count_;
        penalty_s_ += rules_.off_course_s;
        off_course_since_s_ = t;
        add(TimingKind::off_course, t, rules_.off_course_s, pose);
    } else if (any_on && off_course_) {
        add(TimingKind::back_on_course, t, t-off_course_since_s_, pose);
    }
    off_course_ = !any_on;
    if (off_course_ && t-off_course_since_s_ >= rules_.off_course_limit_s) {
        disqualify(t, pose, "off course for "+std::to_string(static_cast<int>(rules_.off_course_limit_s))+" s");
        return;
    }
    // Cones down or out: the body touching a cone's base, each cone once.
    const std::array<Vec2, 4> body{at(f.body_front_m, f.body_half_width_m), at(-f.body_rear_m, f.body_half_width_m),
                                   at(-f.body_rear_m, -f.body_half_width_m), at(f.body_front_m, -f.body_half_width_m)};
    const double reach = std::hypot(f.body_front_m, f.body_half_width_m)+f.cone_width_m;
    const double h = f.cone_width_m/2;
    for (std::size_t i = 0; i < course_.cones.size(); ++i) {
        if (hit_[i]) continue;
        const Vec2 q = course_.cones[i].position;
        if (norm(sub(q, p)) > reach) continue;
        const std::array<Vec2, 4> base{Vec2{q.x+h, q.y+h}, Vec2{q.x-h, q.y+h}, Vec2{q.x-h, q.y-h}, Vec2{q.x+h, q.y-h}};
        if (!overlap(body, base)) continue;
        hit_[i] = true;
        ++cones_hit_;
        penalty_s_ += rules_.cone_hit_s;
        add(TimingKind::cone_hit, t, rules_.cone_hit_s, pose, i);
    }
    // After the finish: at rest near the line, or an unsafe stop.
    if (finished_) {
        const auto& line = course_.gates.front();
        const double distance = segment_distance(line.left, line.right, p);
        if (pose.speed_mps < rules_.stopped_below_mps && distance < rules_.stop_zone_m) {
            stopped_ = true;
            add(TimingKind::stopped, t, t-finished_at_s_, pose);
        } else if (distance >= rules_.stop_zone_m || t-finished_at_s_ >= rules_.stop_within_s) {
            if (rules_.discipline == Discipline::autocross) { disqualify(t, pose, "unsafe stop"); return; }
            stopped_ = true;
            penalty_s_ += rules_.unsafe_stop_s;
            add(TimingKind::unsafe_stop, t, rules_.unsafe_stop_s, pose);
        }
        return;
    }
    // Timeouts.
    if (!started_ && t > rules_.start_timeout_s) disqualify(t, pose, "did not start");
    else if (started_ && rules_.discipline == Discipline::autocross && t-start_time_s_ > rules_.autocross_timeout_s) disqualify(t, pose, "timeout");
    else if (started_ && rules_.discipline == Discipline::trackdrive && laps() == 0 && t-start_time_s_ > rules_.first_lap_timeout_s)
        disqualify(t, pose, "first lap timeout");
    else if (started_ && rules_.discipline == Discipline::trackdrive && t-start_time_s_ > rules_.total_timeout_s) disqualify(t, pose, "timeout");
}

} // namespace fd
