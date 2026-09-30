#include "fd/competition.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <class F> bool refused(F&& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; }
    return false;
}

fd::Course preset_course(double width = 5) {
    auto track = fd::make_preset_track();
    track.width_m = width;
    return {track, fd::make_cone_layout(track), fd::make_gates(track)};
}

// The rear axle's pose at a station and offset of the track, heading along it.
fd::State pose_on(const fd::Track& track, double s, double offset = 0, double speed = 10) {
    const auto here = fd::sample(track, s), ahead = fd::sample(track, s+0.05);
    const double tx = ahead.x_m-here.x_m, ty = ahead.y_m-here.y_m, n = std::hypot(tx, ty);
    return {here.x_m-ty/n*offset, here.y_m+tx/n*offset, std::atan2(ty, tx), speed, 0, 0};
}

// Drives the judge along the centreline at a steady speed for a number of laps, then as told.
struct Drive {
    fd::Judge& judge;
    double t{}, s{};
    const double dt{0.005};
    void run(double seconds, double speed, double offset = 0) {
        const auto& track = judge.course().track;
        for (int k = 0; k < static_cast<int>(std::lround(seconds/dt)); ++k) {
            auto pose = pose_on(track, std::fmod(s, track.length_m), offset, speed);
            pose.time_s = t;
            judge.observe(pose, t);
            t += dt;
            s += speed*dt;
        }
    }
};

std::size_t count(const fd::Judge& judge, fd::TimingKind kind) {
    return static_cast<std::size_t>(std::count_if(judge.events().begin(), judge.events().end(),
                                                  [&](const fd::TimingEvent& e) { return e.kind == kind; }));
}

// The start line starts the clock the tick the car leaves it; each return to it is a lap; the sector lines split it,
// their times summing to the lap's. A car on the centreline hits nothing and never leaves the course.
void laps_and_sectors_are_timed() {
    fd::CompetitionRules rules;
    rules.trackdrive_laps = 2;
    fd::Judge judge(preset_course(), rules);
    Drive drive{judge};
    const double length = judge.course().track.length_m;
    drive.run(2*length/10+0.5, 10);
    require(judge.started() && judge.events().front().kind == fd::TimingKind::start && judge.events().front().time_s == 0.005,
            "the clock starts the tick the car leaves the line");
    require(judge.laps() == 2 && judge.finished(), "two laps, the trackdrive asked for, finish it");
    for (const double lap : judge.lap_times()) require(std::abs(lap-length/10) < 0.006, "each lap its length at 10 m/s");
    require(judge.sector_times().size() == 2 && judge.sector_times()[0].size() == 3, "three sectors a lap");
    double sum = 0;
    for (const double sector : judge.sector_times()[0]) { sum += sector; require(std::abs(sector-length/30) < 0.011, "each a third of the lap"); }
    require(std::abs(sum-judge.lap_times()[0]) < 1e-9, "the sectors sum to the lap");
    require(judge.cones_hit() == 0 && judge.off_course_count() == 0 && judge.penalty_s() == 0, "the centreline is clean");
    require(std::abs(judge.elapsed_s(drive.t)-(judge.lap_times()[0]+judge.lap_times()[1])) < 1e-9, "the elapsed time is the laps' once finished");
    std::cout << "  two laps of " << judge.lap_times()[0] << " and " << judge.lap_times()[1] << " s at 10 m/s, sectors "
              << judge.sector_times()[0][0] << ", " << judge.sector_times()[0][1] << ", " << judge.sector_times()[0][2] << " s\n";
}

// A cone the body touches is down or out once, for 2 s; a car beside it that does not touch it pays nothing.
void a_cone_touched_costs_two_seconds_once() {
    auto course = preset_course();
    const auto& track = course.track;
    // A cone placed in the lane, 200 m on and 0.7 m left of the centreline, where the body's side passes.
    const auto p = pose_on(track, 200, 0.7);
    course.cones.push_back({{p.x_m, p.y_m}, fd::ConeColour::orange});
    const std::size_t placed = course.cones.size()-1;
    {
        fd::Judge judge(course);
        Drive drive{judge};
        drive.run(30, 10, 0);
        require(judge.cones_hit() == 1 && judge.hit()[placed] && judge.penalty_s() == 2, "a body 0.7 m wide either side touches a cone whose centre is 0.7 m aside");
        const auto hit = std::find_if(judge.events().begin(), judge.events().end(), [](const fd::TimingEvent& e) { return e.kind == fd::TimingKind::cone_hit; });
        require(hit->cone == placed && hit->seconds == 2, "the event names the cone and its two seconds");
    }
    {
        fd::Judge judge(course);
        Drive drive{judge};
        drive.run(30, 10, -0.5);
        require(judge.cones_hit() == 0, "a car whose side passes 0.5 m from the cone's centre does not touch its 0.228 m base");
    }
}

// All four wheels outside the course is off course, 10 s each time; eight seconds of it is a DNF.
void off_course_costs_ten_seconds_and_eight_is_a_dnf() {
    auto course = preset_course();
    course.cones.clear();  // nothing to knock over while off course
    {
        fd::Judge judge(course);
        Drive drive{judge};
        drive.run(5, 10, 0);
        drive.run(2, 10, 2.5+0.6+0.1);  // the near wheels 0.1 m beyond the edge
        drive.run(2, 10, 0);
        require(judge.off_course_count() == 1 && judge.penalty_s() == 10 && !judge.dnf(), "one excursion, ten seconds");
        require(count(judge, fd::TimingKind::back_on_course) == 1, "and its end");
        const auto back = std::find_if(judge.events().begin(), judge.events().end(), [](const fd::TimingEvent& e) { return e.kind == fd::TimingKind::back_on_course; });
        require(std::abs(back->seconds-2) < 0.011, "which lasted two seconds");
    }
    {
        fd::Judge judge(course);
        Drive drive{judge};
        drive.run(5, 10, 0);
        drive.run(2, 10, 2.5+0.6-0.1);  // the near wheels still inside
        require(judge.off_course_count() == 0, "a wheel inside keeps the car on course");
        drive.run(9, 10, 5);
        require(judge.dnf() && judge.dnf_reason() == "off course for 8 s", "eight seconds off course is a DNF");
        const auto events = judge.events().size();
        drive.run(1, 10, 0);
        require(judge.events().size() == events, "and nothing is judged after it");
    }
}

// After the finish the car must stop within 30 m of the line: at rest there it stopped safely; driving on, a trackdrive
// pays 10 s and an autocross is a DNF.
void the_car_must_stop_after_the_finish() {
    auto course = preset_course();
    course.cones.clear();
    const auto finish = [&](fd::Discipline discipline, double then_speed) {
        fd::CompetitionRules rules;
        rules.discipline = discipline;
        rules.trackdrive_laps = 1;
        auto judge = std::make_unique<fd::Judge>(course, rules);
        Drive drive{*judge};
        drive.run(course.track.length_m/10+0.5, 10);
        require(judge->finished(), "the lap finishes the run");
        drive.run(10, then_speed);
        return judge;
    };
    const auto rolling = finish(fd::Discipline::trackdrive, 10);
    require(count(*rolling, fd::TimingKind::unsafe_stop) == 1 && rolling->penalty_s() == 10, "driving on past 30 m is an unsafe stop in a trackdrive");
    const auto autocross = finish(fd::Discipline::autocross, 10);
    require(autocross->dnf() && autocross->dnf_reason() == "unsafe stop", "and a DNF in an autocross");
    const auto standing = finish(fd::Discipline::trackdrive, 0);
    require(standing->stopped() && count(*standing, fd::TimingKind::stopped) == 1 && standing->penalty_s() == 0, "at rest near the line is a safe stop");
}

// A car that never reaches the start line in a minute does not start.
void timeouts_end_the_run() {
    fd::Judge judge(preset_course());
    for (int k = 0; k <= 12100; ++k) {
        auto pose = pose_on(judge.course().track, 400, 0, 0);
        judge.observe(pose, k*0.005);
    }
    require(judge.dnf() && judge.dnf_reason() == "did not start", "no start within 60 s is a DNF");
}

// PacSim's track format: lanes of coloured cones, timekeeping cones in pairs, a start pose; read into a closed course
// whose corridor is the cones' own and whose start line keeps its left end first.
void a_pacsim_layout_becomes_a_course() {
    const auto file = std::filesystem::temp_directory_path()/"fd-competition-oval.yaml";
    {
        std::ofstream out(file);
        out << "track:\n  version: 1.0\n  lanesFirstWithLastConnected: true\n  start:\n    position: [0.0, 0.0, 0.0]\n"
               "    orientation: [0.0, 0.0, 0.0]\n  earthToTrack:\n    position: [0.0, 0.0, 0.0]\n    orientation: [0.0, 0.0, 0.0]\n";
        // An oval 60 m by 30 m on its centreline, run counterclockwise from its bottom, 4 m wide.
        const auto lane = [&](const char* name, double offset, const char* cls) {
            out << "  " << name << ":\n";
            for (int k = 0; k < 48; ++k) {
                const double a = -std::numbers::pi/2+2*std::numbers::pi*k/48;
                const double cx = 30*std::cos(a), cy = 15*std::sin(a)+15;
                const double nx = -std::cos(a)/30, ny = -std::sin(a)/15, n = std::hypot(nx, ny);
                out << "  - position: [" << cx+nx/n*offset << ", " << cy+ny/n*offset << ", 0.0]\n    class: " << cls << "\n";
            }
        };
        lane("left", 2, "blue");
        lane("right", -2, "yellow");
        out << "  time_keeping:\n  - position: [5.0, -3.0, 0.0]\n    class: timekeeping\n  - position: [5.0, 3.0, 0.0]\n    class: timekeeping\n"
               "  unknown: []\n";
    }
    const auto layout = fd::load_pacsim_course(file);
    require(layout.closed && layout.left.size() == 48 && layout.right.size() == 48 && layout.gates.size() == 1, "the layout's lanes and gate are read");
    const auto course = fd::course_from_layout(layout);
    require(course.cones.size() == 96 && course.gates.size() == 1, "its cones and gate make the course");
    require(course.gates[0].left.y > course.gates[0].right.y, "the start line's left end, seen running along the bottom, is the upper one");
    // The lanes are straight between cones 3 m apart, so the edges fall short of 2 m by up to a chord's sagitta, most on
    // the ends' 5.5 m inner radius.
    double worst = 0, sum = 0;
    for (std::size_t i = 0; i < course.track.points.size(); ++i) {
        worst = std::max({worst, std::abs(course.track.left_edge_m[i]-2), std::abs(course.track.right_edge_m[i]-2)});
        sum += std::abs(course.track.left_edge_m[i]-2)+std::abs(course.track.right_edge_m[i]-2);
    }
    const double mean = sum/static_cast<double>(2*course.track.points.size());
    const auto start = course.track.points.front();
    std::cout << "  a traced oval: " << course.track.length_m << " m, corridor edges a mean " << mean << " m and at most " << worst
              << " m from the cones' 2 m, starting at (" << start.x_m << ", " << start.y_m << ")\n";
    require(mean < 0.1 && worst < 0.4, "the corridor is the cones' own, within the chords between them");
    require(std::hypot(start.x_m, start.y_m) < 2.5, "and it starts beside the start pose");
    fd::Judge judge(course);
    Drive drive{judge};
    drive.run(course.track.length_m/10+2, 10);
    require(judge.laps() == 1 && judge.cones_hit() == 0 && judge.off_course_count() == 0, "a lap of it is clean");
    std::filesystem::remove(file);
    // What cannot be read is refused by line.
    const auto broken = std::filesystem::temp_directory_path()/"fd-competition-broken.yaml";
    { std::ofstream out(broken); out << "track:\n  left:\n  - position: [1.0, x, 0.0]\n    class: blue\n"; }
    bool named = false;
    try { fd::load_pacsim_course(broken); } catch (const std::invalid_argument& e) { named = std::string(e.what()).find("line 3") != std::string::npos; }
    require(named, "a position that is not a number is refused, naming its line");
    std::filesystem::remove(broken);
    fd::CourseFile open = layout;
    open.closed = false;
    require(refused([&] { fd::course_from_layout(open); }), "an open layout is refused");
}

void practice_laps_start_on_gas_and_require_a_circuit() {
    const auto track=preset_course().track;
    fd::PracticeLaps clock(track);
    double time=10;
    auto observe=[&](double from,double to,double throttle=0.0,double dt=0.1,double offset=0.0) {
        auto a=pose_on(track,from,offset),b=pose_on(track,to,offset);
        a.time_s=time; b.time_s=time+dt; clock.observe(a,b,throttle); time+=dt;
    };
    observe(0,0);
    require(!clock.started() && clock.current_seconds(time)==0,"waiting or pressing Play alone does not time a lap");
    observe(0,0,1);
    require(clock.started() && std::abs(clock.current_seconds(time)-0.1)<1e-9,"gas starts at the beginning of its tick");
    observe(-0.1,0.1); observe(0.1,-0.1); observe(-0.1,0.1);
    require(clock.completed()==0 && clock.previous_seconds()==0,"rocking across the line is not a full lap");
    // Missing a checkpoint cannot be recovered by merely crossing the start.
    observe(track.length_m/6-0.1,track.length_m/6+0.1);
    observe(-0.1,0.1);
    require(clock.completed()==0,"out-of-order checkpoints cannot complete a lap");
    auto circuit=[&](double dt) {
        for(int i=1;i<=12;++i) {
            const double s=track.length_m*i/12;
            observe(s-0.1,s+0.1,0,dt);
        }
    };
    circuit(1);
    require(clock.completed()==1 && clock.previous_seconds()>12 && clock.best_seconds()==clock.previous_seconds(),"first whole lap is retained");
    require(std::abs(clock.current_seconds(time)-0.5)<0.01,"crossing time is interpolated into the new lap");
    const double previous=clock.previous_seconds();
    observe(0,0);
    require(clock.previous_seconds()==previous,"previous means completed lap, never the running timer");
    circuit(0.5);
    require(clock.completed()==2 && clock.best_seconds()==clock.previous_seconds() && clock.best_seconds()<previous,"best updates on a faster full lap");
    const double best=clock.best_seconds();
    circuit(1);
    require(clock.completed()==3 && clock.previous_seconds()>best && clock.best_seconds()==best,"slower lap updates previous but preserves best");
    clock.reset();
    require(!clock.started() && clock.current_seconds(0)==0 && clock.previous_seconds()>0 && clock.completed()==3,"restart preserves completed history and waits for gas");
    clock.reset(false);
    require(clock.completed()==0 && clock.previous_seconds()==0 && clock.best_seconds()==0,"new session clears history");
    observe(0,0,1);
    for(int i=1;i<=12;++i) {
        double s=track.length_m*i/12;
        observe(s+0.1,s-0.1);
    }
    require(clock.completed()==0,"reverse crossings never count");
    for(int i=1;i<=12;++i) {
        double s=track.length_m*i/12;
        observe(s-0.1,s+0.1,0,1,track.width_m);
    }
    require(clock.completed()==0,"passing outside the timing gates never counts");
}

void bad_rules_are_refused() {
    fd::CompetitionRules rules;
    rules.trackdrive_laps = 0;
    require(refused([&] { fd::Judge judge(preset_course(), rules); }), "a trackdrive of no laps is refused");
    fd::Footprint footprint;
    footprint.body_front_m = 1;
    require(refused([&] { fd::Judge judge(preset_course(), {}, footprint); }), "a body short of its front axle is refused");
    auto course = preset_course();
    course.gates.clear();
    require(refused([&] { fd::Judge judge(course); }), "a course with no start line is refused");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"laps_and_sectors_are_timed", laps_and_sectors_are_timed},
        {"practice_laps_start_on_gas_and_require_a_circuit", practice_laps_start_on_gas_and_require_a_circuit},
        {"a_cone_touched_costs_two_seconds_once", a_cone_touched_costs_two_seconds_once},
        {"off_course_costs_ten_seconds_and_eight_is_a_dnf", off_course_costs_ten_seconds_and_eight_is_a_dnf},
        {"the_car_must_stop_after_the_finish", the_car_must_stop_after_the_finish},
        {"timeouts_end_the_run", timeouts_end_the_run},
        {"a_pacsim_layout_becomes_a_course", a_pacsim_layout_becomes_a_course},
        {"bad_rules_are_refused", bad_rules_are_refused}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " competition groups passed\n";
    return failures ? 1 : 0;
}
