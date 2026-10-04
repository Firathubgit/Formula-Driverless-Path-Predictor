#include "fd/kart_coach.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// The coaching read from the poster's line (decision 0049): corners found from the line's curvature alone, the calls a driver
// is given along it, and the steering angle its bends ask for.

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double wheelbase = 1.045;
constexpr double step = 0.25;
const std::filesystem::path tracks_directory{FD_KART_DIR};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// A stadium: two straights of `straight` metres joined by two half circles of `radius`, a point every 0.25 m, from the
// start of the bottom straight, anticlockwise (bends to the left) or clockwise. Brake is advised from 18 m to 6 m before the
// end of each straight.
std::vector<fd::CoachSample> stadium(double straight, double radius, bool anticlockwise) {
    std::vector<fd::CoachSample> line;
    const double lap = 2 * straight + 2 * pi * radius;
    for (double s = 0; s < lap - 1e-9; s += step) {
        fd::Vec2 p{};
        double on_straight = -1;
        if (s < straight) { p = {s, 0}; on_straight = s; }
        else if (s < straight + pi * radius) {
            const double a = (s - straight) / radius;
            p = {straight + radius * std::sin(a), radius - radius * std::cos(a)};
        } else if (s < 2 * straight + pi * radius) {
            const double d = s - straight - pi * radius;
            p = {straight - d, 2 * radius};
            on_straight = d;
        } else {
            const double a = (s - 2 * straight - pi * radius) / radius;
            p = {-radius * std::sin(a), radius + radius * std::cos(a)};
        }
        if (!anticlockwise) p.y = -p.y;
        line.push_back({p, on_straight >= straight - 18 && on_straight < straight - 6 ? fd::Advice::Brake : fd::Advice::Accelerate});
    }
    return line;
}

// The point of a uniformly spaced line at a station.
fd::Vec2 at_station(const std::vector<fd::CoachSample>& line, double station) {
    return line[static_cast<std::size_t>(std::llround(station / step)) % line.size()].position;
}
// Whether a station lies from `from` to `to` going forward round a loop of `length`, as a corner that crosses the line does.
bool between(double station, double from, double to, double length) {
    const auto wrapped = [&](double v) { v = std::fmod(v, length); return v < 0 ? v + length : v; };
    return wrapped(station - from) <= wrapped(to - from);
}

void a_stadium_has_two_corners_and_steers_into_each() {
    for (const bool left : {true, false}) {
        const auto line = stadium(40, 8, left);
        const fd::KartCoach coach(line, wheelbase);
        require(std::abs(coach.length_m() - (80 + 2 * pi * 8)) < 0.3, "the loop's length is its own");
        require(coach.corners().size() == 2, "two bends are two corners");
        for (const auto& corner : coach.corners()) {
            require(corner.direction == (left ? 1 : -1), "a corner knows which way it turns");
            require(std::abs(corner.radius_m - 8) < 0.6, "and its radius");
            require(between(corner.apex_m, corner.turn_in_m, corner.exit_m, coach.length_m()) &&
                        between(corner.exit_m, corner.apex_m, corner.turn_in_m, coach.length_m()),
                    "turn-in, apex, exit in order");
        }
        require(coach.corners()[0].number == 1 && coach.corners()[1].number == 2, "numbered in lap order");
        require(std::abs(coach.corners()[0].apex_m - (40 + pi * 4)) < 1.5, "the apex is the middle of the bend");
        const auto apex = coach.cues(at_station(line, coach.corners()[0].apex_m), 8);
        require(apex.located && apex.in_corner && apex.corner == 1 && apex.direction == (left ? 1 : -1), "inside the first corner");
        require(apex.call == fd::CoachCall::Apex && std::abs(apex.to_apex_m) < 0.5, "at its apex the call is the apex");
        const double expected = (left ? 1 : -1) * std::atan(wheelbase / 8);
        require(std::abs(apex.advised_steering_rad - expected) < 0.01,
                "the steering a bend asks for is the wheelbase's over its radius, positive to the left");
        require(std::abs(apex.offset_m) < 0.01, "on the line the car's offset is nothing");
    }
}

void calls_follow_the_brake_zone_and_the_turn() {
    const auto line = stadium(40, 8, true);
    const fd::KartCoach coach(line, wheelbase);
    // The first red begins 22 m along the bottom straight and ends at 34, 6 m before the bend.
    const auto early = coach.cues(at_station(line, 12), 8);
    require(early.call == fd::CoachCall::Gas && early.advice == fd::Advice::Accelerate, "on the straight the call is gas");
    require(std::abs(early.to_brake_m - 10) < 0.4, "and the brake zone is counted down");
    require(!early.in_corner && early.corner == 1 && std::abs(early.to_turn_in_m - (coach.corners()[0].turn_in_m - 12)) < 0.4,
            "the next corner is the first, and its turn-in is counted down");
    const auto near_brake = coach.cues(at_station(line, 20), 8);
    require(near_brake.call == fd::CoachCall::Brake && near_brake.advice == fd::Advice::Accelerate,
            "a little before the red, at speed, the call is brake, early by the time the driver needs");
    const auto braking = coach.cues(at_station(line, 27), 8);
    require(braking.call == fd::CoachCall::Brake && braking.to_brake_m == 0 && braking.advice == fd::Advice::Brake, "in the red it is brake");
    const double turn_in = coach.corners()[0].turn_in_m;
    const auto coming = coach.cues(at_station(line, turn_in - 1.5), 2);
    require(coming.call == fd::CoachCall::TurnIn, "just before the bend, with the brake over, turn in");
    const auto after = coach.cues(at_station(line, coach.corners()[0].apex_m + 3), 8);
    require(after.in_corner && after.to_apex_m < -2 && after.call == fd::CoachCall::Gas, "past the apex, the gas");
    // Standing still: no lead, so the calls wait for the line itself.
    const auto still = coach.cues(at_station(line, 19), 0);
    require(still.call == fd::CoachCall::Gas, "at a standstill nothing needs to be called early");
}

void the_car_beside_the_line_is_placed_and_the_hint_keeps_it_on_its_own_leg() {
    const auto line = stadium(40, 8, true);
    const fd::KartCoach coach(line, wheelbase);
    const auto left = coach.cues({20, 1}, 8);
    require(left.located && std::abs(left.offset_m - 1) < 0.01 && std::abs(left.along_m - 20) < 0.3, "one metre left of the line is +1");
    const auto right = coach.cues({20, -1}, 8);
    require(right.located && std::abs(right.offset_m + 1) < 0.01, "one metre right is -1");
    require(!coach.cues({20, -9}, 8).located, "nine metres away, the car is off the line");
    require(std::abs(coach.cues({20, 15.5}, 8).along_m - (40 + pi * 8 + 20)) < 1, "without a hint, the top leg is the nearest line");
    require(!coach.cues({20, 15.5}, 8, 20.0).located, "with a hint on the bottom leg, the top leg is not mistaken for it");
    const double length = coach.length_m();
    const auto before = coach.cues({-0.1, 0.0}, 8, length - 0.3);
    require(before.located && (before.along_m > length - 1 || before.along_m < 1), "the line is a loop through the start");
}

void bad_lines_are_refused() {
    const auto good = stadium(40, 8, true);
    const auto refused = [](const std::vector<fd::CoachSample>& line, double wheelbase_m, const std::string& what) {
        try { fd::KartCoach coach(line, wheelbase_m); }
        catch (const std::invalid_argument&) { return; }
        throw std::runtime_error(what + " is not refused");
    };
    refused({good[0], good[1]}, wheelbase, "two points");
    refused(good, 0.0, "a zero wheelbase");
    refused(good, -1.0, "a negative wheelbase");
    refused(good, std::nan(""), "a wheelbase that is not a number");
    auto repeated = good;
    repeated[5] = repeated[4];
    refused(repeated, wheelbase, "a repeated point");
    auto broken = good;
    broken[7].position.x = std::nan("");
    refused(broken, wheelbase, "a point that is not a number");
    auto gap = good;
    gap.erase(gap.begin() + 20, gap.begin() + 40);
    refused(gap, wheelbase, "a gap of five metres");
}

std::vector<fd::CoachSample> poster_guide() {
    std::ifstream file(tracks_directory / "gokartcentralen-goteborg-guide.csv");
    require(file.good(), "the poster's coloured line is bundled");
    std::vector<fd::CoachSample> line;
    for (std::string text; std::getline(file, text);) {
        if (text.empty() || text[0] == '#' || text.rfind("x_m,", 0) == 0) continue;
        const auto first = text.find(','), second = text.find(',', first + 1);
        require(first != std::string::npos && second != std::string::npos, "three fields");
        const auto word = text.substr(second + 1);
        const auto advice = word.rfind("brake", 0) == 0 ? fd::Advice::Brake : word.rfind("coast", 0) == 0 ? fd::Advice::Coast : fd::Advice::Accelerate;
        line.push_back({{std::stod(text.substr(0, first)), std::stod(text.substr(first + 1, second - first - 1))}, advice});
    }
    return line;
}

void the_poster_line_has_the_tracks_ten_turns_in_order() {
    const auto line = poster_guide();
    const fd::KartCoach coach(line, wheelbase);
    require(coach.corners().size() == 10, "the poster's ten turns: " + std::to_string(coach.corners().size()));
    // T1 right, T2 a left kink, T3 right, T4 right, T5 left, T6 left, T7 right, T8 right, T9 left, T10 right (decision 0038).
    const std::string expected = "RLRRLLRRLR";
    std::string found;
    double last = -1;
    for (const auto& corner : coach.corners()) {
        found += corner.direction > 0 ? 'L' : 'R';
        require(corner.apex_m > last, "in lap order");
        last = corner.apex_m;
        require(corner.radius_m > 2.5 && corner.radius_m < 20, "its radius is a kart track's");
        require(between(corner.apex_m, corner.turn_in_m, corner.exit_m, coach.length_m()), "its apex is between turn-in and exit");
        const auto cue = coach.cues(coach.position_at(corner.apex_m), 8);
        require(cue.located && cue.corner == corner.number && cue.in_corner, "the car at its apex is in it");
        require(cue.call == fd::CoachCall::Apex, "and is called to the apex");
        require(cue.advised_steering_rad * corner.direction > 0.045 && cue.advised_steering_rad * corner.direction < 0.7,
                "the steering it asks for is to its own side and a kart's: " + std::to_string(cue.advised_steering_rad));
    }
    require(found == expected, "the turns are " + found + ", not " + expected);
}

void walking_the_poster_line_calls_every_corner_and_every_red() {
    const auto line = poster_guide();
    const fd::KartCoach coach(line, wheelbase);
    std::set<int> apexed;
    int braking = 0, lifted = 0, turned_in = 0;
    double hint = -1, last_along = 0;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const auto cue = coach.cues(line[i].position, 8, hint);
        require(cue.located && std::abs(cue.offset_m) < 0.15, "every point of the line is on it");
        if (i > 0) {
            const double advance = cue.along_m - last_along;
            require((advance > -0.05 && advance < 1.0) || advance < -coach.length_m() + 1.0, "walking the line moves the station forward");
        }
        hint = last_along = cue.along_m;
        require(cue.call != fd::CoachCall::Apex || cue.in_corner, "an apex is only ever called inside a corner");
        if (cue.call == fd::CoachCall::Apex) apexed.insert(cue.corner);
        if (cue.call == fd::CoachCall::Brake) ++braking;
        if (cue.call == fd::CoachCall::Lift) ++lifted;
        if (cue.call == fd::CoachCall::TurnIn) ++turned_in;
    }
    require(apexed.size() == 10, "every corner is called to its apex: " + std::to_string(apexed.size()));
    require(braking > 100 && lifted > 50 && turned_in > 10, "and the line's reds, blues and turn-ins are all called");
}
} // namespace

int main() {
    const std::vector<std::pair<const char*, void (*)()>> tests{
        {"a_stadium_has_two_corners_and_steers_into_each", a_stadium_has_two_corners_and_steers_into_each},
        {"calls_follow_the_brake_zone_and_the_turn", calls_follow_the_brake_zone_and_the_turn},
        {"the_car_beside_the_line_is_placed_and_the_hint_keeps_it_on_its_own_leg", the_car_beside_the_line_is_placed_and_the_hint_keeps_it_on_its_own_leg},
        {"bad_lines_are_refused", bad_lines_are_refused},
        {"the_poster_line_has_the_tracks_ten_turns_in_order", the_poster_line_has_the_tracks_ten_turns_in_order},
        {"walking_the_poster_line_calls_every_corner_and_every_red", walking_the_poster_line_calls_every_corner_and_every_red}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size() - static_cast<std::size_t>(failures) << '/' << tests.size() << " kart coach groups passed\n";
    return failures ? 1 : 0;
}
