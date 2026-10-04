#include "fd/track_conditioning.hpp"
#include "fd/track_relief.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// The ground a banked bend gives the display (decision 0050): its tilt over its ramps, the surface across the road, and the
// one raised bend of Gokartcentralen Göteborg.

namespace {
const std::filesystem::path tracks_directory{FD_KART_DIR};
constexpr double degrees = std::numbers::pi / 180;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// The point at a station along the track and an offset from it, left positive.
fd::Vec2 at(const fd::Track& track, double s, double offset) {
    const auto p = fd::sample(track, s), q = fd::sample(track, s + 0.2);
    const double dx = q.x_m - p.x_m, dy = q.y_m - p.y_m, n = std::hypot(dx, dy);
    return {p.x_m - dy / n * offset, p.y_m + dx / n * offset};
}

void a_flat_track_has_flat_ground() {
    const auto track = fd::make_preset_track();
    const fd::TrackRelief relief(track, {});
    require(relief.flat() && relief.ground_at({10, 1}).height_m == 0 && relief.ground_at({10, 1}).roll_rad == 0, "no banking, no relief");
    require(fd::gokartcentralen_relief(track).flat(), "and a track that is not the kart's is flat");
}

void a_banking_tilts_the_road_across_its_width() {
    const auto track = fd::make_preset_track();
    const double tilt = 6 * degrees;
    const fd::TrackRelief relief(track, {{100, 160, 10, tilt}});
    require(relief.tilt_at(99) == 0 && relief.tilt_at(161) == 0 && relief.tilt_at(10) == 0, "flat outside the banking");
    require(std::abs(relief.tilt_at(130) - tilt) < 1e-12, "at the full tilt through its middle");
    require(std::abs(relief.tilt_at(105) - tilt / 2) < 1e-9 && std::abs(relief.tilt_at(155) - tilt / 2) < 1e-9, "half way up its ramps");
    double last = 0;
    for (double s = 100; s <= 110; s += 0.25) { require(relief.tilt_at(s) >= last, "it only rises over the ramp"); last = relief.tilt_at(s); }
    const double half = track.width_m / 2;
    const auto left = relief.ground_at(at(track, 130, half)), middle = relief.ground_at(at(track, 130, 0)), right = relief.ground_at(at(track, 130, -half));
    require(std::abs(left.height_m) < 1e-6, "the left edge is at the floor");
    require(std::abs(right.height_m - track.width_m * std::tan(tilt)) < 1e-6, "the right edge is the width times the tangent above it");
    require(std::abs(middle.height_m - right.height_m / 2) < 1e-6, "linear between");
    require(std::abs(left.roll_rad - tilt) < 1e-9 && std::abs(right.roll_rad - tilt) < 1e-9, "the car on it rolls with it, right side higher");
    require(std::abs(relief.ground_at(at(track, 130, -half - 0.3)).height_m - right.height_m) < 1e-6, "under the barrier beyond the right edge the height holds");
    const auto behind = relief.ground_at(at(track, 130, -half - 0.4));
    require(behind.height_m == 0 && behind.roll_rad == 0 && relief.ground_at(at(track, 130, -half - 3)).height_m == 0,
            "behind the barrier the floor is flat again: the bank stands on a retaining wall");
    require(relief.ground_at(at(track, 130, half + 0.3)).height_m == 0 && relief.ground_at(at(track, 130, half + 3)).height_m == 0, "and beyond the left it never rose");
    require(relief.ground_at(at(track, 60, -half)).height_m == 0 && relief.ground_at({5000, 5000}).height_m == 0, "no rise away from it");
    const fd::TrackRelief reversed(track, {{100, 160, 10, -tilt}});
    require(reversed.ground_at(at(track, 130, half)).height_m > 0.5 && std::abs(reversed.ground_at(at(track, 130, -half)).height_m) < 1e-6 &&
                reversed.ground_at(at(track, 130, 0)).roll_rad < 0,
            "a negative tilt lifts the left edge instead");
}

void bad_bankings_are_refused() {
    const auto track = fd::make_preset_track();
    const auto refused = [&](std::vector<fd::Banking> bankings, const std::string& what) {
        try { fd::TrackRelief relief(track, std::move(bankings)); }
        catch (const std::invalid_argument&) { return; }
        throw std::runtime_error(what + " is not refused");
    };
    refused({{100, 160, 0, 0.1}}, "a ramp of nothing");
    refused({{100, 110, 10, 0.1}}, "a banking shorter than its ramps");
    refused({{-5, 60, 5, 0.1}}, "a banking before the line");
    refused({{100, track.length_m + 5, 5, 0.1}}, "a banking past the lap");
    refused({{100, 160, 10, 0.9}}, "a tilt of 50 degrees");
    refused({{100, 160, 10, std::nan("")}}, "a tilt that is not a number");
    refused({{100, 160, 10, 0.1}, {150, 200, 10, 0.1}}, "overlapping bankings");
}

void the_kart_tracks_ninth_bend_is_banked_into_the_left() {
    fd::ConditioningOptions options;
    options.smoothing_rms_m = 0.02;
    const auto track = fd::condition_track(fd::load_centreline(tracks_directory / "gokartcentralen-goteborg.csv"), 6.0, "Gokartcentralen Göteborg", options).track;
    const auto relief = fd::gokartcentralen_relief(track);
    require(!relief.flat() && relief.bankings().size() == 1, "the one raised bend");
    const double half = track.width_m / 2;
    // T9 is the left-hand bend 299 to 329 m round the lap: the road banks into it, the outside, on the right, the higher.
    const auto outside = relief.ground_at(at(track, 314, -half)), inside = relief.ground_at(at(track, 314, half));
    require(std::abs(inside.height_m) < 0.02 && outside.height_m > 0.5 && outside.height_m < 0.8,
            "the outside is six degrees across six metres, about 0.63 m, above the inside: " + std::to_string(outside.height_m));
    for (const double s : {50.0, 120.0, 200.0, 260.0, 285.0, 345.0, 380.0})
        require(relief.ground_at(at(track, s, -half)).height_m == 0 && relief.ground_at(at(track, s, half)).height_m == 0,
                "flat everywhere else: " + std::to_string(s));
    // The ground never jumps between one metre and the next along either edge.
    double last = 0;
    for (double s = 280; s < 345; s += 0.5) {
        const double h = relief.ground_at(at(track, s, -half)).height_m;
        require(std::abs(h - last) < 0.08, "the surface is continuous along the road at " + std::to_string(s));
        last = h;
    }
}
} // namespace

int main() {
    const std::vector<std::pair<const char*, void (*)()>> tests{
        {"a_flat_track_has_flat_ground", a_flat_track_has_flat_ground},
        {"a_banking_tilts_the_road_across_its_width", a_banking_tilts_the_road_across_its_width},
        {"bad_bankings_are_refused", bad_bankings_are_refused},
        {"the_kart_tracks_ninth_bend_is_banked_into_the_left", the_kart_tracks_ninth_bend_is_banked_into_the_left}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size() - static_cast<std::size_t>(failures) << '/' << tests.size() << " track relief groups passed\n";
    return failures ? 1 : 0;
}
