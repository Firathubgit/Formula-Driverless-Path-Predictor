#include "fd/simulation.hpp"
#include "fd/track_conditioning.hpp"
#include "fd/vehicle_profiles.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// The rental kart fitted to the poster's coloured line (decision 0046): the plant driven round the line the poster paints,
// Pure Pursuit aiming 2 m ahead as the desktop's kart is driven, its speed planned from a share of its own envelope, and
// timed by the judge from the start line. The share is the effort of the driver: the whole envelope is the kart's limit,
// and the real laps the user and the operator give (34.0 s for a quick kart, 36.38 s a good driver's best) lie between the
// limit and about four fifths of it. tools/kart/fit_kart.py is how the numbers were chosen and re-checks them.

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

const std::filesystem::path tracks_directory{FD_KART_DIR};

fd::Track poster_track() {
    fd::ConditioningOptions options;
    options.smoothing_rms_m = 0.05;
    return fd::condition_track(fd::load_centreline(tracks_directory/"gokartcentralen-goteborg-poster-line.csv"), 3.0, "poster line", options).track;
}

struct Result { std::vector<double> laps; int cones{}, off_course{}; double flying() const { return laps.empty() ? 0.0 : laps.back(); } };

Result drive(const std::string& profile_id, double share, int laps) {
    const auto& profile = fd::vehicle_profile(profile_id);
    auto config = fd::configured_for(profile, fd::Config{});
    config.lookahead_base_m = 2.0;
    config.envelope_fraction = share;
    fd::Simulation simulation(poster_track(), config, profile.car, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope);
    simulation.set_course({}, fd::make_gates(simulation.track()), "the poster's line: barriers, no cones");
    for (int k = 0; k < static_cast<int>(laps*45/config.fixed_dt_s) && simulation.judge().laps() < laps && !simulation.judge().dnf(); ++k)
        simulation.step();
    Result r;
    r.laps = simulation.judge().lap_times();
    r.cones = simulation.judge().cones_hit();
    r.off_course = simulation.judge().off_course_count();
    return r;
}

std::string laps_text(const std::vector<double>& laps) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < laps.size(); ++i) text << (i ? ", " : "") << laps[i];
    return laps.empty() ? "none" : text.str();
}

// The poster's line is the one the track gives the kart: 362.6 m of it, bending no tighter than a kart can turn, and the file
// the tests and the tool drive is that line smoothed.
void the_poster_line_is_a_closed_line_a_kart_can_turn() {
    const auto track = poster_track();
    double tightest = 0;
    for (const auto& p : track.points) tightest = std::max(tightest, std::abs(p.curvature));
    std::cout << "  the poster's line: " << track.length_m << " m, " << track.points.size() << " samples, tightest radius " << 1/tightest << " m\n";
    require(track.length_m > 355 && track.length_m < 370, "the poster's inside line is about 363 m, shorter than the track's 400 m middle");
    require(1/tightest > 2.5, "no bend tighter than a 2.5 m radius, which a kart with 0.55 rad of steering lock can turn");
}

// The fitted kart on the poster's line at the whole of its envelope, at nine tenths and at four fifths: laps of about
// 34.5, 36.1 and 38.5 s, every one clean. The earlier rental kart, which the same line spun at its limit, laps over 40 s
// at 85% of it: the fit is worth about five seconds on the line.
void the_fitted_kart_laps_the_poster_line_as_real_karts_do() {
    const auto limit = drive("gokartcentralen-realistic", 1.0, 2);
    const auto good = drive("gokartcentralen-realistic", 0.9, 2);
    const auto average = drive("gokartcentralen-realistic", 0.8, 2);
    const auto earlier = drive("gokartcentralen-rsx2", 0.85, 2);
    std::cout << "  fitted kart at 100% of its envelope: " << laps_text(limit.laps) << " s; at 90%: " << laps_text(good.laps) << " s; at 80%: "
              << laps_text(average.laps) << " s\n  earlier rental kart at 85%: " << laps_text(earlier.laps) << " s\n";
    for (const auto* run : {&limit, &good, &average})
        require(run->laps.size() >= 2 && run->cones == 0 && run->off_course == 0, "every share of its limit drives two clean laps");
    require(limit.flying() > 34.0 && limit.flying() < 35.0,
            "at the whole of its envelope the kart laps in about 34.5 s, the operator's own standard for a quick kart being 34.0 s");
    require(good.flying() > 35.7 && good.flying() < 36.6 && good.flying() < 36.38 && average.flying() > 36.38,
            "and a good driver's 36.38 s lies between nine tenths and four fifths of it");
    require(average.flying() > 38.0 && average.flying() < 39.0, "while four fifths of it laps in 38 to 39 s, an average driver's lap");
    require(limit.flying() < good.flying() && good.flying() < average.flying(), "less effort is a slower lap");
    require(earlier.laps.size() >= 2 && earlier.flying() > 40.0 && good.flying() < earlier.flying()-4.0,
            "the earlier rental kart laps over 40 s even at 85%, four seconds slower than the fitted kart at 90%");
}
} // namespace

int main() {
    const std::vector<std::pair<const char*, void (*)()>> tests{
        {"the_poster_line_is_a_closed_line_a_kart_can_turn", the_poster_line_is_a_closed_line_a_kart_can_turn},
        {"the_fitted_kart_laps_the_poster_line_as_real_karts_do", the_fitted_kart_laps_the_poster_line_as_real_karts_do}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " kart fit groups passed\n";
    return failures ? 1 : 0;
}
