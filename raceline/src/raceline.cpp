#include "fd/raceline.hpp"

#include "osqp.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fd {
namespace {

// Linearised curvature is held this far inside the steering limit, so the refitted line still meets it.
constexpr double steering_share = 0.98;
// The refit that turns shifted samples back into a line keeps them this close, root mean square.
constexpr double refit_rms_m = 0.001;

// A sparse matrix gathered entry by entry and handed to OSQP in compressed columns, rows sorted within each column.
struct Entries {
    OSQPInt rows{}, cols{};
    std::map<std::pair<OSQPInt, OSQPInt>, OSQPFloat> by_column_row;
    void add(OSQPInt row, OSQPInt col, OSQPFloat value) { by_column_row[{col, row}] += value; }
};
class Compressed {
public:
    explicit Compressed(const Entries& entries) : column_start_(static_cast<std::size_t>(entries.cols+1), 0) {
        for (const auto& [key, value] : entries.by_column_row) {
            row_.push_back(key.second);
            value_.push_back(value);
            ++column_start_[static_cast<std::size_t>(key.first+1)];
        }
        for (std::size_t c = 0; c < static_cast<std::size_t>(entries.cols); ++c) column_start_[c+1] += column_start_[c];
        matrix_.m = entries.rows;
        matrix_.n = entries.cols;
        matrix_.p = column_start_.data();
        matrix_.i = row_.data();
        matrix_.x = value_.data();
        matrix_.nzmax = static_cast<OSQPInt>(value_.size());
        matrix_.nz = -1;
        matrix_.owned = 0;
    }
    Compressed(const Compressed&) = delete;
    Compressed& operator=(const Compressed&) = delete;
    const OSQPCscMatrix* get() const { return &matrix_; }
private:
    std::vector<OSQPInt> column_start_, row_;
    std::vector<OSQPFloat> value_;
    OSQPCscMatrix matrix_{};
};

struct Solution { OSQPInt status{}; std::vector<double> x; };
// Minimise x'Px/2 + q'x subject to l <= Ax <= u; P holds its upper triangle.
Solution solve(const Entries& P, const std::vector<double>& q, const Entries& A, const std::vector<double>& l, const std::vector<double>& u) {
    const Compressed hessian(P), constraints(A);
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polishing = 1;
    settings.eps_abs = 1e-7;
    settings.eps_rel = 1e-7;
    settings.max_iter = 100000;
    OSQPSolver* raw = nullptr;
    const OSQPInt setup = osqp_setup(&raw, hessian.get(), q.data(), constraints.get(), l.data(), u.data(), A.rows, P.cols, &settings);
    const std::unique_ptr<OSQPSolver, OSQPInt (*)(OSQPSolver*)> solver(raw, &osqp_cleanup);
    if (setup != 0) throw std::runtime_error(std::string("OSQP setup failed: ")+osqp_error_message(setup));
    osqp_solve(solver.get());
    Solution solution;
    solution.status = solver->info->status_val;
    if (solution.status == OSQP_SOLVED || solution.status == OSQP_SOLVED_INACCURATE)
        solution.x.assign(solver->solution->x, solver->solution->x+P.cols);
    return solution;
}

std::string metres(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(2);
    out << value << " m";
    return out.str();
}

double squared_curvature(const Track& track) {
    double sum = 0;
    const double step = track.length_m/static_cast<double>(track.points.size());
    for (const auto& p : track.points) sum += p.curvature*p.curvature*step;
    return sum;
}

} // namespace

RacingLine make_minimum_curvature_line(const ConditionedTrack& centreline, const Config& config, const RacingLineOptions& options) {
    validate_config(config);
    validate_track(centreline.track);
    const auto& reference = centreline.track;
    if (centreline.left_normals.size() != reference.points.size())
        throw std::invalid_argument("The centreline needs a left normal at every sample; condition it first");
    const auto within = [](double value, double low, double high) { return std::isfinite(value) && value >= low && value <= high; };
    if (!within(options.vehicle_half_width_m, 0, 5) || !within(options.margin_m, 0, 5))
        throw std::invalid_argument("Racing line vehicle_half_width_m and margin_m must lie within 0 and 5 m");
    if (options.max_iterations < 1 || options.max_iterations > 500)
        throw std::invalid_argument("Racing line max_iterations must lie within 1 and 500");
    if (!within(options.converged_shift_m, 1e-4, 1))
        throw std::invalid_argument("Racing line converged_shift_m must lie within 0.0001 and 1 m");
    const double keep = options.vehicle_half_width_m+options.margin_m;
    for (std::size_t i = 0; i < reference.points.size(); ++i) {
        const auto corridor = corridor_at(reference, project(reference, {reference.points[i].x_m, reference.points[i].y_m}));
        if (corridor.left_m+corridor.right_m < 2*keep)
            throw std::invalid_argument("The corridor near "+metres(reference.points[i].s_m)+" is narrower than the vehicle and margin, "+
                                        metres(2*keep));
    }
    const double limit = std::tan(config.max_steering_rad)/config.wheelbase_m;
    const double allowed = steering_share*limit;

    ConditionedTrack line = centreline;
    RacingLine result;
    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        const auto& points = line.track.points;
        const auto n = static_cast<OSQPInt>(points.size());
        const double step = line.track.length_m/static_cast<double>(points.size());
        const double inverse = 1/(step*step);
        Entries P{n, n}, A{2*n, n};
        std::vector<double> q(static_cast<std::size_t>(n), 0.0), l(static_cast<std::size_t>(2*n)), u(static_cast<std::size_t>(2*n));
        for (OSQPInt i = 0; i < n; ++i) {
            const auto k = static_cast<std::size_t>(i);
            const auto& p = points[k];
            // Shifts keep the vehicle and margin inside the corridor measured from the centreline.
            const auto projection = project(reference, {p.x_m, p.y_m});
            const auto corridor = corridor_at(reference, projection);
            l[k] = -(corridor.right_m-keep)-projection.signed_error_m;
            u[k] = (corridor.left_m-keep)-projection.signed_error_m;
            if (l[k] > u[k]) std::swap(l[k], u[k]);
            A.add(i, i, 1);
            // Curvature of the shifted line, linearised: kappa + (shift'' along the line) + kappa^2 shift.
            const std::array<OSQPInt, 3> columns{(i+n-1)%n, i, (i+1)%n};
            const std::array<double, 3> values{inverse, -2*inverse+p.curvature*p.curvature, inverse};
            for (std::size_t a = 0; a < 3; ++a) {
                A.add(n+i, columns[a], values[a]);
                q[static_cast<std::size_t>(columns[a])] += 2*values[a]*p.curvature;
                for (std::size_t b = 0; b < 3; ++b)
                    if (columns[a] <= columns[b]) P.add(columns[a], columns[b], 2*values[a]*values[b]);
            }
            // A vanishing weight on the shifts themselves makes the minimum unique where curvature does not care.
            P.add(i, i, 2e-12*inverse*inverse);
            l[static_cast<std::size_t>(n+i)] = -allowed-p.curvature;
            u[static_cast<std::size_t>(n+i)] = allowed-p.curvature;
        }
        const auto solution = solve(P, q, A, l, u);
        if (solution.status == OSQP_PRIMAL_INFEASIBLE || solution.status == OSQP_PRIMAL_INFEASIBLE_INACCURATE) {
            std::ostringstream message;
            message << "No line within the corridor meets the steering limit of " << limit << " 1/m (radius " << metres(1/limit) << ")";
            throw std::invalid_argument(message.str());
        }
        if (solution.x.empty())
            throw std::runtime_error("OSQP did not solve the racing line's quadratic program, status "+std::to_string(solution.status));
        std::vector<Vec2> moved(points.size());
        double largest = 0;
        for (std::size_t k = 0; k < points.size(); ++k) {
            const double shift = solution.x[k];
            largest = std::max(largest, std::abs(shift));
            moved[k] = {points[k].x_m+shift*line.left_normals[k].x, points[k].y_m+shift*line.left_normals[k].y};
        }
        ConditioningOptions refit;
        refit.input_spacing_m = std::clamp(step, 0.2, 5.0);
        refit.output_spacing_m = std::clamp(step, 0.2, 5.0);
        refit.smoothing_rms_m = refit_rms_m;
        refit.corridor_checks = false;
        line = condition_track(moved, reference.width_m, reference.name+" racing line", refit);
        result.iterations = iteration+1;
        result.last_shift_m = largest;
        if (largest < options.converged_shift_m) { result.converged = true; break; }
    }

    result.track = line.track;
    result.left_normals = line.left_normals;
    result.track.left_edge_m.reserve(result.track.points.size());
    result.track.right_edge_m.reserve(result.track.points.size());
    for (const auto& p : result.track.points) {
        const auto projection = project(reference, {p.x_m, p.y_m});
        const auto corridor = corridor_at(reference, projection);
        result.offset_m.push_back(projection.signed_error_m);
        result.track.left_edge_m.push_back(std::max(0.0, corridor.left_m-projection.signed_error_m));
        result.track.right_edge_m.push_back(std::max(0.0, corridor.right_m+projection.signed_error_m));
        if (std::abs(p.curvature) > limit)
            throw std::invalid_argument("The racing line's curvature near "+metres(p.s_m)+" exceeds the steering limit after fitting");
    }
    validate_track(result.track);
    result.squared_curvature = squared_curvature(result.track);
    result.centreline_squared_curvature = squared_curvature(reference);
    return result;
}

} // namespace fd
