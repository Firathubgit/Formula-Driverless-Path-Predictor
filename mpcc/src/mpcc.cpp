#include "fd/mpcc.hpp"

#include "osqp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <tuple>

namespace fd {
namespace {

constexpr int NX = Mpcc::state_size, NU = Mpcc::input_size;
using StateVector = std::array<double, NX>;
using InputVector = std::array<double, NU>;
// The prediction model's state: the centre of gravity's pose and body-frame velocity, the reference station of the
// progress point, the acceleration and steering being applied, and the rate at which the progress point moves. Its
// inputs are the rates of the last three, so each is continuous and bounded like the plant's own steering rate.
enum : int { X = 0, Y, PHI, VX, VY, R, S, ACCEL, STEER, VS };
enum : int { JERK = 0, STEER_RATE, VS_RATE };
// Softened constraints per stage, one slack each.
enum : int { SLACK_CORRIDOR = 0, SLACK_FRONT_SLIP, SLACK_REAR_SLIP, SLACK_FRONT_FRICTION, SLACK_REAR_FRICTION, SLACK_SPEED, NS };
// Half the step, in metres of station, over which path headings and the lag error's rate are differenced.
constexpr double station_step_m = 0.25;
// Stations the progress point may move in one step, MPCC's trust region.
constexpr double station_trust_m = 30.0;
// A blockage constrains the centre of gravity from this far before it until the rear axle has passed it.
constexpr double blockage_lead_m = vehicle_half_width_m;

double wrap_near(double angle, double near) {
    return angle+2*std::numbers::pi*std::round((near-angle)/(2*std::numbers::pi));
}
double forward_distance(double from, double to, double length) {
    const double d = std::fmod(to-from, length);
    return d < 0 ? d+length : d;
}

// ---------------------------------------------------------------- the path being driven and its corridor

// The chosen action's path as offsets from the reference at continuous stations, the corridor the action leaves open,
// and the track's frame, for one decision.
class DrivenPath {
public:
    DrivenPath(const Track& track, const LocalOption& option, const std::vector<Obstruction>& obstructions,
               double keep_m, double margin_m)
        : track_(track), path_(option.path), obstructions_(obstructions), keep_m_(keep_m), margin_m_(margin_m) {}

    struct Frame { Vec2 point, tangent, normal; };
    Frame frame(double s) const {
        const auto a = sample(track_, s-station_step_m), b = sample(track_, s+station_step_m), c = sample(track_, s);
        const double tx = b.x_m-a.x_m, ty = b.y_m-a.y_m, norm = std::hypot(tx, ty);
        return {{c.x_m, c.y_m}, {tx/norm, ty/norm}, {-ty/norm, tx/norm}};
    }
    // The path's offset at a continuous station: its first offset before it, its last beyond it, interpolated between.
    double offset(double s) const {
        if (path_.empty()) return 0.0;
        const double at = path_.front().s_m+forward_distance(path_.front().s_m, s, track_.length_m);
        // Past the path's end: nearer its end it is beyond the path, nearer a lap on it is just behind its start.
        if (at >= path_.back().s_m)
            return at-path_.back().s_m <= path_.front().s_m+track_.length_m-at ? path_.back().offset_m : path_.front().offset_m;
        const auto upper = std::upper_bound(path_.begin(), path_.end(), at, [](double v, const StationOffset& p) { return v < p.s_m; });
        const auto& b = *upper;
        const auto& a = *(upper-1);
        return a.offset_m+(b.offset_m-a.offset_m)*(at-a.s_m)/(b.s_m-a.s_m);
    }
    Vec2 point(double s) const {
        const auto f = frame(s);
        const double d = offset(s);
        return {f.point.x+d*f.normal.x, f.point.y+d*f.normal.y};
    }
    double heading(double s) const {
        const auto a = point(s-station_step_m), b = point(s+station_step_m);
        return std::atan2(b.y-a.y, b.x-a.x);
    }
    // Contouring and lag error of a position against the path point at station s.
    std::array<double, 2> errors(double x, double y, double s) const {
        const auto p = point(s);
        const double th = heading(s), dx = p.x-x, dy = p.y-y;
        return {-std::sin(th)*dx+std::cos(th)*dy, std::cos(th)*dx+std::sin(th)*dy};
    }
    // Where the centre of gravity may lie across the reference at a projection: the corridor less the half width and
    // the margin, and beside a blockage only the side the action's path takes.
    std::array<double, 2> lateral_bounds(const Projection& at, double centre_to_rear_m) const {
        const auto corridor = corridor_at(track_, at);
        double lo = -corridor.right_m+keep_m_, hi = corridor.left_m-keep_m_;
        for (const auto& o : obstructions_) {
            const double span = forward_distance(o.from_s_m, o.to_s_m, track_.length_m);
            const double start = o.from_s_m-blockage_lead_m-margin_m_;
            if (forward_distance(start, at.s_m, track_.length_m) > span+2*blockage_lead_m+2*margin_m_+centre_to_rear_m) continue;
            const double taken = offset(at.s_m);
            if (taken >= (o.from_offset_m+o.to_offset_m)/2) lo = std::max(lo, o.to_offset_m+keep_m_);
            else hi = std::min(hi, o.from_offset_m-keep_m_);
        }
        return {lo, hi};
    }
private:
    const Track& track_;
    const std::vector<StationOffset>& path_;
    const std::vector<Obstruction>& obstructions_;
    double keep_m_{}, margin_m_{};
};

// ---------------------------------------------------------------- the prediction model

class Prediction {
public:
    Prediction(const SingleTrackModel& model, double stage_s, int substeps)
        : model_(model), stage_s_(stage_s), substeps_(substeps) {}
    StateVector derivative(const StateVector& x, const InputVector& u) const {
        const auto d = model_.derivative(motion(x), x[STEER], u[STEER_RATE], x[ACCEL]);
        return {d.x_m, d.y_m, d.yaw_rad, d.forward_mps, d.lateral_mps, d.yaw_rate_radps, x[VS], u[JERK], u[STEER_RATE], u[VS_RATE]};
    }
    // One stage with the input held, integrated with RK4; the car never reverses, as the plant does not.
    StateVector step(StateVector x, const InputVector& u) const {
        const double h = stage_s_/substeps_;
        const auto add = [](const StateVector& a, const StateVector& d, double t) {
            StateVector r;
            for (int i = 0; i < NX; ++i) r[i] = a[i]+t*d[i];
            return r;
        };
        for (int k = 0; k < substeps_; ++k) {
            const auto k1 = derivative(x, u), k2 = derivative(add(x, k1, h/2), u);
            const auto k3 = derivative(add(x, k2, h/2), u), k4 = derivative(add(x, k3, h), u);
            for (int i = 0; i < NX; ++i) x[i] += h/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
            x[VX] = std::max(0.0, x[VX]);
        }
        return x;
    }
    // The stage's linearisation: next = F + A dx + B du. Position and progress enter only themselves, and the progress
    // point's motion is exact, so only the six columns and two inputs the motion depends on are differenced.
    struct Linear { StateVector next; std::array<StateVector, NX> a; std::array<StateVector, NU> b; };
    Linear linearise(const StateVector& x, const InputVector& u) const {
        Linear lin{step(x, u), {}, {}};
        for (auto& column : lin.a) column.fill(0);
        for (auto& column : lin.b) column.fill(0);
        lin.a[X][X] = lin.a[Y][Y] = lin.a[S][S] = lin.a[VS][VS] = 1;
        lin.a[VS][S] = stage_s_;
        lin.b[VS_RATE][VS] = stage_s_;
        lin.b[VS_RATE][S] = stage_s_*stage_s_/2;
        for (const int j : {PHI, VX, VY, R, ACCEL, STEER}) {
            auto shifted = x;
            const double h = 1e-6*std::max(1.0, std::abs(x[j]));
            shifted[j] += h;
            const auto moved = step(shifted, u);
            for (int i = 0; i < NX; ++i) lin.a[j][i] = (moved[i]-lin.next[i])/h;
        }
        for (const int j : {JERK, STEER_RATE}) {
            auto shifted = u;
            const double h = 1e-6*std::max(1.0, std::abs(u[j]));
            shifted[j] += h;
            const auto moved = step(x, shifted);
            for (int i = 0; i < NX; ++i) lin.b[j][i] = (moved[i]-lin.next[i])/h;
        }
        return lin;
    }
    SingleTrackMotion motion(const StateVector& x) const { return {x[X], x[Y], x[PHI], x[VX], x[VY], x[R]}; }
    const SingleTrackModel& model() const { return model_; }
private:
    const SingleTrackModel& model_;
    double stage_s_{};
    int substeps_{};
};

// ---------------------------------------------------------------- the quadratic programme

// Minimise z'Pz/2 + q'z subject to l <= Az <= u, gathered entry by entry.
struct QuadraticProgramme {
    explicit QuadraticProgramme(int variables) : n(variables), q(static_cast<std::size_t>(variables), 0.0) {}
    int n{};
    std::vector<std::tuple<OSQPInt, OSQPInt, OSQPFloat>> p, a;
    std::vector<double> q, l, u;
    void hessian(int i, int j, double v) {
        if (v == 0) return;
        if (i > j) std::swap(i, j);
        p.emplace_back(i, j, v);
    }
    int row(double low, double high) {
        l.push_back(low);
        u.push_back(high);
        return static_cast<int>(l.size())-1;
    }
    void entry(int r, int c, double v) { if (v != 0) a.emplace_back(r, c, v); }
};

// Compressed columns from entries, summing repeats; rows sorted within each column.
class Compressed {
public:
    Compressed(std::vector<std::tuple<OSQPInt, OSQPInt, OSQPFloat>> entries, OSQPInt rows, OSQPInt cols)
        : column_start_(static_cast<std::size_t>(cols+1), 0) {
        std::sort(entries.begin(), entries.end(), [](const auto& x, const auto& y) {
            return std::get<1>(x) != std::get<1>(y) ? std::get<1>(x) < std::get<1>(y) : std::get<0>(x) < std::get<0>(y);
        });
        for (const auto& [r, c, v] : entries) {
            if (!row_.empty() && last_col_ == c && row_.back() == r) { value_.back() += v; continue; }
            row_.push_back(r);
            value_.push_back(v);
            last_col_ = c;
            ++column_start_[static_cast<std::size_t>(c+1)];
        }
        for (std::size_t c = 0; c+1 < column_start_.size(); ++c) column_start_[c+1] += column_start_[c];
        matrix_.m = rows;
        matrix_.n = cols;
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
    OSQPInt last_col_{-1};
    OSQPCscMatrix matrix_{};
};

struct Solution { ControllerStatus status{ControllerStatus::not_solved}; std::vector<double> z, y; };
// Solves from the multipliers of a previous programme of the same shape when given them: consecutive programmes differ
// little, and their active constraints hardly at all.
Solution solve(const QuadraticProgramme& qp, const std::vector<double>& duals, double tolerance) {
    const auto rows = static_cast<OSQPInt>(qp.l.size());
    const Compressed hessian(qp.p, qp.n, qp.n), constraints(qp.a, rows, qp.n);
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polishing = 0;
    settings.eps_abs = tolerance;
    settings.eps_rel = tolerance;
    settings.max_iter = 4000;
    OSQPSolver* raw = nullptr;
    const OSQPInt setup = osqp_setup(&raw, hessian.get(), qp.q.data(), constraints.get(), qp.l.data(), qp.u.data(), rows, qp.n, &settings);
    const std::unique_ptr<OSQPSolver, OSQPInt (*)(OSQPSolver*)> solver(raw, &osqp_cleanup);
    if (setup != 0) return {};
    if (duals.size() == qp.l.size()) osqp_warm_start(solver.get(), nullptr, duals.data());
    osqp_solve(solver.get());
    Solution solution;
    const auto status = solver->info->status_val;
    if (status == OSQP_SOLVED) solution.status = ControllerStatus::solved;
    else if (status == OSQP_SOLVED_INACCURATE) solution.status = ControllerStatus::solved_inaccurate;
    else return solution;
    solution.z.assign(solver->solution->x, solver->solution->x+qp.n);
    solution.y.assign(solver->solution->y, solver->solution->y+rows);
    return solution;
}

} // namespace

// ---------------------------------------------------------------- Mpcc

Mpcc::Mpcc(MpccOptions options) : options_(options) {
    const auto within = [](double v, double low, double high) { return std::isfinite(v) && v >= low && v <= high; };
    const auto& o = options_;
    if (o.stages < 5 || o.stages > 400 || !within(o.stage_s, 0.005, 0.5) ||
        std::abs(o.stages*o.stage_s-prediction_horizon_s) > 1e-9)
        throw std::invalid_argument("MPCC's horizon must be 5 to 400 stages of 0.005 to 0.5 s spanning the prediction horizon");
    if (o.substeps < 1 || o.substeps > 20 || o.iterations < 1 || o.iterations > 20 || !within(o.step_share, 0.05, 1) ||
        !within(o.step_weight, 0, 1e4) || !within(o.solver_tolerance, 1e-8, 0.1) ||
        o.restart_after_failures < 1 || o.restart_after_failures > 100)
        throw std::invalid_argument("MPCC's substeps, iterations, step share and restart count are outside their ranges");
    for (const double w : {o.contouring_weight, o.lag_weight, o.progress_weight, o.terminal_contouring_multiplier, o.heading_weight,
                           o.sideslip_weight, o.yaw_rate_weight, o.acceleration_weight, o.steering_weight, o.progress_rate_weight,
                           o.jerk_weight, o.steering_rate_weight, o.progress_acceleration_weight, o.slack_quadratic_weight,
                           o.slack_linear_weight})
        if (!within(w, 0, 1e7)) throw std::invalid_argument("MPCC's weights must be finite and nonnegative");
    if (!within(o.slip_angle_share, 0.1, 1) || !within(o.friction_share, 0.1, 1) || !within(o.corridor_margin_m, 0, 3) ||
        !within(o.max_jerk_mps3, 1, 1000) || !within(o.max_progress_acceleration_mps2, 1, 1000))
        throw std::invalid_argument("MPCC's constraint shares, margin and rate limits are outside their ranges");
}

void Mpcc::reset() {
    duals_.clear();
    states_.clear();
    inputs_.clear();
    have_plan_ = false;
    failures_ = 0;
}

ControllerSettings Mpcc::settings() const {
    const auto& o = options_;
    return {ControllerMode::mpcc,
            {{"stages", o.stages}, {"stage_s", o.stage_s}, {"substeps", o.substeps}, {"iterations", o.iterations},
             {"step_share", o.step_share}, {"step_weight", o.step_weight}, {"solver_tolerance", o.solver_tolerance},
             {"restart_after_failures", o.restart_after_failures},
             {"contouring_weight", o.contouring_weight}, {"lag_weight", o.lag_weight}, {"progress_weight", o.progress_weight},
             {"terminal_contouring_multiplier", o.terminal_contouring_multiplier}, {"heading_weight", o.heading_weight},
             {"sideslip_weight", o.sideslip_weight}, {"yaw_rate_weight", o.yaw_rate_weight},
             {"acceleration_weight", o.acceleration_weight}, {"steering_weight", o.steering_weight},
             {"progress_rate_weight", o.progress_rate_weight}, {"jerk_weight", o.jerk_weight},
             {"steering_rate_weight", o.steering_rate_weight}, {"progress_acceleration_weight", o.progress_acceleration_weight},
             {"slack_quadratic_weight", o.slack_quadratic_weight}, {"slack_linear_weight", o.slack_linear_weight},
             {"slip_angle_share", o.slip_angle_share}, {"friction_share", o.friction_share},
             {"corridor_margin_m", o.corridor_margin_m}, {"max_jerk_mps3", o.max_jerk_mps3},
             {"max_progress_acceleration_mps2", o.max_progress_acceleration_mps2}}};
}

void Mpcc::check(const VehicleModel& model, const Config& config) const {
    const SingleTrackModel reduced(reduced_single_track(model), config);
    const double ticks = options_.stage_s/config.fixed_dt_s;
    if (std::abs(ticks-std::round(ticks)) > 1e-9) throw std::invalid_argument("MPCC's stage must be a whole number of plant ticks");
}

ControllerPlan Mpcc::plan(const ControlRequest& request) {
    const auto began = std::chrono::steady_clock::now();
    const auto& config = request.config;
    const auto& track = request.track;
    const auto& o = options_;
    if (request.decision.holding) throw std::invalid_argument("MPCC does not drive a hold for a blockage; the policy does");
    check(request.model, config);
    const int N = o.stages;
    const double Ts = o.stage_s;
    const SingleTrackModel model(reduced_single_track(request.model), config);
    const Prediction prediction(model, Ts, o.substeps);
    const auto& option = request.decision.choice();
    const double keep = vehicle_half_width_m+o.corridor_margin_m;
    const DrivenPath path(track, option, request.obstructions, keep, o.corridor_margin_m);
    const double length = track.length_m;
    const double now = request.state.pose.time_s;
    const double control_ticks = static_cast<double>(std::llround(config.control_dt_s/config.fixed_dt_s));
    const double hold_s = (request.ticks_to_next_control ? static_cast<double>(request.ticks_to_next_control) : control_ticks)*config.fixed_dt_s;
    const InputVector input_limit{o.max_jerk_mps3, config.max_steering_rate_radps, o.max_progress_acceleration_mps2};
    const double lr = model.cg_to_rear_m();

    // The state now, its progress point where the centre of gravity projects onto the reference.
    const auto motion = model.motion(request.state);
    StateVector initial{motion.x_m, motion.y_m, motion.yaw_rad, motion.forward_mps, motion.lateral_mps, motion.yaw_rate_radps,
                        project(track, {motion.x_m, motion.y_m}).s_m, request.held.acceleration_mps2,
                        request.state.pose.steering_rad, motion.forward_mps};
    const auto unwrap_station = [&](double s, double near) { return s+length*std::round((near-s)/length); };

    // Where to start: the last plan moved on by the time since, or the policy's prediction of the chosen action.
    std::vector<StateVector> x(static_cast<std::size_t>(N+1));
    std::vector<InputVector> u(static_cast<std::size_t>(N));
    bool restarted = !have_plan_;
    if (have_plan_) {
        const double elapsed = now-planned_at_s_;
        for (int k = 0; k <= N; ++k) {
            const double t = (elapsed+k*Ts)/Ts;
            const int j = static_cast<int>(std::floor(t));
            if (j+1 <= N) {
                const double f = t-j;
                for (int i = 0; i < NX; ++i) x[k][i] = states_[j][i]+f*(states_[j+1][i]-states_[j][i]);
                if (k < N) u[k] = inputs_[std::min(j, N-1)];
            } else {
                x[k] = prediction.step(x[k-1], inputs_[N-1]);
                if (k < N) u[k] = inputs_[N-1];
            }
        }
        initial[S] = unwrap_station(initial[S], x[0][S]);
        initial[VS] = x[0][VS];
        initial[PHI] = wrap_near(initial[PHI], x[0][PHI]);
    } else {
        // The policy's prediction holds a point at every control tick; read it at each stage's time, interpolated.
        const auto& points = request.decision.trajectory().points;
        const auto at = [&](int k) {
            const double t = points.front().state.time_s+k*Ts;
            const auto later = std::find_if(points.begin(), points.end(), [&](const TrajectorySample& p) { return p.state.time_s > t; });
            if (later == points.begin()) return points.front();
            if (later == points.end()) return points.back();
            const auto& a = *(later-1);
            const auto& b = *later;
            const double f = (t-a.state.time_s)/(b.state.time_s-a.state.time_s);
            TrajectorySample p = a;
            p.state.x_m += f*(b.state.x_m-a.state.x_m);
            p.state.y_m += f*(b.state.y_m-a.state.y_m);
            p.state.yaw_rad += f*wrap_angle(b.state.yaw_rad-a.state.yaw_rad);
            p.state.speed_mps += f*(b.state.speed_mps-a.state.speed_mps);
            p.state.steering_rad += f*(b.state.steering_rad-a.state.steering_rad);
            p.state.time_s = t;
            return p;
        };
        double previous_s = initial[S];
        for (int k = 0; k <= N; ++k) {
            const auto p = at(k);
            const auto& st = p.state;
            const double rate = st.speed_mps*std::tan(st.steering_rad)/config.wheelbase_m;
            StateVector g{st.x_m+lr*std::cos(st.yaw_rad), st.y_m+lr*std::sin(st.yaw_rad), st.yaw_rad, st.speed_mps, lr*rate, rate,
                          0, p.acceleration_mps2, st.steering_rad, st.speed_mps};
            g[S] = unwrap_station(project(track, {g[X], g[Y]}).s_m, previous_s);
            if (k > 0) g[PHI] = wrap_near(g[PHI], x[k-1][PHI]);
            previous_s = g[S];
            x[k] = g;
        }
        for (int k = 0; k <= N; ++k) {
            const int a = std::max(0, k-1), b = std::min(N, k+1);
            x[k][VS] = std::max(0.0, (x[b][S]-x[a][S])/((b-a)*Ts));
        }
        for (int k = 0; k < N; ++k)
            for (int j = 0; j < NU; ++j) {
                const int i = j == JERK ? ACCEL : j == STEER_RATE ? STEER : VS;
                u[k][j] = std::clamp((x[k+1][i]-x[k][i])/Ts, -input_limit[j], input_limit[j]);
            }
        initial[PHI] = wrap_near(initial[PHI], x[0][PHI]);
    }
    x[0] = initial;

    // Bounds. The acceleration always admits the state the car is in. The top speed is the configuration's; a car
    // already above it, as after the policy drove or a grip change, is brought down to it from its own speed at half a g
    // rather than the bound moving up with it. It is softened like the other constraints: acceleration changes at a
    // bounded rate, so a car still accelerating above the top speed cannot shed the excess within one stage, and as a
    // hard bound that made every programme infeasible for as long as the car stayed above it.
    const auto top = [&](int k) { return std::max(config.max_speed_mps, initial[VX]-0.5*gravity_mps2*k*Ts); };
    const double grip = config.grip_mu*gravity_mps2;
    const double accel_low = std::min(-1.2*grip, initial[ACCEL]), accel_high = std::max(1.2*grip, initial[ACCEL]);

    // Variables: every stage's state change, every input change, then each constrained stage's slacks.
    const int nx_all = NX*(N+1), nu_all = NU*N;
    const auto xi = [&](int k, int i) { return NX*k+i; };
    const auto ui = [&](int k, int j) { return nx_all+NU*k+j; };
    const auto si = [&](int k, int m) { return nx_all+nu_all+NS*(k-1)+m; };
    const int variables = nx_all+nu_all+NS*N;

    ControllerStatus best = ControllerStatus::not_solved;
    int solved_steps = 0;
    for (int iteration = 0; iteration < o.iterations; ++iteration) {
        QuadraticProgramme qp(variables);
        // The initial state is fixed.
        for (int i = 0; i < NX; ++i) {
            const int r = qp.row(initial[i]-x[0][i], initial[i]-x[0][i]);
            qp.entry(r, xi(0, i), 1);
        }
        for (int k = 0; k < N; ++k) {
            // Dynamics: dx[k+1] - A dx[k] - B du[k] = F(x[k], u[k]) - x[k+1].
            const auto lin = prediction.linearise(x[k], u[k]);
            for (int i = 0; i < NX; ++i) {
                const double defect = lin.next[i]-x[k+1][i];
                const int r = qp.row(defect, defect);
                qp.entry(r, xi(k+1, i), 1);
                for (int j = 0; j < NX; ++j) qp.entry(r, xi(k, j), -lin.a[j][i]);
                for (int j = 0; j < NU; ++j) qp.entry(r, ui(k, j), -lin.b[j][i]);
            }
            // Input bounds and rate costs.
            const std::array<double, NU> weight{o.jerk_weight, o.steering_rate_weight, o.progress_acceleration_weight};
            for (int j = 0; j < NU; ++j) {
                const int r = qp.row(-input_limit[j]-u[k][j], input_limit[j]-u[k][j]);
                qp.entry(r, ui(k, j), 1);
                qp.hessian(ui(k, j), ui(k, j), 2*weight[j]);
                qp.q[ui(k, j)] += 2*weight[j]*u[k][j];
            }
        }
        for (int k = 1; k <= N; ++k) {
            const auto& g = x[k];
            // State bounds.
            const std::array<std::tuple<int, double, double>, 4> boxes{{{VX, 0.0, OSQP_INFTY}, {ACCEL, accel_low, accel_high},
                {STEER, -config.max_steering_rad, config.max_steering_rad}, {VS, 0.0, 1.5*top(k)}}};
            for (const auto& [i, low, high] : boxes) {
                const int r = qp.row(low-g[i], high-g[i]);
                qp.entry(r, xi(k, i), 1);
            }
            {
                const int r = qp.row(-station_trust_m, station_trust_m);
                qp.entry(r, xi(k, S), 1);
            }
            // Contouring and lag, their gradient in position analytic and in station differenced.
            const auto e = path.errors(g[X], g[Y], g[S]);
            const auto ahead = path.errors(g[X], g[Y], g[S]+station_step_m), behind = path.errors(g[X], g[Y], g[S]-station_step_m);
            const double th = path.heading(g[S]);
            const std::array<std::array<double, 3>, 2> grad{{{std::sin(th), -std::cos(th), (ahead[0]-behind[0])/(2*station_step_m)},
                                                             {-std::cos(th), -std::sin(th), (ahead[1]-behind[1])/(2*station_step_m)}}};
            const std::array<double, 2> w{o.contouring_weight*(k == N ? o.terminal_contouring_multiplier : 1.0), o.lag_weight};
            const std::array<int, 3> idx{xi(k, X), xi(k, Y), xi(k, S)};
            for (int m = 0; m < 2; ++m)
                for (int a = 0; a < 3; ++a) {
                    qp.q[idx[a]] += 2*w[m]*e[m]*grad[m][a];
                    for (int b = a; b < 3; ++b) qp.hessian(idx[a], idx[b], 2*w[m]*grad[m][a]*grad[m][b]);
                }
            // Progress.
            qp.q[xi(k, VS)] -= o.progress_weight;
            // Heading along the path.
            const double heading_error = g[PHI]-wrap_near(th, g[PHI]);
            qp.hessian(xi(k, PHI), xi(k, PHI), 2*o.heading_weight);
            qp.q[xi(k, PHI)] += 2*o.heading_weight*heading_error;
            // Sideslip beyond the kinematic bicycle's.
            {
                const double vx = std::max(g[VX], 1.0), vy = g[VY], share = lr/config.wheelbase_m;
                const double t = std::tan(g[STEER]);
                const double f = std::atan(share*t)-std::atan2(vy, vx);
                const double d_vx = g[VX] > 1.0 ? vy/(vx*vx+vy*vy) : 0.0, d_vy = -vx/(vx*vx+vy*vy);
                const double sec = 1/std::cos(g[STEER]);
                const double d_steer = share*sec*sec/(1+share*share*t*t);
                const std::array<std::pair<int, double>, 3> gr{{{xi(k, VX), d_vx}, {xi(k, VY), d_vy}, {xi(k, STEER), d_steer}}};
                for (std::size_t a = 0; a < 3; ++a) {
                    qp.q[gr[a].first] += 2*o.sideslip_weight*f*gr[a].second;
                    for (std::size_t b = a; b < 3; ++b) qp.hessian(gr[a].first, gr[b].first, 2*o.sideslip_weight*gr[a].second*gr[b].second);
                }
            }
            // Values.
            const std::array<std::pair<int, double>, 4> values{{{R, o.yaw_rate_weight}, {ACCEL, o.acceleration_weight},
                                                                {STEER, o.steering_weight}, {VS, o.progress_rate_weight}}};
            for (const auto& [i, weight] : values) {
                qp.hessian(xi(k, i), xi(k, i), 2*weight);
                qp.q[xi(k, i)] += 2*weight*g[i];
            }
            // Slack costs.
            for (int m = 0; m < NS; ++m) {
                qp.hessian(si(k, m), si(k, m), 2*o.slack_quadratic_weight);
                qp.q[si(k, m)] += o.slack_linear_weight;
                const int r = qp.row(0, OSQP_INFTY);
                qp.entry(r, si(k, m), 1);
            }
            {
                const int r = qp.row(-OSQP_INFTY, top(k)-g[VX]);
                qp.entry(r, xi(k, VX), 1);
                qp.entry(r, si(k, SLACK_SPEED), -1);
            }
            // Corridor, across the reference at the centre of gravity's projection.
            {
                const auto at = project(track, {g[X], g[Y]});
                const auto bounds = path.lateral_bounds(at, lr);
                const auto f = path.frame(at.s_m);
                const double lateral = (g[X]-f.point.x)*f.normal.x+(g[Y]-f.point.y)*f.normal.y;
                const int low = qp.row(bounds[0]-lateral, OSQP_INFTY), high = qp.row(-OSQP_INFTY, bounds[1]-lateral);
                for (const int r : {low, high}) {
                    qp.entry(r, xi(k, X), f.normal.x);
                    qp.entry(r, xi(k, Y), f.normal.y);
                }
                qp.entry(low, si(k, SLACK_CORRIDOR), 1);
                qp.entry(high, si(k, SLACK_CORRIDOR), -1);
            }
            // Each axle's slip angle and friction ellipse, linearised by differences.
            {
                const auto axles = [&](const StateVector& s) { return model.axle_demand(prediction.motion(s), s[STEER], s[ACCEL]); };
                const auto base = axles(g);
                const std::array<int, 5> cols{VX, VY, R, STEER, ACCEL};
                std::array<std::array<double, 5>, 4> d{};
                for (std::size_t c = 0; c < cols.size(); ++c) {
                    auto shifted = g;
                    const double h = 1e-6*std::max(1.0, std::abs(g[cols[c]]));
                    shifted[cols[c]] += h;
                    const auto moved = axles(shifted);
                    d[0][c] = (moved.front_slip_angle_rad-base.front_slip_angle_rad)/h;
                    d[1][c] = (moved.rear_slip_angle_rad-base.rear_slip_angle_rad)/h;
                    d[2][c] = (moved.front_friction_use*moved.front_friction_use-base.front_friction_use*base.front_friction_use)/h;
                    d[3][c] = (moved.rear_friction_use*moved.rear_friction_use-base.rear_friction_use*base.rear_friction_use)/h;
                }
                const double front_limit = o.slip_angle_share*base.front_peak_slip_angle_rad;
                const double rear_limit = o.slip_angle_share*base.rear_peak_slip_angle_rad;
                const std::array<std::tuple<int, double, double, double>, 4> rows{{
                    {SLACK_FRONT_SLIP, base.front_slip_angle_rad, -front_limit, front_limit},
                    {SLACK_REAR_SLIP, base.rear_slip_angle_rad, -rear_limit, rear_limit},
                    {SLACK_FRONT_FRICTION, base.front_friction_use*base.front_friction_use, -OSQP_INFTY, o.friction_share*o.friction_share},
                    {SLACK_REAR_FRICTION, base.rear_friction_use*base.rear_friction_use, -OSQP_INFTY, o.friction_share*o.friction_share}}};
                for (std::size_t m = 0; m < rows.size(); ++m) {
                    const auto& [slack, value, low, high] = rows[m];
                    if (low > -OSQP_INFTY) {
                        const int r = qp.row(low-value, OSQP_INFTY);
                        for (std::size_t c = 0; c < cols.size(); ++c) qp.entry(r, xi(k, cols[c]), d[m][c]);
                        qp.entry(r, si(k, slack), 1);
                    }
                    const int r = qp.row(-OSQP_INFTY, high-value);
                    for (std::size_t c = 0; c < cols.size(); ++c) qp.entry(r, xi(k, cols[c]), d[m][c]);
                    qp.entry(r, si(k, slack), -1);
                }
            }
        }
        // Each step is penalised by its size, a proximal term: it keeps every direction strictly convex, which the
        // first-order solver needs to converge in few iterations on a problem this close to a linear programme, and it
        // damps each step, which the next decision continues from.
        {
            for (int i = 0; i < variables; ++i) qp.hessian(i, i, i < nx_all+nu_all ? o.step_weight : 1e-6);
        }
        const auto solution = solve(qp, duals_, o.solver_tolerance);
        if (solution.status == ControllerStatus::not_solved) { duals_.clear(); continue; }
        duals_ = solution.y;
        ++solved_steps;
        if (best == ControllerStatus::not_solved || solution.status == ControllerStatus::solved) best = solution.status;
        for (int k = 0; k <= N; ++k)
            for (int i = 0; i < NX; ++i) x[k][i] += o.step_share*solution.z[static_cast<std::size_t>(xi(k, i))];
        for (int k = 0; k < N; ++k)
            for (int j = 0; j < NU; ++j) u[k][j] += o.step_share*solution.z[static_cast<std::size_t>(ui(k, j))];
    }

    // The plan is the prediction model driven by the planned inputs from the actual state, so it is consistent with
    // itself; it is also where the next decision starts.
    std::vector<StateVector> planned(static_cast<std::size_t>(N+1));
    planned[0] = initial;
    for (int k = 0; k < N; ++k) {
        for (int j = 0; j < NU; ++j) u[k][j] = std::clamp(u[k][j], -input_limit[j], input_limit[j]);
        planned[k+1] = prediction.step(planned[k], u[k]);
        planned[k+1][STEER] = std::clamp(planned[k+1][STEER], -config.max_steering_rad, config.max_steering_rad);
    }
    if (solved_steps > 0) failures_ = 0;
    else if (++failures_ >= o.restart_after_failures) { have_plan_ = false; failures_ = 0; }
    if (solved_steps > 0 || have_plan_) {
        states_ = planned;
        inputs_ = u;
        planned_at_s_ = now;
        have_plan_ = true;
    }

    ControllerPlan result;
    result.status = best;
    result.iterations = solved_steps;
    result.restarted = restarted;
    auto& trajectory = result.trajectory;
    trajectory.within_model_envelope = true;
    trajectory.validity_reason = "MPCC plan within the prediction model's slip angles and the corridor";
    double distance = 0;
    for (int k = 0; k <= N; ++k) {
        const auto& g = planned[k];
        const auto rear = model.plant_state(prediction.motion(g), g[STEER], now+k*Ts);
        TrajectorySample sample;
        sample.state = rear.pose;
        sample.state.time_s = now+k*Ts;
        if (k > 0) {
            const auto& before = trajectory.points.back().state;
            distance += std::hypot(sample.state.x_m-before.x_m, sample.state.y_m-before.y_m);
            trajectory.points.back().acceleration_mps2 = (sample.state.speed_mps-before.speed_mps)/Ts;
        }
        sample.distance_m = distance;
        trajectory.points.push_back(sample);
        trajectory.commands.push_back({g[ACCEL], g[STEER]});
        const auto axles = model.axle_demand(prediction.motion(g), g[STEER], g[ACCEL]);
        if (trajectory.within_model_envelope && k > 0 && g[VX] > model.car().kinematic_below_mps &&
            (std::abs(axles.front_slip_angle_rad) > axles.front_peak_slip_angle_rad ||
             std::abs(axles.rear_slip_angle_rad) > axles.rear_peak_slip_angle_rad)) {
            trajectory.within_model_envelope = false;
            trajectory.validity_reason = "MPCC plan passes a tire's peak slip angle";
        }
        if (trajectory.within_model_envelope && k > 0 &&
            !within_corridor(track, project(track, {sample.state.x_m, sample.state.y_m}), vehicle_half_width_m)) {
            trajectory.within_model_envelope = false;
            trajectory.validity_reason = "MPCC plan leaves the corridor";
        }
    }
    // The command for the interval until the next decision: where the plan puts acceleration and steering by its end.
    const Command command = planned_command(point_times(trajectory), trajectory.commands, now+hold_s);
    auto& control = trajectory.first_control;
    control.requested = command;
    control.applied = command;
    control.geometric_steering_rad = command.steering_rad;
    control.target_speed_mps = planned[1][VX];
    control.reference = project(track, {request.state.pose.x_m, request.state.pose.y_m});
    control.track_distance_m = control.reference.distance_m;
    result.solve_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    return result;
}

} // namespace fd
