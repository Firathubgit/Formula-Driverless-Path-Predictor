#include "fd/track_conditioning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace fd {
namespace {

constexpr double pi = std::numbers::pi;
// Knots are this many resampled points apart; a loop needs at least this many knot spans for the fit.
constexpr int points_per_knot = 2;
constexpr int minimum_knots = 8;

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    return value.substr(first, value.find_last_not_of(" \t\r\n")-first+1);
}

// A periodic uniform cubic B-spline in the parameter t in [0, length), with one control point per knot. Span j uses
// control points j to j+3, wrapping, so the curve closes with continuous position, tangent and curvature.
class ClosedSpline {
public:
    ClosedSpline(std::vector<Vec2> controls, double length)
        : controls_(std::move(controls)), length_(length), spacing_(length/static_cast<double>(controls_.size())) {}
    struct Point { Vec2 position, first, second; };
    Point at(double t) const {
        t = std::fmod(t, length_);
        if (t < 0) t += length_;
        const auto m = controls_.size();
        auto span = static_cast<std::size_t>(t/spacing_);
        if (span >= m) span = m-1;
        const double u = t/spacing_-static_cast<double>(span);
        const auto b = basis(u);
        Point result;
        for (std::size_t k = 0; k < 4; ++k) {
            const Vec2& c = controls_[(span+k)%m];
            result.position.x += b.value[k]*c.x;
            result.position.y += b.value[k]*c.y;
            result.first.x += b.first[k]*c.x/spacing_;
            result.first.y += b.first[k]*c.y/spacing_;
            result.second.x += b.second[k]*c.x/(spacing_*spacing_);
            result.second.y += b.second[k]*c.y/(spacing_*spacing_);
        }
        return result;
    }
    double length() const { return length_; }
    double spacing() const { return spacing_; }
    struct Basis { std::array<double, 4> value, first, second; };
    static Basis basis(double u) {
        const double v = 1-u;
        return {{v*v*v/6, (3*u*u*u-6*u*u+4)/6, (-3*u*u*u+3*u*u+3*u+1)/6, u*u*u/6},
                {-v*v/2, 1.5*u*u-2*u, -1.5*u*u+u+0.5, u*u/2},
                {v, 3*u-2, -3*u+1, u}};
    }
private:
    std::vector<Vec2> controls_;
    double length_, spacing_;
};

// A symmetric positive definite matrix stored by rows from each row's first nonzero column to its diagonal, factored
// in place by Cholesky, which keeps that envelope. The cyclic band of a closed spline has a narrow envelope in every
// row but the last few, so a factorisation costs about as much as the matrix has entries.
class EnvelopeMatrix {
public:
    explicit EnvelopeMatrix(std::vector<std::size_t> first) : first_(std::move(first)), rows_(first_.size()) {
        for (std::size_t i = 0; i < rows_.size(); ++i) rows_[i].assign(i-first_[i]+1, 0.0);
    }
    double& at(std::size_t i, std::size_t j) { return rows_[i][j-first_[i]]; }  // j <= i, j >= first(i)
    void factor() {
        for (std::size_t i = 0; i < rows_.size(); ++i)
            for (std::size_t j = first_[i]; j <= i; ++j) {
                double sum = at(i, j);
                for (std::size_t k = std::max(first_[i], first_[j]); k < j; ++k) sum -= at(i, k)*at(j, k);
                if (j < i) { at(i, j) = sum/at(j, j); continue; }
                if (!(sum > 0)) throw std::runtime_error("Track conditioning system is not positive definite");
                at(i, i) = std::sqrt(sum);
            }
    }
    std::vector<double> solve(std::vector<double> b) {
        const auto n = rows_.size();
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t k = first_[i]; k < i; ++k) b[i] -= at(i, k)*b[k];
            b[i] /= at(i, i);
        }
        for (std::size_t r = n; r > 0; --r) {
            const std::size_t i = r-1;
            b[i] /= at(i, i);
            for (std::size_t k = first_[i]; k < i; ++k) b[k] -= at(i, k)*b[i];
        }
        return b;
    }
private:
    std::vector<std::size_t> first_;
    std::vector<std::vector<double>> rows_;
};

// The closed polyline through the distinct points, resampled at equal arc length.
std::vector<Vec2> resample_linearly(const std::vector<Vec2>& points, double spacing, double& length) {
    std::vector<double> station{0.0};
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[(i+1)%points.size()];
        station.push_back(station.back()+std::hypot(b.x-a.x, b.y-a.y));
    }
    length = station.back();
    const auto count = static_cast<std::size_t>(std::max<long long>(std::llround(length/spacing), 1));
    std::vector<Vec2> result;
    result.reserve(count);
    std::size_t segment = 0;
    for (std::size_t k = 0; k < count; ++k) {
        const double s = length*static_cast<double>(k)/static_cast<double>(count);
        while (segment+1 < points.size() && station[segment+1] <= s) ++segment;
        const auto& a = points[segment];
        const auto& b = points[(segment+1)%points.size()];
        const double f = (s-station[segment])/(station[segment+1]-station[segment]);
        result.push_back({a.x+f*(b.x-a.x), a.y+f*(b.y-a.y)});
    }
    return result;
}

// The smoothest closed spline, in the sense of the summed squared second differences of its control points, whose
// squared distances from the resampled points at their parameters sum to at most the allowance. The distance grows
// with the penalty's weight, so the weight is found by bisection on its logarithm.
ClosedSpline fit(const std::vector<Vec2>& samples, double length, double allowance, double& residual) {
    const std::size_t n = samples.size();
    const std::size_t m = n/points_per_knot;
    const double knot = length/static_cast<double>(m), step = length/static_cast<double>(n);
    // Every nonzero of the normal equations and the penalty lies within three places of the diagonal, cyclically.
    std::vector<std::size_t> first(m);
    for (std::size_t i = 0; i < m; ++i) {
        first[i] = i;
        for (std::size_t d = 1; d <= 3; ++d) {
            first[i] = std::min(first[i], (i+d)%m);
            first[i] = std::min(first[i], (i+m-d)%m);
        }
    }
    struct Row { std::size_t span; ClosedSpline::Basis basis; };
    std::vector<Row> rows;
    rows.reserve(n);
    // The normal equations' matrix by row and cyclic offset -3 to 3, the only places it has entries (m >= 8, so none alias).
    std::vector<std::array<double, 7>> gram(m, std::array<double, 7>{});
    std::vector<double> rhs_x(m, 0.0), rhs_y(m, 0.0);
    const auto offset = [m](std::size_t i, std::size_t j) {
        const std::size_t ahead = (j+m-i)%m;
        return ahead <= 3 ? static_cast<std::size_t>(3+ahead) : static_cast<std::size_t>(3-(m-ahead));
    };
    for (std::size_t i = 0; i < n; ++i) {
        const double t = step*static_cast<double>(i);
        auto span = static_cast<std::size_t>(t/knot);
        if (span >= m) span = m-1;
        const auto b = ClosedSpline::basis(t/knot-static_cast<double>(span));
        rows.push_back({span, b});
        for (std::size_t a = 0; a < 4; ++a) {
            const auto ia = (span+a)%m;
            rhs_x[ia] += b.value[a]*samples[i].x;
            rhs_y[ia] += b.value[a]*samples[i].y;
            for (std::size_t c = 0; c < 4; ++c) gram[ia][offset(ia, (span+c)%m)] += b.value[a]*b.value[c];
        }
    }
    const auto controls_for = [&](double weight) {
        EnvelopeMatrix matrix(first);
        for (std::size_t i = 0; i < m; ++i)
            for (std::size_t j = first[i]; j <= i; ++j) {
                const std::size_t d = std::min((i+m-j)%m, (j+m-i)%m);
                if (d > 3) continue;
                const double penalty = d == 0 ? 6 : d == 1 ? -4 : d == 2 ? 1 : 0;
                matrix.at(i, j) = gram[i][offset(i, j)]+weight*penalty;
            }
        matrix.factor();
        const auto x = matrix.solve(rhs_x);
        const auto y = matrix.solve(rhs_y);
        std::vector<Vec2> controls(m);
        for (std::size_t i = 0; i < m; ++i) controls[i] = {x[i], y[i]};
        return controls;
    };
    const auto distance_for = [&](const std::vector<Vec2>& controls) {
        double sum = 0;
        for (std::size_t i = 0; i < n; ++i) {
            double x = 0, y = 0;
            for (std::size_t a = 0; a < 4; ++a) {
                x += rows[i].basis.value[a]*controls[(rows[i].span+a)%m].x;
                y += rows[i].basis.value[a]*controls[(rows[i].span+a)%m].y;
            }
            sum += (x-samples[i].x)*(x-samples[i].x)+(y-samples[i].y)*(y-samples[i].y);
        }
        return sum;
    };
    // The weight is relative to the points each control point is fitted to.
    const double scale = static_cast<double>(points_per_knot);
    double low = -8, high = 12;
    auto best = controls_for(scale*std::pow(10.0, low));
    residual = distance_for(best);
    if (residual <= allowance) {
        for (int halving = 0; halving < 48; ++halving) {
            const double mid = 0.5*(low+high);
            auto controls = controls_for(scale*std::pow(10.0, mid));
            const double distance = distance_for(controls);
            if (distance <= allowance) { low = mid; best = std::move(controls); residual = distance; }
            else high = mid;
        }
    }
    return ClosedSpline(std::move(best), length);
}

// Arc length along the spline from a to b by five-point Gauss-Legendre quadrature of its speed.
double arc(const ClosedSpline& spline, double a, double b) {
    static constexpr std::array<double, 5> nodes{0.0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640, 0.9061798459386640};
    static constexpr std::array<double, 5> weights{0.5688888888888889, 0.4786286704993665, 0.4786286704993665, 0.2369268850561891, 0.2369268850561891};
    const double half = 0.5*(b-a), middle = 0.5*(a+b);
    double sum = 0;
    for (std::size_t k = 0; k < 5; ++k) {
        const auto p = spline.at(middle+half*nodes[k]);
        sum += weights[k]*std::hypot(p.first.x, p.first.y);
    }
    return sum*half;
}

double cross(Vec2 a, Vec2 b) { return a.x*b.y-a.y*b.x; }
// Whether closed segments pq and rs share a point.
bool segments_meet(Vec2 p, Vec2 q, Vec2 r, Vec2 s) {
    const Vec2 d1{q.x-p.x, q.y-p.y}, d2{s.x-r.x, s.y-r.y}, w{r.x-p.x, r.y-p.y};
    const double denominator = cross(d1, d2);
    if (std::abs(denominator) < 1e-12*std::max(1.0, std::hypot(d1.x, d1.y)*std::hypot(d2.x, d2.y))) {
        if (std::abs(cross(w, d1)) > 1e-9*std::max(1.0, std::hypot(d1.x, d1.y))) return false;
        const double length2 = d1.x*d1.x+d1.y*d1.y;
        if (length2 == 0) return false;
        const double t0 = (w.x*d1.x+w.y*d1.y)/length2, t1 = t0+(d2.x*d1.x+d2.y*d1.y)/length2;
        return std::max(t0, t1) >= 0 && std::min(t0, t1) <= 1;
    }
    const double t = cross(w, d2)/denominator, u = cross(w, d1)/denominator;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

std::string metres(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(1);
    out << value << " m";
    return out.str();
}

} // namespace

ConditionedTrack condition_track(const std::vector<Vec2>& centreline, double width_m, std::string name, const ConditioningOptions& options) {
    const auto within = [](double value, double low, double high) { return std::isfinite(value) && value >= low && value <= high; };
    if (!within(width_m, 0.5, 50)) throw std::invalid_argument("Track width must lie within 0.5 and 50 m");
    if (!within(options.input_spacing_m, 0.2, 5)) throw std::invalid_argument("Conditioning input_spacing_m must lie within 0.2 and 5 m");
    if (!within(options.output_spacing_m, 0.2, 5)) throw std::invalid_argument("Conditioning output_spacing_m must lie within 0.2 and 5 m");
    if (!within(options.smoothing_rms_m, 0, 5)) throw std::invalid_argument("Conditioning smoothing_rms_m must lie within 0 and 5 m");
    std::vector<Vec2> points;
    for (const auto& p : centreline) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::invalid_argument("Centreline contains non-finite coordinates");
        if (points.empty() || std::hypot(p.x-points.back().x, p.y-points.back().y) > 1e-9) points.push_back(p);
    }
    while (points.size() > 1 && std::hypot(points.front().x-points.back().x, points.front().y-points.back().y) <= 1e-9) points.pop_back();
    if (points.size() < 4) throw std::invalid_argument("Centreline needs at least four distinct points");

    ConditionedTrack result;
    double length = 0;
    const auto samples = resample_linearly(points, options.input_spacing_m, length);
    if (samples.size() < static_cast<std::size_t>(points_per_knot*minimum_knots))
        throw std::invalid_argument("Centreline loop of "+metres(length)+" is too short to condition at "+metres(options.input_spacing_m)+" spacing");
    result.resampled_points = samples.size();
    double residual = 0;
    const auto spline = fit(samples, length, static_cast<double>(samples.size())*options.smoothing_rms_m*options.smoothing_rms_m, residual);
    result.residual_rms_m = std::sqrt(residual/static_cast<double>(samples.size()));

    // Cumulative arc length at quarter knots, then each sample by Newton's method on arc length within its quarter.
    const auto quarters = static_cast<std::size_t>(std::llround(spline.length()/spline.spacing()))*4;
    const double quarter = spline.length()/static_cast<double>(quarters);
    std::vector<double> station{0.0};
    for (std::size_t q = 0; q < quarters; ++q)
        station.push_back(station.back()+arc(spline, quarter*static_cast<double>(q), quarter*static_cast<double>(q+1)));
    const double total = station.back();
    const auto count = std::max<std::size_t>(8, static_cast<std::size_t>(std::llround(total/options.output_spacing_m)));
    const double spacing = total/static_cast<double>(count);
    Track& track = result.track;
    track.name = std::move(name);
    track.width_m = width_m;
    track.length_m = total;
    track.points.reserve(count);
    result.left_normals.reserve(count);
    for (std::size_t k = 0; k < count; ++k) {
        const double target = spacing*static_cast<double>(k);
        const auto q = std::min<std::size_t>(quarters-1, static_cast<std::size_t>(std::upper_bound(station.begin(), station.end(), target)-station.begin())-1);
        double lo = quarter*static_cast<double>(q), hi = lo+quarter, t = lo+quarter*(target-station[q])/(station[q+1]-station[q]);
        for (int iteration = 0; iteration < 50; ++iteration) {
            const double error = station[q]+arc(spline, quarter*static_cast<double>(q), t)-target;
            if (std::abs(error) < 1e-12*std::max(1.0, total)) break;
            if (error > 0) hi = t; else lo = t;
            const auto p = spline.at(t);
            double next = t-error/std::hypot(p.first.x, p.first.y);
            t = next > lo && next < hi ? next : 0.5*(lo+hi);
        }
        const auto p = spline.at(t);
        const double speed = std::hypot(p.first.x, p.first.y);
        const double curvature = cross(p.first, p.second)/(speed*speed*speed);
        track.points.push_back({p.position.x, p.position.y, target, curvature});
        result.left_normals.push_back({-p.first.y/speed, p.first.x/speed});
    }
    validate_track(track);

    const auto& pts = track.points;
    const auto at = [&](std::size_t i) { return Vec2{pts[i%count].x_m, pts[i%count].y_m}; };
    // The conditioned line must not cross itself.
    for (std::size_t i = 0; i < count; ++i)
        for (std::size_t j = i+2; j < count; ++j) {
            if (i == 0 && j == count-1) continue;
            if (segments_meet(at(i), at(i+1), at(j), at(j+1)))
                throw std::invalid_argument("Conditioned centreline crosses itself between "+metres(pts[i].s_m)+" and "+metres(pts[j].s_m));
        }
    if (!options.corridor_checks) {
        for (const auto& sample : samples) result.max_deviation_m = std::max(result.max_deviation_m, project(track, sample).distance_m);
        return result;
    }
    // Normals across the width must not cross within the distance over which a bend of half the width turns back.
    const double half = 0.5*width_m;
    const auto horizon = static_cast<std::size_t>(std::ceil(pi*half/spacing))+1;
    const auto normal_segment = [&](std::size_t i, Vec2& a, Vec2& b) {
        const Vec2 p = at(i), n = result.left_normals[i%count];
        a = {p.x-n.x*half, p.y-n.y*half};
        b = {p.x+n.x*half, p.y+n.y*half};
    };
    for (std::size_t i = 0; i < count; ++i) {
        Vec2 a, b;
        normal_segment(i, a, b);
        for (std::size_t k = 1; k <= horizon && k < count; ++k) {
            Vec2 c, d;
            normal_segment(i+k, c, d);
            if (segments_meet(a, b, c, d))
                throw std::invalid_argument("Normals across the track cross near "+metres(pts[i].s_m)+
                                            ": a bend there is tighter than half the "+metres(width_m)+
                                            " width; if the centreline is noisy, a larger smoothing allowance removes such kinks");
        }
    }
    // Parts of the track farther apart along it than that must stay at least a width apart.
    const auto separated = static_cast<std::size_t>(std::ceil(pi*half/spacing));
    for (std::size_t i = 0; i < count; ++i)
        for (std::size_t j = i+separated+1; j < count && j+separated < i+count; ++j)
            if (std::hypot(pts[i].x_m-pts[j].x_m, pts[i].y_m-pts[j].y_m) < width_m)
                throw std::invalid_argument("Track corridor overlaps itself: "+metres(pts[i].s_m)+" and "+metres(pts[j].s_m)+
                                            " are closer than the "+metres(width_m)+" width");

    for (const auto& sample : samples) result.max_deviation_m = std::max(result.max_deviation_m, project(track, sample).distance_m);
    return result;
}

ConditionedTrack condition_track(const Track& track, const ConditioningOptions& options) {
    validate_track(track);
    if (!(std::isfinite(options.input_spacing_m) && options.input_spacing_m >= 0.2 && options.input_spacing_m <= 5))
        throw std::invalid_argument("Conditioning input_spacing_m must lie within 0.2 and 5 m");
    std::vector<Vec2> points;
    for (double s = 0; s < track.length_m-options.input_spacing_m/2; s += options.input_spacing_m) {
        const auto p = sample(track, s);
        points.push_back({p.x_m, p.y_m});
    }
    return condition_track(points, track.width_m, track.name, options);
}

std::vector<Vec2> load_centreline(const std::filesystem::path& file) {
    std::ifstream input(file);
    if (!input) throw std::runtime_error("Cannot open centreline: "+file.string());
    std::vector<Vec2> points;
    std::string line;
    std::size_t number = 0;
    bool header_allowed = true;
    while (std::getline(input, line)) {
        ++number;
        if (const auto hash = line.find('#'); hash != std::string::npos) line.resize(hash);
        line = trim(line);
        if (line.empty()) continue;
        const auto comma = line.find(',');
        const auto where = "Centreline line "+std::to_string(number)+": ";
        if (comma == std::string::npos || line.find(',', comma+1) != std::string::npos)
            throw std::invalid_argument(where+"expected x_m,y_m");
        const auto field = [&](const std::string& text, double& value) {
            try {
                std::size_t end{};
                value = std::stod(text, &end);
                return end == text.size() && std::isfinite(value);
            } catch (const std::exception&) { return false; }
        };
        double x{}, y{};
        const bool numeric = field(trim(line.substr(0, comma)), x) && field(trim(line.substr(comma+1)), y);
        if (!numeric) {
            if (header_allowed) { header_allowed = false; continue; }
            throw std::invalid_argument(where+"expected two finite numbers in metres");
        }
        header_allowed = false;
        points.push_back({x, y});
    }
    return points;
}

} // namespace fd
