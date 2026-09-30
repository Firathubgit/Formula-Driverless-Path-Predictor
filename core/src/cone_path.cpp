// A port of FaSTTUBe's ft-fsd-path-planning (MIT License, Copyright (c) 2022 Panagiotis): its cone sorting
// (sorting_cones/), cone matching (cone_matching/) and path calculation (calculate_path/) for a trackdrive, with the
// helpers of utils/math_utils.py they use. Each function keeps the name and order of operations of the Python it ports,
// including NumPy's indexing where a -1 index reads the last element, so that its results can be compared with the
// published package's on the same cones (tests/fixtures/cone_path, decision 0031). See THIRD_PARTY_NOTICES.md.
#include "fd/cone_path.hpp"
#include "fitpack.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <tuple>

namespace fd {
namespace {

constexpr double pi = std::numbers::pi;
constexpr double inf = std::numeric_limits<double>::infinity();
constexpr int left_type = static_cast<int>(ConeType::blue), right_type = static_cast<int>(ConeType::yellow);

// ------------------------------------------------------------------ utils/math_utils.py

struct V { double x{}, y{}; };
V operator-(V a, V b) { return {a.x-b.x, a.y-b.y}; }
V operator+(V a, V b) { return {a.x+b.x, a.y+b.y}; }
V operator*(V a, double s) { return {a.x*s, a.y*s}; }
double norm(V v) { return std::sqrt(v.x*v.x+v.y*v.y); }
double angle_of(V v) { return std::atan2(v.y, v.x); }
double sign(double v) { return v > 0 ? 1.0 : v < 0 ? -1.0 : 0.0; }

// rotate(): points turned counterclockwise by theta.
V rotate(V p, double theta) {
    const double c = std::cos(theta), s = std::sin(theta);
    return {p.x*c-p.y*s, p.x*s+p.y*c};
}

// vec_angle_between(): the angle between two vectors, with the cosine clipped to [-1, 1]; NaN where either is zero,
// as NumPy's 0/0 is.
double vec_angle_between(V a, V b) {
    double c = (a.x*b.x+a.y*b.y)/(norm(a)*norm(b));
    if (c < -1) c = -1;
    if (c > 1) c = 1;
    return std::acos(c);
}

// angle_difference(): (angle1 - angle2 + 3 pi) % 2 pi - pi, with Python's floor modulo.
double angle_difference(double a1, double a2) {
    const double x = a1-a2+3*pi, period = 2*pi;
    return x-period*std::floor(x/period)-pi;
}

bool inside_ellipse(V point, V centre, V major_direction, double major_radius, double minor_radius) {
    const V r = rotate(point-centre, -std::atan2(major_direction.y, major_direction.x));
    return r.x*r.x/(major_radius*major_radius)+r.y*r.y/(minor_radius*minor_radius) < 1;
}

double squared_distance(V a, V b) {
    const double dx = a.x-b.x, dy = a.y-b.y;
    return dx*dx+dy*dy;
}

// circle_fit(): the hyper fit of a circle to points, {centre x, centre y, radius}. Where FaSTTUBe divides by zero, on
// points in a perfect line, the radius is infinite.
std::array<double, 3> circle_fit(const std::vector<V>& coords, int max_iter = 99) {
    const double n = static_cast<double>(coords.size());
    double mx = 0, my = 0;
    for (const auto& p : coords) { mx += p.x; my += p.y; }
    mx /= n;
    my /= n;
    double Mxy = 0, Mxx = 0, Myy = 0, Mxz = 0, Myz = 0, Mzz = 0;
    for (const auto& p : coords) {
        const double Xi = p.x-mx, Yi = p.y-my, Zi = Xi*Xi+Yi*Yi;
        Mxy += Xi*Yi; Mxx += Xi*Xi; Myy += Yi*Yi; Mxz += Xi*Zi; Myz += Yi*Zi; Mzz += Zi*Zi;
    }
    Mxy /= n; Mxx /= n; Myy /= n; Mxz /= n; Myz /= n; Mzz /= n;
    const double Mz = Mxx+Myy, Cov_xy = Mxx*Myy-Mxy*Mxy, Var_z = Mzz-Mz*Mz;
    const double A2 = 4*Cov_xy-3*Mz*Mz-Mzz;
    const double A1 = Var_z*Mz+4.0*Cov_xy*Mz-Mxz*Mxz-Myz*Myz;
    const double A0 = Mxz*(Mxz*Myy-Myz*Mxy)+Myz*(Myz*Mxx-Mxz*Mxy)-Var_z*Cov_xy;
    const double A22 = A2+A2;
    double y = A0, x = 0.0;
    for (int i = 0; i < max_iter; ++i) {
        const double Dy = A1+x*(A22+16.0*x*x);
        if (Dy == 0) break;
        const double x_new = x-y/Dy;
        if (x_new == x || !std::isfinite(x_new)) break;
        const double y_new = A0+x_new*(A1+x_new*(A2+4.0*x_new*x_new));
        if (std::abs(y_new) >= std::abs(y)) break;
        x = x_new;
        y = y_new;
    }
    const double det = x*x-x*Mz+Cov_xy;
    if (det == 0) return {mx, my, inf};
    const double X_center = (Mxz*(Myy-x)-Myz*Mxy)/det/2.0;
    const double Y_center = (Myz*(Mxx-x)-Mxz*Mxy)/det/2.0;
    return {X_center+mx, Y_center+my, std::sqrt(std::abs(X_center*X_center+Y_center*Y_center+Mz))};
}

// The sign of the determinant of [[1, a], [1, b], [1, c]], the orientation of three points.
double orientation(V a, V b, V c) { return sign((b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y)); }

// line_segment_intersection.lines_segments_intersect_indicator().
bool segments_intersect(V a0, V a1, V b0, V b1) {
    constexpr double epsilon = 1e-6;
    const auto cross = [](std::array<double, 3> p, std::array<double, 3> q) {
        return std::array<double, 3>{p[1]*q[2]-p[2]*q[1], p[2]*q[0]-p[0]*q[2], p[0]*q[1]-p[1]*q[0]};
    };
    const auto line_a = cross({a0.x, a0.y, 1}, {a1.x, a1.y, 1});
    const auto line_b = cross({b0.x, b0.y, 1}, {b1.x, b1.y, 1});
    const auto inter = cross(line_a, line_b);
    if (std::abs(inter[2]) < epsilon) {
        // Parallel.
        const V difference = a1-a0;
        bool maybe_overlap;
        double slope;
        if (difference.x < epsilon) {
            maybe_overlap = std::abs(a0.x-b0.x) < epsilon;
            slope = inf;
        } else {
            slope = difference.y/difference.x;
            maybe_overlap = std::abs((a0.y-slope*a0.x)-(b0.y-slope*b0.x)) < epsilon;
        }
        if (!maybe_overlap) return false;
        const auto axis = [&](V p) { return slope > 1 ? p.y : p.x; };
        double left_end, right_start;
        if (axis(a0) < axis(b0)) {
            left_end = axis(a1);
            right_start = std::min(axis(b0), axis(b1));
        } else {
            left_end = axis(b1);
            right_start = std::min(axis(a0), axis(a1));
        }
        return left_end >= right_start;
    }
    const double ix = inter[0]/inter[2], iy = inter[1]/inter[2];
    const auto within = [&](double v, double p, double q) {
        return std::min(p, q)-epsilon <= v && v <= std::max(p, q)+epsilon;
    };
    return within(ix, a0.x, a1.x) && within(ix, b0.x, b1.x) && within(iy, a0.y, a1.y) && within(iy, b0.y, b1.y);
}

// ------------------------------------------------------------------ sorting_cones/

struct Flat { V p; int type{}; };

int invert(int type) { return type == left_type ? right_type : type == right_type ? left_type : type; }

class Sorter {
public:
    Sorter(const ConePathSettings& s, const std::vector<Flat>& cones, V car_pos, V car_dir)
        : s_(s), cones_(cones), pos_(car_pos), dir_(car_dir) {}

    // Returns the configurations sorted by cost, best first, or nothing.
    std::optional<std::vector<std::vector<int>>> configurations(int cone_type) const {
        if (cones_.size() < 3) return std::nullopt;
        const auto first_k = select_first_k_starting_cones(cone_type);
        if (!first_k) return std::nullopt;
        const int start_idx = (*first_k)[0];
        const std::vector<int> must_be = first_k->size() > 1 ? *first_k : std::vector<int>{};
        const int n_neighbors = std::min(s_.max_neighbours, static_cast<int>(cones_.size())-1);
        auto configs = end_configurations(cone_type, n_neighbors, start_idx, must_be);
        if (configs.empty()) return std::nullopt;
        const auto costs = cost_configurations(configs, cone_type);
        std::vector<std::size_t> order(configs.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return costs[a] < costs[b]; });
        std::vector<std::vector<int>> sorted;
        for (const auto i : order) sorted.push_back(configs[i]);
        return sorted;
    }

private:
    const ConePathSettings& s_;
    const std::vector<Flat>& cones_;
    V pos_, dir_;

    // core_trace_sorter.mask_cone_can_be_first_in_config()
    std::pair<std::vector<double>, std::vector<bool>> mask_can_be_first(int cone_type) const {
        std::vector<double> distances(cones_.size());
        std::vector<bool> valid(cones_.size());
        const double facing = angle_of(dir_);
        for (std::size_t i = 0; i < cones_.size(); ++i) {
            const V rel = rotate(cones_[i].p-pos_, -facing);
            const double rel_angle = angle_of(rel);
            distances[i] = norm(rel);
            const bool in_ellipse = inside_ellipse(cones_[i].p, pos_, dir_, s_.max_distance_to_first_m*1.5, s_.max_distance_to_first_m/1.5);
            const double valid_sign = cone_type == left_type ? 1 : -1;
            const bool valid_side = sign(rel_angle) == valid_sign;
            const bool valid_angle = std::abs(rel_angle) < pi-pi/5;
            const bool valid_angle_min = std::abs(rel_angle) > pi/10;
            const bool right_colour = cones_[i].type == cone_type;
            const bool side = (valid_side && valid_angle && valid_angle_min) || right_colour;
            const bool not_opposite = cones_[i].type != invert(cone_type);
            valid[i] = in_ellipse && side && not_opposite;
        }
        return {distances, valid};
    }

    // core_trace_sorter.select_starting_cone()
    std::optional<int> select_starting_cone(int cone_type, const std::vector<int>* skip) const {
        auto [distances, valid] = mask_can_be_first(cone_type);
        if (skip)
            for (const int i : *skip) valid[static_cast<std::size_t>(i)] = false;
        std::vector<double> copy = distances;
        bool any = false;
        for (std::size_t i = 0; i < copy.size(); ++i) {
            if (!valid[i]) copy[i] = inf;
            any = any || valid[i];
        }
        if (!any) return std::nullopt;
        std::vector<int> order(copy.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return copy[static_cast<std::size_t>(a)] < copy[static_cast<std::size_t>(b)]; });
        std::optional<int> start;
        for (const int idx : order) {
            if (!skip || std::find(skip->begin(), skip->end(), idx) == skip->end()) { start = idx; break; }
        }
        if (start && copy[static_cast<std::size_t>(*start)] > s_.max_distance_to_first_m) start.reset();
        return start;
    }

    // core_trace_sorter.select_first_k_starting_cones(): the nearest cone ahead and, if there is one at a plausible
    // distance, the nearest behind, ordered along the car's direction.
    std::optional<std::vector<int>> select_first_k_starting_cones(int cone_type) const {
        auto index_1 = select_starting_cone(cone_type, nullptr);
        if (!index_1) return std::nullopt;
        std::vector<int> skip;
        for (std::size_t i = 0; i < cones_.size(); ++i)
            if (std::abs(vec_angle_between(cones_[i].p-pos_, dir_)) < pi/2) skip.push_back(static_cast<int>(i));
        if (std::find(skip.begin(), skip.end(), *index_1) == skip.end()) skip.push_back(*index_1);
        auto index_2 = select_starting_cone(cone_type, &skip);
        if (!index_2) return std::vector<int>{*index_1};
        int i1 = *index_1, i2 = *index_2;
        const V d1 = cones_[static_cast<std::size_t>(i1)].p-cones_[static_cast<std::size_t>(i2)].p;
        const V d2 = cones_[static_cast<std::size_t>(i2)].p-cones_[static_cast<std::size_t>(i1)].p;
        if (vec_angle_between(d1, dir_) > vec_angle_between(d2, dir_)) std::swap(i1, i2);
        const double dist = norm(d1);
        if (dist > s_.max_neighbour_distance_m*1.1 || dist < 1.4) return std::vector<int>{i1};
        return std::vector<int>{i2, i1};
    }

    // adjacency_matrix.create_adjacency_matrix() and common.breadth_first_order(): each cone's neighbours, those among
    // each other's closest within the distance, ascending, and the cones reachable from the start.
    std::pair<std::vector<std::vector<int>>, std::size_t> adjacency(int cone_type, int n_neighbors, int start_idx) const {
        const std::size_t n = cones_.size();
        std::vector<std::vector<double>> d(n, std::vector<double>(n));
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j) d[i][j] = i == j ? inf : squared_distance(cones_[i].p, cones_[j].p);
        const int other = invert(cone_type);
        for (std::size_t i = 0; i < n; ++i)
            if (cones_[i].type == other)
                for (std::size_t j = 0; j < n; ++j) d[i][j] = d[j][i] = inf;
        std::vector<std::vector<bool>> adj(n, std::vector<bool>(n, false));
        for (std::size_t i = 0; i < n; ++i) {
            std::vector<std::size_t> order(n);
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return d[i][a] < d[i][b]; });
            for (int k = 0; k < n_neighbors; ++k) adj[i][order[static_cast<std::size_t>(k)]] = true;
        }
        const double max_sq = s_.max_neighbour_distance_m*s_.max_neighbour_distance_m;
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                if (d[i][j] > max_sq) adj[i][j] = false;
        std::vector<std::vector<int>> neighbours(n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                if (adj[i][j] && adj[j][i]) neighbours[i].push_back(static_cast<int>(j));
        // Breadth first from the start.
        std::vector<bool> visited(n, false);
        std::vector<int> queue{start_idx};
        visited[static_cast<std::size_t>(start_idx)] = true;
        for (std::size_t q = 0; q < queue.size(); ++q)
            for (const int next : neighbours[static_cast<std::size_t>(queue[q])])
                if (!visited[static_cast<std::size_t>(next)]) {
                    visited[static_cast<std::size_t>(next)] = true;
                    queue.push_back(next);
                }
        return {neighbours, queue.size()};
    }

    // end_configurations.neighbor_bool_mask_can_be_added_to_attempt()
    std::vector<bool> can_be_added(int cone_type, const std::vector<int>& attempt, int position, const std::vector<int>& neighbours) const {
        const V dir_n = dir_*(1.0/norm(dir_));
        const auto P = [&](int i) { return cones_[static_cast<std::size_t>(i)].p; };
        std::vector<bool> can(neighbours.size());
        for (std::size_t i = 0; i < neighbours.size(); ++i)
            can[i] = std::find(attempt.begin(), attempt.begin()+position+1, neighbours[i]) == attempt.begin()+position+1;
        if (position >= 1) {
            const V last = P(attempt[static_cast<std::size_t>(position)]);
            const V second_to_last = P(attempt[static_cast<std::size_t>(position-1)]);
            for (std::size_t i = 0; i < neighbours.size(); ++i)
                can[i] = can[i] && inside_ellipse(P(neighbours[i]), last, last-second_to_last, 6, 3);
        }
        if (position == 0) {
            const double car_angle = angle_of(dir_n);
            const double expected = cone_type == left_type ? 1 : -1;
            for (std::size_t i = 0; i < neighbours.size(); ++i) {
                const double diff = angle_difference(angle_of(P(neighbours[i])-pos_), car_angle);
                can[i] = can[i] && (sign(diff) == expected || std::abs(diff) < 5.0*pi/180);
            }
        }
        for (std::size_t i = 0; i < can.size(); ++i) {
            if (!can[i]) continue;
            const int candidate = neighbours[i];
            // A neighbour between the last cone and the candidate means the candidate would skip it.
            for (const int neighbour : neighbours) {
                if (neighbour == neighbours[i]) continue;
                const V to_last = P(attempt[static_cast<std::size_t>(position)])-P(neighbour);
                const V to_candidate = P(candidate)-P(neighbour);
                if (norm(to_candidate) < 6.0 && norm(to_last) < 6.0 && vec_angle_between(to_last, to_candidate) > 150.0*pi/180) {
                    can[i] = false;
                    break;
                }
            }
            const V candidate_pos = P(candidate);
            if (can[i] && position >= 1) {
                const V second_to_last = P(attempt[static_cast<std::size_t>(position-1)]);
                const V last = P(attempt[static_cast<std::size_t>(position)]);
                const double angle_1 = angle_of(last-second_to_last);
                const double angle_2 = angle_of(candidate_pos-last);
                const double difference = angle_difference(angle_2, angle_1);
                const double length = norm(candidate_pos-last);
                if (std::abs(difference) > s_.absolute_angle_rad) can[i] = false;
                else if (cone_type == left_type) can[i] = difference < s_.directional_angle_rad || length < 4.0;
                else can[i] = difference > -s_.directional_angle_rad || length < 4.0;
                if (position >= 2) {
                    const V third_to_last = P(attempt[static_cast<std::size_t>(position-2)]);
                    const double angle_3 = angle_of(second_to_last-third_to_last);
                    const double difference_2 = angle_difference(angle_1, angle_3);
                    if (sign(difference) != sign(difference_2) && std::abs(difference-difference_2) > 1.3) can[i] = false;
                }
            }
            if (can[i] && position == 1) {
                const V start = P(attempt[0]);
                can[i] = can[i] && vec_angle_between(dir_, candidate_pos-start) < pi/2;
            }
            if (can[i] && position >= 0) {
                const V last = P(attempt[static_cast<std::size_t>(position)]);
                const double car_size = 2.1;
                const V car_start = pos_-dir_n*(car_size/2), car_end = pos_+dir_n*car_size;
                can[i] = can[i] && !segments_intersect(last, candidate_pos, car_start, car_end);
            }
        }
        return can;
    }

    // find_configs_and_scores.calc_scores_and_end_configurations(), without the costs, and
    // end_configurations.find_all_end_configurations(): every ordering the search reaches, as rows padded with -1.
    std::vector<std::vector<int>> end_configurations(int cone_type, int n_neighbors, int start_idx, const std::vector<int>& must_be) const {
        const auto [neighbours, reachable] = adjacency(cone_type, n_neighbors, start_idx);
        const int target_length = static_cast<int>(std::min<std::size_t>(reachable, static_cast<std::size_t>(s_.max_trace_length)));
        if (!must_be.empty() && target_length < static_cast<int>(must_be.size())) return {};
        std::vector<std::vector<int>> ends;
        std::vector<int> attempt(static_cast<std::size_t>(target_length), -1);
        std::vector<std::pair<int, int>> stack;
        if (!must_be.empty()) {
            const int pos = static_cast<int>(must_be.size())-1;
            for (int i = 0; i < pos; ++i) attempt[static_cast<std::size_t>(i)] = must_be[static_cast<std::size_t>(i)];
            stack.push_back({must_be.back(), pos});
        } else {
            stack.push_back({start_idx, 0});
        }
        while (!stack.empty()) {
            const auto [next_idx, position] = stack.back();
            stack.pop_back();
            attempt[static_cast<std::size_t>(position)] = next_idx;
            for (std::size_t i = static_cast<std::size_t>(position)+1; i < attempt.size(); ++i) attempt[i] = -1;
            const auto& candidates = neighbours[static_cast<std::size_t>(next_idx)];
            const auto can = can_be_added(cone_type, attempt, position, candidates);
            const bool has_valid = position < target_length-1 && std::any_of(can.begin(), can.end(), [](bool b) { return b; });
            if (has_valid) {
                for (std::size_t i = 0; i < can.size(); ++i)
                    if (can[i]) stack.push_back({candidates[i], position+1});
            } else {
                ends.push_back(attempt);
            }
        }
        const auto length = [](const std::vector<int>& c) { return static_cast<int>(std::count_if(c.begin(), c.end(), [](int v) { return v != -1; })); };
        std::vector<std::vector<int>> kept;
        for (const auto& c : ends)
            if (length(c) > 2) kept.push_back(c);
        if (!must_be.empty() && !kept.empty()) {
            std::vector<std::vector<int>> prefixed;
            for (const auto& c : kept)
                if (std::equal(must_be.begin(), must_be.end(), c.begin())) prefixed.push_back(c);
            kept = prefixed;
        }
        std::vector<std::vector<int>> atleast3;
        for (auto c : kept) {
            if (length(c) < 3) continue;
            // The last cone of a configuration goes if it is not of the side's colour.
            const int width = static_cast<int>(c.size());
            const auto first_minus = std::find(c.begin(), c.end(), -1);
            const int argmax = first_minus == c.end() ? 0 : static_cast<int>(first_minus-c.begin());
            const int last_idx = ((argmax-1)%width+width)%width;
            if (cones_[static_cast<std::size_t>(c[static_cast<std::size_t>(last_idx)])].type != cone_type) c[static_cast<std::size_t>(last_idx)] = -1;
            if (length(c) >= 3) atleast3.push_back(c);
        }
        std::sort(atleast3.begin(), atleast3.end());
        atleast3.erase(std::unique(atleast3.begin(), atleast3.end()), atleast3.end());
        // A configuration that another covers wherever it has a cone is a duplicate.
        std::vector<std::vector<int>> result;
        for (std::size_t j = 0; j < atleast3.size(); ++j) {
            int covering = 0;
            for (std::size_t i = 0; i < atleast3.size(); ++i) {
                bool all = true;
                for (std::size_t k = 0; k < atleast3[j].size() && all; ++k)
                    all = atleast3[i][k] == atleast3[j][k] || atleast3[j][k] == -1;
                if (all) ++covering;
            }
            if (covering <= 1) result.push_back(atleast3[j]);
        }
        return result;
    }

    // nearby_cone_search._impl_number_cones_on_each_side_for_each_config()
    std::pair<std::vector<int>, std::vector<int>> cones_on_each_side(const std::vector<std::vector<int>>& configs, int cone_type,
                                                                     double search_distance, double search_angle) const {
        const std::size_t n = cones_.size();
        std::vector<int> in_all;
        for (const auto& c : configs)
            for (const int v : c)
                if (v != -1) in_all.push_back(v);
        std::sort(in_all.begin(), in_all.end());
        in_all.erase(std::unique(in_all.begin(), in_all.end()), in_all.end());
        const auto dsq = [&](std::size_t i, std::size_t j) { return i == j ? 1e7 : squared_distance(cones_[i].p, cones_[j].p); };
        const double range_sq = search_distance*search_distance;
        const auto within = [&](int i, int j) { return dsq(static_cast<std::size_t>(i), static_cast<std::size_t>(j)) < range_sq; };
        // sorted_set_diff(a, b): a with the element at searchsorted(a, v) removed for every v in b, as NumPy does it.
        const auto set_diff = [](const std::vector<int>& a, const std::vector<int>& b) {
            std::vector<bool> keep(a.size(), true);
            for (const int v : b) {
                const auto at = static_cast<std::size_t>(std::lower_bound(a.begin(), a.end(), v)-a.begin());
                if (at < keep.size()) keep[at] = false;
            }
            std::vector<int> out;
            for (std::size_t i = 0; i < a.size(); ++i)
                if (keep[i]) out.push_back(a[i]);
            return out;
        };
        std::vector<int> nearby;
        for (std::size_t j = 0; j < n; ++j)
            for (const int i : in_all)
                if (within(i, static_cast<int>(j))) { nearby.push_back(static_cast<int>(j)); break; }
        const auto close = set_diff(nearby, in_all);
        const auto search_direction = [&](int a, int b) {
            const V track = cones_[static_cast<std::size_t>(b)].p-cones_[static_cast<std::size_t>(a)].p;
            const V d = rotate(track, cone_type == right_type ? pi/2 : -pi/2);
            return d*(1.0/norm(d));
        };
        std::vector<int> good(configs.size(), 0), bad(configs.size(), 0);
        for (std::size_t ci = 0; ci < configs.size(); ++ci) {
            std::vector<int> c;
            for (const int v : configs[ci])
                if (v != -1) c.push_back(v);
            auto other = close;
            const auto extra = set_diff(in_all, c);
            other.insert(other.end(), extra.begin(), extra.end());
            for (std::size_t j = 0; j < c.size(); ++j) {
                V direction;
                if (j == 0) direction = search_direction(c[0], c[1]);
                else if (j == c.size()-1) direction = search_direction(c[j-1], c[j]);
                else direction = search_direction(c[j-1], c[j+1]);
                for (const int idx : other) {
                    if (!within(c[j], idx)) continue;
                    const V to_other = cones_[static_cast<std::size_t>(idx)].p-cones_[static_cast<std::size_t>(c[j])].p;
                    if (vec_angle_between(to_other, direction) < search_angle/2) ++good[ci];
                    if (vec_angle_between(to_other, direction*-1.0) < search_angle/2) ++bad[ci];
                }
            }
        }
        return {good, bad};
    }

    // cost_function.cost_configurations()
    std::vector<double> cost_configurations(const std::vector<std::vector<int>>& configs, int cone_type) const {
        const std::size_t count = configs.size();
        const std::size_t width = configs.front().size();
        // points[-1] is the last cone, as NumPy reads a -1 index.
        const auto P = [&](int i) { return cones_[i < 0 ? cones_.size()-1 : static_cast<std::size_t>(i)].p; };
        std::vector<double> angle_cost(count), distance_cost(count), number_cost(count), initial_cost(count), either_cost(count),
            wrong_cost(count);
        for (std::size_t r = 0; r < count; ++r) {
            const auto& c = configs[r];
            // calc_angle_cost_for_configuration(): all_to_next[j] = points[c[j]] - points[c[j+1]], set to 100 where c[j+1]
            // is padding.
            std::vector<V> to_next(width-1);
            for (std::size_t j = 0; j+1 < width; ++j) {
                to_next[j] = P(c[j])-P(c[j+1]);
                if (c[j+1] == -1) to_next[j] = {100, 100};
            }
            double cost_sum = 0, part = 0, under = 0;
            for (std::size_t j = 0; j+2 < width; ++j) {
                const V middle_to_next = to_next[j+1], middle_to_prev = to_next[j]*-1.0;
                const double angle = vec_angle_between(middle_to_next, middle_to_prev);
                const bool is_part = c[j+2] != -1;
                if (!is_part) continue;
                cost_sum += (pi-angle)/pi;
                part += 1;
                if (angle < 40.0*pi/180) under += 1;
            }
            angle_cost[r] = cost_sum/part*(under+1);
            // calc_distance_cost() with a threshold of 3 m.
            double residual = 0;
            for (std::size_t j = 0; j+1 < width; ++j) {
                if (c[j+1] == -1) continue;
                residual += std::max(0.0, norm(P(c[j+1])-P(c[j]))-3.0);
            }
            distance_cost[r] = residual;
            const double n = static_cast<double>(std::count_if(c.begin(), c.end(), [](int v) { return v != -1; }));
            number_cost[r] = 1/n;
            initial_cost[r] = vec_angle_between(P(c[1])-P(c[0]), dir_);
            // calc_wrong_direction_cost()
            std::vector<V> valid;
            for (const int v : c)
                if (v != -1) valid.push_back(P(v));
            if (valid.size() != 3) {
                std::vector<double> angles;
                for (std::size_t j = 0; j+1 < valid.size(); ++j) angles.push_back(angle_of(valid[j+1]-valid[j]));
                const double unwanted = cone_type == left_type ? 1 : -1;
                double sum = 0;
                for (std::size_t j = 0; j+1 < angles.size(); ++j) {
                    const double d = angle_difference(angles[j], angles[j+1]);
                    if (sign(d) == unwanted && std::abs(d) > 40.0*pi/180) sum += d;
                }
                wrong_cost[r] = std::abs(sum);
            }
        }
        const auto [good, bad] = cones_on_each_side(configs, cone_type, 6.0, pi/1.5);
        std::vector<int> diff(count);
        for (std::size_t r = 0; r < count; ++r) diff[r] = good[r]-bad[r];
        const int m_value = *std::min_element(diff.begin(), diff.end());
        for (std::size_t r = 0; r < count; ++r) either_cost[r] = 1.0/static_cast<double>(diff[r]+std::abs(m_value)+1);
        const std::array<double, 7> raw{1000.0, 200.0, 5000.0, 1000.0, 0.0, 1000.0, 1000.0};
        const double total = std::accumulate(raw.begin(), raw.end(), 0.0);
        std::vector<double> costs(count);
        for (std::size_t r = 0; r < count; ++r)
            costs[r] = angle_cost[r]*(raw[0]/total)+distance_cost[r]*(raw[1]/total)+number_cost[r]*(raw[2]/total)+
                       initial_cost[r]*(raw[3]/total)+0.0*(raw[4]/total)+either_cost[r]*(raw[5]/total)+wrong_cost[r]*(raw[6]/total);
        return costs;
    }
};

// combine_traces.calc_angle_change_at_position()
double angle_change_at(const std::vector<Flat>& cones, const std::vector<int>& config, std::size_t position) {
    const V prev = cones[static_cast<std::size_t>(config[position-1])].p;
    const V here = cones[static_cast<std::size_t>(config[position])].p;
    const V next = cones[static_cast<std::size_t>(config[position+1])].p;
    return angle_difference(angle_of(next-here), angle_of(prev-here));
}

// combine_traces.calc_final_configs_for_left_and_right() for both sides found: the best of each, cut back where they
// share a cone.
std::pair<std::vector<int>, std::vector<int>> handle_same_cone(const std::vector<Flat>& cones, std::vector<int> left, std::vector<int> right) {
    std::optional<std::size_t> li, ri;
    for (std::size_t i = 0; i < left.size(); ++i)
        for (std::size_t j = 0; j < right.size(); ++j)
            if (left[i] == right[j]) {
                if (!li || i < *li) li = i;
                if (!ri || j < *ri) ri = j;
            }
    if (!li) return {left, right};
    const std::size_t left_index = *li, right_index = *ri;
    std::optional<std::size_t> left_stop, right_stop;
    if (left_index > 0 && right_index > 0) {
        const V intersection = cones[static_cast<std::size_t>(left[left_index])].p;
        const double to_left = norm(intersection-cones[static_cast<std::size_t>(left[left_index-1])].p);
        const double to_right = norm(intersection-cones[static_cast<std::size_t>(right[right_index-1])].p);
        const bool left_low = to_left < 3.0, right_low = to_right < 3.0;
        if ((left_low || right_low) && !(left_low && right_low)) {
            if (left_low) { left_stop = left.size(); right_stop = right_index; }
            else { left_stop = left_index; right_stop = right.size(); }
        }
    }
    if (!left_stop && !right_stop && left[left_index] == right[right_index] && left_index >= 1 && left_index+1 < left.size() &&
        right_index >= 1 && right_index+1 < right.size()) {
        const double angle_left = angle_change_at(cones, left, left_index), angle_right = angle_change_at(cones, right, right_index);
        const double sign_left = sign(angle_left), sign_right = sign(angle_right);
        const double absolute_diff = std::abs(std::abs(angle_left)-std::abs(angle_right));
        const std::size_t n_cones_diff = left.size() > right.size() ? left.size()-right.size() : right.size()-left.size();
        if (sign_left == sign_right) {
            if (sign_left == 1) { left_stop = left.size(); right_stop = right_index; }
            else { left_stop = left_index; right_stop = right.size(); }
        } else if (n_cones_diff > 2) {
            if (left.size() > right.size()) { left_stop = left.size(); right_stop = right_index; }
            else { left_stop = left_index; right_stop = right.size(); }
        } else if (absolute_diff > 5.0*pi/180) {
            if (std::abs(angle_left) > std::abs(angle_right)) { left_stop = left.size(); right_stop = right_index; }
            else { left_stop = left_index; right_stop = right.size(); }
        } else {
            left_stop = left_index;
            right_stop = right_index;
        }
    } else if (!left_stop && !right_stop) {
        const bool left_at_end = left_index == left.size()-1, right_at_end = right_index == right.size()-1;
        if (left_at_end && right_at_end) { left_stop = left.size()-1; right_stop = right.size()-1; }
        else if (left_at_end) { right_stop = right.size(); left_stop = left_index; }
        else if (right_at_end) { left_stop = left.size(); right_stop = right_index; }
        else { left_stop = left_index; right_stop = right_index; }
    }
    if (left_stop) left.resize(std::min(*left_stop, left.size()));
    if (right_stop) right.resize(std::min(*right_stop, right.size()));
    return {left, right};
}

// ------------------------------------------------------------------ cone_matching/

// match_directions.calculate_match_search_direction(): across the track from each cone, perpendicular to the trace.
std::vector<V> match_search_directions(const std::vector<V>& cones, int cone_type) {
    const auto one = [&](std::size_t a, std::size_t b) {
        const V d = rotate(cones[b]-cones[a], cone_type == right_type ? pi/2 : -pi/2);
        return d*(1.0/norm(d));
    };
    std::vector<V> out(cones.size());
    out.front() = one(0, 1);
    out.back() = one(cones.size()-2, cones.size()-1);
    for (std::size_t i = 1; i+1 < cones.size(); ++i) out[i] = one(i-1, i+1);
    return out;
}

// functional_cone_matching.find_boolean_mask_of_all_potential_matches(): at most the two nearest cones across, within
// an ellipse ahead of each cone's search direction, whose own search direction faces back. FaSTTUBe fails on a single
// cone across, having no direction for it; this finds no match there instead.
std::vector<std::vector<bool>> potential_matches(const std::vector<V>& start, const std::vector<V>& directions, const std::vector<V>& other,
                                                 const std::vector<V>& other_directions, double major, double minor, double max_angle) {
    std::vector<std::vector<bool>> mask(start.size(), std::vector<bool>(other.size(), false));
    if (start.empty() || other.empty() || other_directions.size() != other.size()) return mask;
    for (std::size_t i = 0; i < start.size(); ++i) {
        const double angle = angle_of(directions[i]);
        for (std::size_t j = 0; j < other.size(); ++j) {
            const V r = rotate(other[j]-start[i], -angle);
            bool m = r.x*r.x/(major*major)+r.y*r.y/(minor*minor) < 1;
            if (std::abs(angle_of(r)/2) > max_angle) m = false;
            if (vec_angle_between(directions[i], other_directions[j]) < pi/2) m = false;
            mask[i][j] = m;
        }
        std::vector<double> d(other.size());
        for (std::size_t j = 0; j < other.size(); ++j) d[j] = mask[i][j] ? norm(other[j]-start[i]) : inf;
        std::vector<std::size_t> order(other.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return d[a] < d[b]; });
        std::vector<bool> keep(other.size(), false);
        for (std::size_t k = 0; k < std::min<std::size_t>(2, order.size()); ++k)
            if (std::isfinite(d[order[k]])) keep[order[k]] = true;
        mask[i] = keep;
    }
    return mask;
}

// functional_cone_matching.select_best_match_candidate(), not monotonic: the nearest cone across, if any was a candidate.
std::vector<int> best_matches(const std::vector<V>& cones, const std::vector<std::vector<bool>>& mask, const std::vector<V>& other) {
    std::vector<int> matched(cones.size(), -1);
    if (other.empty()) return matched;
    for (std::size_t i = 0; i < cones.size(); ++i) {
        std::size_t best = 0;
        for (std::size_t j = 1; j < other.size(); ++j)
            if (squared_distance(cones[i], other[j]) < squared_distance(cones[i], other[best])) best = j;
        const bool any = std::any_of(mask[i].begin(), mask[i].end(), [](bool b) { return b; });
        matched[i] = any ? static_cast<int>(best) : -1;
    }
    return matched;
}

// functional_cone_matching.calculate_matches_for_side()
std::pair<std::vector<int>, std::vector<V>> matches_for_side(const std::vector<V>& cones, int cone_type, const std::vector<V>& other,
                                                            const ConePathSettings& s) {
    if (cones.size() <= 1) return {std::vector<int>(cones.size(), -1), {}};
    const auto directions = match_search_directions(cones, cone_type);
    const auto other_directions = other.size() > 1 ? match_search_directions(other, invert(cone_type)) : std::vector<V>{};
    const auto mask = potential_matches(cones, directions, other, other_directions, s.max_search_range_m*1.5, s.min_track_width_m,
                                        s.max_search_angle_rad);
    return {best_matches(cones, mask, other), directions};
}

// functional_cone_matching.insert_virtual_cones_to_existing()
std::vector<V> insert_virtual(const std::vector<V>& other, const std::vector<V>& virtual_cones, V car_pos) {
    std::vector<V> existing = other.size() > virtual_cones.size() ? other : virtual_cones;
    std::vector<V> to_insert = other.size() > virtual_cones.size() ? virtual_cones : other;
    std::vector<double> nearest(to_insert.size(), inf);
    for (std::size_t i = 0; i < to_insert.size(); ++i)
        for (const auto& e : existing) nearest[i] = std::min(nearest[i], squared_distance(to_insert[i], e));
    std::vector<std::size_t> order(to_insert.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return nearest[a] < nearest[b]; });
    for (const auto k : order) {
        const V cone = to_insert[k];
        std::vector<double> d(existing.size());
        for (std::size_t j = 0; j < existing.size(); ++j) d[j] = norm(existing[j]-cone);
        std::vector<std::size_t> by_distance(existing.size());
        std::iota(by_distance.begin(), by_distance.end(), 0);
        std::stable_sort(by_distance.begin(), by_distance.end(), [&](std::size_t a, std::size_t b) { return d[a] < d[b]; });
        std::size_t index;
        if (by_distance.size() == 1) {
            index = norm(cone-car_pos) < norm(existing[0]-car_pos) ? 0 : 1;
        } else {
            const std::size_t closest = by_distance[0], second = by_distance[1];
            if ((closest > second ? closest-second : second-closest) != 1) continue;
            const bool between = vec_angle_between(existing[closest]-cone, existing[second]-cone) > pi/2;
            if (between) index = std::min(closest, second)+1;
            else index = closest < second ? closest : closest+1;
        }
        existing.insert(existing.begin()+static_cast<std::ptrdiff_t>(index), cone);
    }
    // Remove cones where the trace turns back on itself, an angle under 85 degrees.
    if (existing.size() >= 3) {
        std::vector<bool> low(existing.size(), false);
        bool any = false;
        for (std::size_t k = 1; k+1 < existing.size(); ++k) {
            const double a = vec_angle_between(existing[k+1]-existing[k], existing[k-1]-existing[k]);
            low[k] = a < 85.0*pi/180;
            any = any || low[k];
        }
        if (any) {
            std::vector<V> kept;
            for (std::size_t k = 0; k < existing.size(); ++k)
                if (!low[k]) kept.push_back(existing[k]);
            existing = kept;
        }
    }
    return existing;
}

// functional_cone_matching.calculate_cones_for_other_side()
std::vector<V> cones_for_other_side(const std::vector<V>& cones, int cone_type, const std::vector<V>& other, V car_pos, const ConePathSettings& s) {
    const auto [matches, directions] = matches_for_side(cones, cone_type, other, s);
    std::vector<V> virtual_cones;
    for (std::size_t i = 0; i < matches.size(); ++i)
        if (matches[i] == -1) virtual_cones.push_back(cones[i]+directions[i]*s.min_track_width_m);
    std::vector<V> combined;
    if (other.empty()) combined = virtual_cones;
    else if (virtual_cones.empty()) combined = other;
    else combined = insert_virtual(other, virtual_cones, car_pos);
    if (combined.size() < 2) combined = other;
    return combined;
}

// ------------------------------------------------------------------ calculate_path/

struct Evaluator {
    fitpack::Curve curve;
    double max_u{};
    double predict_every{};
    bool empty{};
    std::vector<V> predict(std::optional<double> max = std::nullopt) const {
        if (empty) return {};
        const double stop = max.value_or(max_u);
        const auto count = static_cast<std::size_t>(std::max(0.0, std::ceil(stop/predict_every)));
        std::vector<V> out;
        out.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const auto p = fitpack::evaluate(curve, static_cast<double>(i)*predict_every);
            out.push_back({p[0], p[1]});
        }
        return out;
    }
    std::vector<double> u_eval() const {
        const auto count = static_cast<std::size_t>(std::max(0.0, std::ceil(max_u/predict_every)));
        std::vector<double> out(count);
        for (std::size_t i = 0; i < count; ++i) out[i] = static_cast<double>(i)*predict_every;
        return out;
    }
};

// A fit SciPy refuses, as FaSTTUBe catches it.
struct FitRefused : std::runtime_error { using std::runtime_error::runtime_error; };

// spline_fit.SplineFitterFactory.fit()
Evaluator fit(const std::vector<V>& trace, double smoothing, double predict_every, int max_degree) {
    Evaluator e;
    e.predict_every = predict_every;
    if (trace.size() < 2) { e.empty = true; return e; }
    const int k = std::clamp(static_cast<int>(trace.size())-1, 1, max_degree);
    std::vector<double> u{0.0};
    std::vector<std::array<double, 2>> points{{trace[0].x, trace[0].y}};
    for (std::size_t i = 1; i < trace.size(); ++i) {
        u.push_back(u.back()+norm(trace[i]-trace[i-1]));
        points.push_back({trace[i].x, trace[i].y});
    }
    try {
        e.curve = fitpack::fit_curve(u, points, k, smoothing);
    } catch (const std::invalid_argument& error) {
        throw FitRefused(error.what());
    }
    e.max_u = u.back();
    return e;
}

// NumPy's sum of a contiguous array of doubles: pairwise, with eight accumulators in blocks of up to 128, and plain
// addition below eight. The parameterisation's sample count turns on the last bit of a path's length, so its sums must
// be NumPy's own.
double numpy_pairwise_sum(const double* a, std::size_t n) {
    if (n < 8) {
        double res = 0.;
        for (std::size_t i = 0; i < n; ++i) res += a[i];
        return res;
    }
    if (n <= 128) {
        double r[8];
        for (std::size_t j = 0; j < 8; ++j) r[j] = a[j];
        std::size_t i = 8;
        for (; i < n-(n%8); i += 8)
            for (std::size_t j = 0; j < 8; ++j) r[j] += a[i+j];
        double res = ((r[0]+r[1])+(r[2]+r[3]))+((r[4]+r[5])+(r[6]+r[7]));
        for (; i < n; ++i) res += a[i];
        return res;
    }
    std::size_t n2 = n/2;
    n2 -= n2%8;
    return numpy_pairwise_sum(a, n2)+numpy_pairwise_sum(a+n2, n-n2);
}

double numpy_sum(const std::vector<double>& values) { return 0.0+numpy_pairwise_sum(values.data(), values.size()); }

std::vector<double> distances_to_next(const std::vector<V>& path) {
    std::vector<double> d;
    for (std::size_t i = 1; i < path.size(); ++i) d.push_back(norm(path[i]-path[i-1]));
    return d;
}

// path_parameterization.calculate_path_curvature() and _calculate_path_curvature(), for a path that is not closed.
std::vector<double> path_curvature(const std::vector<V>& points) {
    const int n = static_cast<int>(points.size());
    int window = std::min(n/5, 30);
    if (window%2 == 0) window += 1;
    const int half = window/2;
    std::vector<double> curvature(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        std::vector<int> w;
        for (int k = -half; k <= half; ++k) w.push_back(((i+k)%n+n)%n);
        std::size_t cutoff = 0;
        bool broken = false;
        for (std::size_t k = 1; k < w.size(); ++k)
            if (w[k]-w[k-1] != 1) { cutoff = k; broken = true; break; }
        if (broken) {
            if (i < window) w.erase(w.begin(), w.begin()+static_cast<std::ptrdiff_t>(cutoff));
            else w.resize(cutoff);
        }
        std::vector<V> in_window;
        for (const int k : w) in_window.push_back(points[static_cast<std::size_t>(k)]);
        const double radius = std::min(std::max(circle_fit(in_window)[2], 1.0), 3000.0);
        const V a = in_window.front(), b = in_window[in_window.size()/2], c = in_window.back();
        curvature[static_cast<std::size_t>(i)] = (1/radius)*orientation(a, b, c);
    }
    // uniform_filter1d(mode="nearest")
    const int size = std::max(2, window/2);
    std::vector<double> filtered(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        double sum = 0;
        for (int k = 0; k < size; ++k) {
            const int at = std::clamp(i-size/2+k, 0, n-1);
            sum += curvature[static_cast<std::size_t>(at)];
        }
        filtered[static_cast<std::size_t>(i)] = sum/size;
    }
    return filtered;
}

// path_parameterization.PathParameterizer.parameterize_path(), not closed.
std::vector<ConePathPoint> parameterize(const std::vector<V>& path, int horizon) {
    const auto d = distances_to_next(path);
    const double length = numpy_sum(d);
    const std::size_t head_count = std::min<std::size_t>(10, d.size());
    const double mean = head_count > 0 ? numpy_sum(std::vector<double>(d.begin(), d.begin()+static_cast<std::ptrdiff_t>(head_count)))/
                                              static_cast<double>(head_count)
                                        : std::numeric_limits<double>::quiet_NaN();
    const double predict_every = length/horizon/3;
    int skip = 1;
    if (std::isfinite(predict_every/mean)) skip = std::max(static_cast<int>(predict_every/mean), 1);
    std::vector<V> skipped;
    for (std::size_t i = 0; i < path.size(); i += static_cast<std::size_t>(skip)) skipped.push_back(path[i]);
    const auto spline = fit(skipped, 0.01, predict_every, 3);
    const auto points = spline.predict();
    const auto curvature = path_curvature(points);
    const auto u = spline.u_eval();
    const int n = static_cast<int>(points.size());
    std::vector<int> indices;
    const double step = static_cast<double>(n-1)/(horizon-1);
    for (int i = 0; i < horizon; ++i) indices.push_back(i == horizon-1 ? n-1 : static_cast<int>(std::floor(i*step)));
    for (std::size_t i = 1; i < indices.size(); ++i)
        if (indices[i] == indices[i-1]) throw FitRefused("Indices of resampled path appear twice");
    std::vector<ConePathPoint> out;
    for (const int i : indices) {
        const auto k = static_cast<std::size_t>(i);
        out.push_back({u[k], points[k].x, points[k].y, curvature[k]});
    }
    return out;
}

// path_calculator_helpers.calculate_almost_straight_path(): a chord of radius 1000 m through pi/50 in 40 points.
std::vector<V> almost_straight_path() {
    std::vector<V> points;
    const double maximum = pi/50;
    const double step = maximum/39;
    for (int i = 0; i < 40; ++i) {
        // np.linspace(0, maximum, 40): i times the step, the last exactly the stop.
        const double a = i == 39 ? maximum : i*step;
        V p{std::cos(a)-1, std::sin(a)};
        p = p*1000;
        p = rotate(p, -pi/2);
        points.push_back(p);
    }
    return points;
}

std::vector<V> xy(const std::vector<ConePathPoint>& path) {
    std::vector<V> out;
    for (const auto& p : path) out.push_back({p.x_m, p.y_m});
    return out;
}

class PathCalculation {
public:
    PathCalculation(const ConePathSettings& s, V pos, V dir, const std::vector<ConePathPoint>& previous)
        : s_(s), pos_(pos), dir_(dir), previous_(previous) {}

    // core_calculate_path.CalculatePath.run_path_calculation(), without a global path.
    std::pair<std::vector<ConePathPoint>, std::vector<V>> run(const std::vector<V>& left, const std::vector<V>& right,
                                                              const std::vector<int>& left_to_right, const std::vector<int>& right_to_left,
                                                              bool& from_previous) const {
        std::vector<V> basis;
        from_previous = false;
        if (left.size() < 3 && right.size() < 3) {
            basis = xy(previous_);
            from_previous = true;
        } else {
            const auto score = [](const std::vector<int>& matches) {
                long long count = 0, sum = 0;
                for (const int m : matches)
                    if (m != -1) { ++count; sum += m; }
                return std::pair<long long, long long>{count, sum};
            };
            const bool use_left = score(left_to_right) >= score(right_to_left);
            const auto& side = use_left ? left : right;
            const auto& matches = use_left ? left_to_right : right_to_left;
            const auto& other = use_left ? right : left;
            for (std::size_t i = 0; i < side.size(); ++i) {
                if (matches[i] == -1) continue;
                const V m = other[static_cast<std::size_t>(matches[i])];
                basis.push_back((side[i]+m)*0.5);
            }
            if (basis.size() < 2) {
                basis = xy(previous_);
                from_previous = true;
            }
        }
        std::vector<V> update;
        try {
            update = fit(basis, s_.smoothing, s_.predict_every_m, s_.max_degree).predict();
        } catch (const FitRefused&) {
            update = fit(xy(previous_), s_.smoothing, s_.predict_every_m, s_.max_degree).predict();
        }
        double closest = inf;
        for (const auto& p : update) closest = std::min(closest, norm(pos_-p));
        if (closest > s_.max_distance_for_valid_path_m) update = xy(previous_);
        std::vector<ConePathPoint> path;
        try {
            path = mpc_path(update);
        } catch (const FitRefused&) {
            // FaSTTUBe retries from the previous path, and raises if that fails too; a car cannot, so it keeps the
            // previous path as it was.
            try {
                path = mpc_path(xy(previous_));
            } catch (const FitRefused&) {
                path = previous_;
                from_previous = true;
            }
        }
        return {path, basis};
    }

private:
    const ConePathSettings& s_;
    V pos_, dir_;
    const std::vector<ConePathPoint>& previous_;

    std::vector<ConePathPoint> mpc_path(const std::vector<V>& update) const {
        auto path = connect_to_car(update);
        path = extend(path);
        path = remove_behind(path);
        path = fit(path, s_.smoothing, s_.predict_every_m, s_.max_degree).predict(s_.path_length_m*1.5);
        path = trim(path);
        return parameterize(path, s_.horizon_points);
    }

    std::vector<V> connect_to_car(std::vector<V> path) const {
        const double distance = norm(pos_-path[0]);
        const V car_to_first = path[0]-pos_;
        if (distance < 0.5 || vec_angle_between(car_to_first, dir_) > pi/2) return path;
        path.insert(path.begin(), pos_+car_to_first*(0.2/norm(car_to_first)));
        return path;
    }

    std::vector<V> extend(std::vector<V> path) const {
        std::vector<bool> ahead(path.size());
        for (std::size_t i = 0; i < path.size(); ++i) {
            const V d = path[i]-pos_;
            ahead[i] = d.x*dir_.x+d.y*dir_.y > 0;
        }
        for (std::size_t i = 0; i < ahead.size(); ++i)
            if (ahead[i]) { std::fill(ahead.begin()+static_cast<std::ptrdiff_t>(i), ahead.end(), true); break; }
        for (std::size_t i = ahead.size() > 20 ? ahead.size()-20 : 0; i < ahead.size(); ++i) ahead[i] = true;
        std::vector<V> in_front;
        for (std::size_t i = 0; i < path.size(); ++i)
            if (ahead[i]) in_front.push_back(path[i]);
        if (in_front.empty()) return path;
        double length = 0;
        for (const double d : distances_to_next(in_front)) length += d;
        if (length > s_.path_length_m) return path;
        const std::vector<V> relevant(in_front.size() > 20 ? in_front.end()-20 : in_front.begin(), in_front.end());
        const auto circle = circle_fit(relevant);
        const V centre{circle[0], circle[1]};
        const double radius = std::min(std::max(circle[2], 10.0), 100.0);
        std::vector<V> new_points;
        if (radius < 80) {
            const V a = relevant.front()-centre, b = relevant[relevant.size()/2]-centre, c = relevant.back()-centre;
            const double turn = orientation(a, b, c);
            const double start = angle_of(a), end = start+turn*pi;
            V first{};
            for (int i = 0; i < 50; ++i) {
                const double angle = i == 49 ? end : start+(end-start)/49*i;
                const V raw{std::cos(angle)*radius, std::sin(angle)*radius};
                if (i == 0) first = raw;
                new_points.push_back(raw-first+path.back());
            }
        } else {
            const V last = path.back(), direction = (last-path[path.size()-2])*(1.0/norm(last-path[path.size()-2]));
            for (int i = 0; i < 30; ++i) new_points.push_back(last+direction*i);
        }
        path.insert(path.end(), new_points.begin()+1, new_points.end());
        return path;
    }

    std::vector<V> remove_behind(const std::vector<V>& path) const {
        std::size_t best = 0;
        for (std::size_t i = 1; i < path.size(); ++i)
            if (norm(pos_-path[i]) < norm(pos_-path[best])) best = i;
        return std::vector<V>(path.begin()+static_cast<std::ptrdiff_t>(best), path.end());
    }

    std::vector<V> trim(const std::vector<V>& path) const {
        const auto d = distances_to_next(path);
        double cumulative = 0;
        std::size_t first = d.size();
        for (std::size_t i = 0; i < d.size(); ++i) {
            cumulative += d[i];
            if (cumulative > s_.path_length_m) { first = i; break; }
        }
        return std::vector<V>(path.begin(), path.begin()+static_cast<std::ptrdiff_t>(first));
    }
};

}  // namespace

void validate_cone_path_settings(const ConePathSettings& s) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    if (s.max_neighbours < 1 || s.max_trace_length < 3 || s.horizon_points < 2 || s.max_degree < 1 || s.max_degree > 5 ||
        !positive(s.max_neighbour_distance_m) || !positive(s.max_distance_to_first_m) || !positive(s.directional_angle_rad) ||
        !positive(s.absolute_angle_rad) || !positive(s.min_track_width_m) || !positive(s.max_search_range_m) ||
        !positive(s.max_search_angle_rad) || !std::isfinite(s.smoothing) || s.smoothing < 0 || !positive(s.predict_every_m) ||
        !positive(s.max_distance_for_valid_path_m) || !positive(s.path_length_m))
        throw std::invalid_argument("Cone path settings are out of range");
}

ConePathPlanner::ConePathPlanner(ConePathSettings settings) : settings_(settings) {
    validate_cone_path_settings(settings_);
    // CalculatePath.calculate_initial_path(), parameterised as FaSTTUBe's constructor does.
    const auto initial = fit(almost_straight_path(), settings_.smoothing, settings_.predict_every_m, settings_.max_degree).predict();
    previous_ = parameterize(initial, settings_.horizon_points);
}

ConePath ConePathPlanner::plan(std::span<const TypedCone> cones, Vec2 position, Vec2 direction) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(direction.x) || !std::isfinite(direction.y) ||
        std::hypot(direction.x, direction.y) == 0)
        throw std::invalid_argument("A cone path needs a finite position and a direction");
    for (const auto& c : cones)
        if (!std::isfinite(c.position.x) || !std::isfinite(c.position.y) || static_cast<int>(c.type) < 0 || static_cast<int>(c.type) > 4)
            throw std::invalid_argument("Every cone needs a finite position and a type");
    const V pos{position.x, position.y}, dir{direction.x, direction.y};
    // Flattened by type, as FaSTTUBe's flatten_cones_by_type_array() orders them; unknown cones dropped if not used.
    std::vector<Flat> flat;
    for (int type = 0; type <= 4; ++type)
        for (const auto& c : cones)
            if (static_cast<int>(c.type) == type && (type != 0 || settings_.use_unknown_cones)) flat.push_back({{c.position.x, c.position.y}, type});
    const Sorter sorter(settings_, flat, pos, dir);
    const auto left_configs = sorter.configurations(left_type);
    const auto right_configs = sorter.configurations(right_type);
    const auto valid = [](const std::vector<int>& c) {
        std::vector<int> out;
        for (const int v : c)
            if (v != -1) out.push_back(v);
        return out;
    };
    std::vector<int> left_config, right_config;
    if (left_configs && right_configs) {
        std::tie(left_config, right_config) = handle_same_cone(flat, valid(left_configs->front()), valid(right_configs->front()));
    } else if (left_configs) {
        left_config = valid(left_configs->front());
    } else if (right_configs) {
        right_config = valid(right_configs->front());
    }
    ConePath out;
    std::vector<V> left, right;
    for (const int i : left_config) left.push_back(flat[static_cast<std::size_t>(i)].p);
    for (const int i : right_config) right.push_back(flat[static_cast<std::size_t>(i)].p);
    for (const auto& p : left) out.left.push_back({p.x, p.y});
    for (const auto& p : right) out.right.push_back({p.x, p.y});
    // calculate_virtual_cones_for_both_sides()
    std::vector<V> left_virtual, right_virtual;
    std::vector<int> left_to_right, right_to_left;
    if (!(left.size() < 2 && right.size() < 2)) {
        const std::size_t min_len = std::min(left.size(), right.size()), max_len = std::max(left.size(), right.size());
        const bool discard_one_side = min_len == 0 || static_cast<double>(max_len)/static_cast<double>(min_len) > 2;
        if (discard_one_side) {
            if (left.size() < right.size()) left.clear();
            else right.clear();
        }
        right_virtual = left.size() >= 2 ? cones_for_other_side(left, left_type, right, pos, settings_) : right;
        left_virtual = right.size() >= 2 ? cones_for_other_side(right, right_type, left, pos, settings_) : left;
        left_to_right = matches_for_side(left_virtual, left_type, right_virtual, settings_).first;
        right_to_left = matches_for_side(right_virtual, right_type, left_virtual, settings_).first;
    }
    for (const auto& p : left_virtual) out.left_with_virtual.push_back({p.x, p.y});
    for (const auto& p : right_virtual) out.right_with_virtual.push_back({p.x, p.y});
    out.left_to_right = left_to_right;
    out.right_to_left = right_to_left;
    const PathCalculation calculation(settings_, pos, dir, previous_);
    auto [path, basis] = calculation.run(left_virtual, right_virtual, left_to_right, right_to_left, out.from_previous);
    for (const auto& p : basis) out.basis.push_back({p.x, p.y});
    out.path = path;
    previous_ = path;
    return out;
}

}  // namespace fd
