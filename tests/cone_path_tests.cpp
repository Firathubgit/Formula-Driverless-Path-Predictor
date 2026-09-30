#include "../core/src/fitpack.hpp"
#include "fd/cone_path.hpp"
#include "fd/delaunay_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::filesystem::path fixtures() { return std::filesystem::path(FD_FIXTURE_DIR); }

std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) out.push_back(field);
    return out;
}

// FITPACK's parametric smoothing fit, ported line for line, gives SciPy's knots and SciPy's curve, inside the data
// and extrapolated beyond it, on thirty curves of two to forty points at three smoothings.
void the_fitpack_port_is_scipy_s_fit() {
    std::ifstream file(fixtures()/"fitpack"/"cases.csv");
    require(file.good(), "the FITPACK fixtures are present");
    struct Case {
        std::string name;
        int degree{};
        double smoothing{};
        std::vector<std::array<double, 2>> points;
        std::vector<double> knots;
        std::vector<std::array<double, 3>> evaluations;
    };
    std::vector<Case> cases;
    std::string line;
    while (std::getline(file, line)) {
        const auto f = fields(line);
        if (f.empty()) continue;
        if (f[0] == "case") cases.push_back({f[1], std::stoi(f[2]), std::stod(f[3]), {}, {}, {}});
        else if (f[0] == "point") cases.back().points.push_back({std::stod(f[1]), std::stod(f[2])});
        else if (f[0] == "knot") cases.back().knots.push_back(std::stod(f[1]));
        else if (f[0] == "eval") cases.back().evaluations.push_back({std::stod(f[1]), std::stod(f[2]), std::stod(f[3])});
    }
    require(cases.size() == 30, "thirty cases");
    double worst_knot = 0, worst_value = 0;
    for (const auto& c : cases) {
        std::vector<double> u{0.0};
        for (std::size_t i = 1; i < c.points.size(); ++i)
            u.push_back(u.back()+std::hypot(c.points[i][0]-c.points[i-1][0], c.points[i][1]-c.points[i-1][1]));
        const auto curve = fd::fitpack::fit_curve(u, c.points, c.degree, c.smoothing);
        require(curve.knots.size() == c.knots.size(),
                c.name+": as many knots as SciPy's, "+std::to_string(curve.knots.size())+" against "+std::to_string(c.knots.size()));
        for (std::size_t i = 0; i < c.knots.size(); ++i) worst_knot = std::max(worst_knot, std::abs(curve.knots[i]-c.knots[i]));
        for (const auto& [at, x, y] : c.evaluations) {
            const auto v = fd::fitpack::evaluate(curve, at);
            worst_value = std::max(worst_value, std::max(std::abs(v[0]-x), std::abs(v[1]-y)));
        }
    }
    std::cout << "  over " << cases.size() << " SciPy fits: knots within " << worst_knot << ", curve within " << worst_value
              << " m, extrapolation included\n";
    require(worst_knot < 1e-9, "the knots are SciPy's");
    require(worst_value < 1e-8, "and so is the curve");
    // What SciPy refuses, the port refuses.
    const auto refused = [](const std::function<void()>& fit) {
        try { fit(); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    require(refused([] { fd::fitpack::fit_curve({0, 1, 1}, {{0, 0}, {1, 0}, {1, 0}}, 2, 0.2); }),
            "a parameter that does not increase is refused");
    require(refused([] { fd::fitpack::fit_curve({0, 1}, {{0, 0}, {1, 0}}, 3, 0.2); }), "too few points for the degree are refused");
}

struct Fixture {
    std::string name, description;
    fd::Vec2 position, direction;
    std::vector<fd::TypedCone> cones;
    std::vector<fd::Vec2> left, right, left_virtual, right_virtual, basis;
    std::vector<int> left_to_right, right_to_left;
    std::vector<fd::ConePathPoint> path;
    std::vector<fd::Vec2> centre;  // the course's true centreline around the car: for evaluation, never for planning
};

Fixture read_fixture(const std::filesystem::path& file) {
    Fixture f;
    f.name = file.stem().string();
    std::ifstream in(file);
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("# ", 0) == 0) { f.description = line.substr(2); continue; }
        const auto v = fields(line);
        if (v.empty()) continue;
        const auto point = [&] { return fd::Vec2{std::stod(v[1]), std::stod(v[2])}; };
        if (v[0] == "pose") { f.position = {std::stod(v[1]), std::stod(v[2])}; f.direction = {std::stod(v[3]), std::stod(v[4])}; }
        else if (v[0] == "cone") f.cones.push_back({{std::stod(v[2]), std::stod(v[3])}, static_cast<fd::ConeType>(std::stoi(v[1]))});
        else if (v[0] == "left") f.left.push_back(point());
        else if (v[0] == "right") f.right.push_back(point());
        else if (v[0] == "left_virtual") f.left_virtual.push_back(point());
        else if (v[0] == "right_virtual") f.right_virtual.push_back(point());
        else if (v[0] == "basis") f.basis.push_back(point());
        else if (v[0] == "left_to_right") f.left_to_right.push_back(std::stoi(v[1]));
        else if (v[0] == "right_to_left") f.right_to_left.push_back(std::stoi(v[1]));
        else if (v[0] == "path") f.path.push_back({std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[4])});
        else if (v[0] == "centre") f.centre.push_back(point());
    }
    return f;
}

double worst(const std::vector<fd::Vec2>& a, const std::vector<fd::Vec2>& b, const std::string& what) {
    require(a.size() == b.size(), what+": "+std::to_string(a.size())+" points against FaSTTUBe's "+std::to_string(b.size()));
    double w = 0;
    for (std::size_t i = 0; i < a.size(); ++i) w = std::max(w, std::hypot(a[i].x-b[i].x, a[i].y-b[i].y));
    return w;
}

// On 44 cone sets taken from a recorded lap of the 5 m course, each as the course laid it, without colour, as the lap's
// own simulated detections saw it, and with only its right-hand side, the port sorts, matches and places virtual cones
// exactly as FaSTTUBe's published package does, fits its path through the same points, and samples the same path.
void the_port_reproduces_fasttube_on_recorded_cones() {
    std::vector<std::filesystem::path> files, failed;
    for (const auto& entry : std::filesystem::directory_iterator(fixtures()/"cone_path")) {
        if (entry.path().extension() == ".csv") files.push_back(entry.path());
        if (entry.path().extension() == ".failed") failed.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    require(files.size() == 44, "forty-four reference cone sets, got "+std::to_string(files.size()));
    double worst_cone = 0, worst_basis = 0, worst_path = 0, worst_parameter = 0, worst_curvature = 0, worst_off_curve = 0;
    std::size_t sorted_any = 0, one_sided = 0, knife_edges = 0, straight_windows = 0;
    double worst_straight = 0;
    for (const auto& file : files) {
        const auto f = read_fixture(file);
        fd::ConePathPlanner planner;
        const auto got = planner.plan(f.cones, f.position, f.direction);
        worst_cone = std::max({worst_cone, worst(got.left, f.left, f.name+" left"), worst(got.right, f.right, f.name+" right"),
                               worst(got.left_with_virtual, f.left_virtual, f.name+" left with virtual cones"),
                               worst(got.right_with_virtual, f.right_virtual, f.name+" right with virtual cones")});
        require(got.left_to_right == f.left_to_right && got.right_to_left == f.right_to_left, f.name+": the matches are FaSTTUBe's");
        worst_basis = std::max(worst_basis, worst(got.basis, f.basis, f.name+" basis"));
        require(got.path.size() == f.path.size(), f.name+": as many path samples");
        double here = 0;
        for (std::size_t i = 0; i < f.path.size(); ++i)
            here = std::max(here, std::hypot(got.path[i].x_m-f.path[i].x_m, got.path[i].y_m-f.path[i].y_m));
        if (here > 1e-6) {
            // The one step that turns on a last bit: FaSTTUBe samples its fitted path every length/120 up to that length,
            // so the count is 120 or 121 as the length's rounding falls, and the 40 samples taken from it move by up to
            // one spacing. Then every sample of the port still lies on FaSTTUBe's own path.
            ++knife_edges;
            std::cout << "    " << f.name << ": sampled a step apart, by up to " << here << " m\n";
            for (const auto& g : got.path) {
                double nearest = 1e9;
                for (std::size_t i = 1; i < f.path.size(); ++i) {
                    const double ax = f.path[i-1].x_m, ay = f.path[i-1].y_m, bx = f.path[i].x_m, by = f.path[i].y_m;
                    const double dx = bx-ax, dy = by-ay, l2 = dx*dx+dy*dy;
                    const double s = l2 > 0 ? std::clamp(((g.x_m-ax)*dx+(g.y_m-ay)*dy)/l2, 0.0, 1.0) : 0.0;
                    nearest = std::min(nearest, std::hypot(g.x_m-(ax+s*dx), g.y_m-(ay+s*dy)));
                }
                worst_off_curve = std::max(worst_off_curve, nearest);
            }
            require(here < 0.2, f.name+": no more than one sample spacing apart");
            continue;
        }
        for (std::size_t i = 0; i < f.path.size(); ++i) {
            worst_path = std::max(worst_path, std::hypot(got.path[i].x_m-f.path[i].x_m, got.path[i].y_m-f.path[i].y_m));
            worst_parameter = std::max(worst_parameter, std::abs(got.path[i].parameter_m-f.path[i].parameter_m));
            // On a straight, FaSTTUBe clamps each window's radius at 3000 m and signs it by the determinant of three of
            // its points, which is then a rounding error from zero, and averages twelve windows: there the two can
            // differ by whole windows of 1/3000 1/m taking the other sign, and by nothing else.
            const double dk = std::abs(got.path[i].curvature_1pm-f.path[i].curvature_1pm);
            const double quantum = (1.0/3000)/12;
            if (dk > 1e-6) {
                const double windows = dk/quantum;
                require(std::abs(windows-std::round(windows)) < 1e-3 && windows <= 24,
                        f.name+": curvature differs only by straight windows taking the other sign");
                ++straight_windows;
                worst_straight = std::max(worst_straight, dk);
                continue;
            }
            worst_curvature = std::max(worst_curvature, dk);
        }
        if (!f.left.empty() || !f.right.empty()) ++sorted_any;
        if (f.left.empty() != f.right.empty()) ++one_sided;
    }
    std::cout << "  " << files.size() << " cone sets, " << sorted_any << " of the exact ones sorted by FaSTTUBe (" << one_sided
              << " one-sided): cones within " << worst_cone << " m, fitted points within " << worst_basis << " m; path within "
              << worst_path << " m, its parameter within " << worst_parameter << " m and curvature within " << worst_curvature
              << " 1/m elsewhere, " << straight_windows << " samples on straights differing by up to " << worst_straight
              << " 1/m in whole windows of 1/36000; " << knife_edges << " sampled a step apart, on FaSTTUBe's path within "
              << worst_off_curve << " m\n";
    require(worst_cone < 1e-9 && worst_basis < 1e-9, "every cone, virtual cone and fitted point is FaSTTUBe's");
    require(worst_path < 1e-6 && worst_parameter < 1e-6 && worst_curvature < 1e-6, "and so is the path, to a micrometre");
    require(knife_edges <= 1 && worst_off_curve < 0.01, "at most one set sampled a step apart, and that on the same path");
    // Where FaSTTUBe's own code divided by zero, on cones along the perfectly straight main straight, the port plans a
    // straight path ahead.
    require(failed.size() == 4, "FaSTTUBe failed on four cone sets");
    for (const auto& file : failed) {
        const auto f = read_fixture(file);
        fd::ConePathPlanner planner;
        const auto got = planner.plan(f.cones, f.position, f.direction);
        require(got.path.size() == 40, f.name+": the port plans a path where FaSTTUBe raised");
        for (const auto& p : got.path) {
            require(std::abs(p.curvature_1pm) < 1e-3, f.name+": a straight path along a straight");
            require(std::abs(p.y_m) < 0.5 && p.x_m > -0.1, f.name+": ahead of the car, down the middle");
        }
    }
}

// Bowyer-Watson gives a Delaunay triangulation: no point lies inside any triangle's circumcircle, every triangle turns
// counterclockwise, and there are 2n - 2 - h triangles for n points of which h lie on the hull.
void the_triangulation_is_delaunay() {
    std::vector<fd::Vec2> points;
    std::uint64_t state = 12345;
    const auto random = [&] { state = state*6364136223846793005ULL+1442695040888963407ULL; return static_cast<double>(state >> 11)/9007199254740992.0; };
    for (int i = 0; i < 60; ++i) points.push_back({random()*40, random()*20});
    const auto triangles = fd::delaunay_triangles(points);
    for (const auto& t : triangles) {
        const auto a = points[static_cast<std::size_t>(t[0])], b = points[static_cast<std::size_t>(t[1])], c = points[static_cast<std::size_t>(t[2])];
        require((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x) > 0, "every triangle is counterclockwise");
        // The in-circle determinant: positive when a point lies inside the circle through a, b and c.
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (static_cast<int>(i) == t[0] || static_cast<int>(i) == t[1] || static_cast<int>(i) == t[2]) continue;
            const auto d = points[i];
            const double ax = a.x-d.x, ay = a.y-d.y, bx = b.x-d.x, by = b.y-d.y, cx = c.x-d.x, cy = c.y-d.y;
            const double det = (ax*ax+ay*ay)*(bx*cy-cx*by)-(bx*bx+by*by)*(ax*cy-cx*ay)+(cx*cx+cy*cy)*(ax*by-bx*ay);
            require(det <= 1e-9, "no point lies inside a triangle's circumcircle");
        }
    }
    // Hull points by gift wrapping, for the count.
    std::size_t hull = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        bool on_hull = false;
        for (std::size_t j = 0; j < points.size() && !on_hull; ++j) {
            if (j == i) continue;
            bool all_left = true;
            for (std::size_t k = 0; k < points.size() && all_left; ++k)
                if (k != i && k != j)
                    all_left = (points[j].x-points[i].x)*(points[k].y-points[i].y)-(points[j].y-points[i].y)*(points[k].x-points[i].x) > 0;
            on_hull = all_left;
        }
        if (on_hull) ++hull;
    }
    require(triangles.size() == 2*points.size()-2-hull, "2n - 2 - h triangles");
    require(fd::delaunay_triangles(std::vector<fd::Vec2>{{0, 0}, {1, 0}}).empty(), "two points make no triangle");
}

// The farthest a path's first metres, fifteen unless given, lie from a polyline: the true centreline, or the other
// method's path. Samples of either lie well within 1.5 m of the next.
double furthest_from(const std::vector<fd::ConePathPoint>& path, const std::vector<fd::Vec2>& line, double over_m = 15) {
    double furthest = 0;
    for (const auto& p : path) {
        if (p.parameter_m > over_m) break;
        double nearest = 1e9;
        for (std::size_t i = 1; i < line.size(); ++i) {
            const auto a = line[i-1], b = line[i];
            const double dx = b.x-a.x, dy = b.y-a.y, l2 = dx*dx+dy*dy;
            if (l2 > 1.5*1.5) continue;
            const double s = l2 > 0 ? std::clamp(((p.x_m-a.x)*dx+(p.y_m-a.y)*dy)/l2, 0.0, 1.0) : 0.0;
            nearest = std::min(nearest, std::hypot(p.x_m-(a.x+s*dx), p.y_m-(a.y+s*dy)));
        }
        furthest = std::max(furthest, nearest);
    }
    return furthest;
}

std::vector<fd::Vec2> points_of(const std::vector<fd::ConePathPoint>& path) {
    std::vector<fd::Vec2> points;
    for (const auto& p : path) points.push_back({p.x_m, p.y_m});
    return points;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n/2] : (values[n/2-1]+values[n/2])/2;
}

// The Delaunay midpoint planner, written from FS-FEUP's description alone, cross-checks the FaSTTUBe port on the same
// cone sets. Agreement alone would not say which is right, so both are measured against the course's true centreline,
// which neither is given: over the first fifteen metres, how far each strays from it. Where the two part, the cone set
// is a good test, and the centreline says which method it caught out.
void a_second_method_cross_checks_the_port() {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(fixtures()/"cone_path"))
        if (entry.path().extension() == ".csv") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    std::size_t compared = 0, parted = 0, none = 0, port_closer = 0;
    std::vector<double> port_errors, delaunay_errors, course_port, course_delaunay;
    for (const auto& file : files) {
        const auto f = read_fixture(file);
        require(f.centre.size() > 50, f.name+": the fixture carries the true centreline around the car");
        fd::ConePathPlanner planner;
        const auto port = planner.plan(f.cones, f.position, f.direction);
        if (port.from_previous) continue;  // FaSTTUBe made no path from these cones
        const auto delaunay = fd::plan_delaunay_path(f.cones, f.position, f.direction);
        if (delaunay.path.empty()) { ++none; std::cout << "    " << f.name << ": the Delaunay walk found no start\n"; continue; }
        ++compared;
        const double port_error = furthest_from(port.path, f.centre), delaunay_error = furthest_from(delaunay.path, f.centre);
        port_errors.push_back(port_error);
        delaunay_errors.push_back(delaunay_error);
        const bool from_course = f.name.find("-course") != std::string::npos;
        if (from_course) { course_port.push_back(port_error); course_delaunay.push_back(delaunay_error); }
        // Apart over the length both paths reach: a Delaunay walk ends where its midpoints do, short of FaSTTUBe's.
        const double common = std::min({15.0, port.path.back().parameter_m, delaunay.path.back().parameter_m});
        const double apart = std::max(furthest_from(delaunay.path, points_of(port.path), common),
                                      furthest_from(port.path, points_of(delaunay.path), common));
        if (apart < 0.5) continue;
        ++parted;
        if (port_error < delaunay_error) ++port_closer;
        std::cout << "    " << f.name << ": the paths part by " << apart << " m over their first " << common
                  << " m; from the centreline the port strays " << port_error << " m, Delaunay " << delaunay_error << " m\n";
    }
    std::cout << "  " << compared << " cone sets both methods planned from (" << none << " where the Delaunay walk found no start): "
              << "the port strays a median " << median(port_errors) << " m from the true centreline over 15 m, at most "
              << *std::max_element(port_errors.begin(), port_errors.end()) << " m, Delaunay a median " << median(delaunay_errors)
              << " m, at most " << *std::max_element(delaunay_errors.begin(), delaunay_errors.end()) << " m; on the course's own cones "
              << median(course_port) << " m and " << median(course_delaunay) << " m; " << parted << " sets part them by 0.5 m or more, "
              << port_closer << " of them with the port the closer\n";
    require(compared >= 20, "the two methods are compared on most cone sets");
    // What the cross-check found: on a frame's own detections, where one side's cones go unseen, FaSTTUBe places each
    // virtual cone its minimum track width of 3 m across, so on this 5 m course its path runs a metre off the middle.
    // Told the course's own width, the port keeps to the middle there too.
    std::vector<double> detected_default, detected_told;
    for (const auto& file : files) {
        const auto f = read_fixture(file);
        if (f.name.find("-detected") == std::string::npos) continue;
        fd::ConePathPlanner planner, told({.min_track_width_m = 5.0});
        const auto by_default = planner.plan(f.cones, f.position, f.direction);
        const auto with_width = told.plan(f.cones, f.position, f.direction);
        if (by_default.from_previous || with_width.from_previous) continue;
        detected_default.push_back(furthest_from(by_default.path, f.centre));
        detected_told.push_back(furthest_from(with_width.path, f.centre));
    }
    std::cout << "  on " << detected_default.size() << " frames of detections the port strays a median " << median(detected_default)
              << " m with FaSTTUBe's 3 m virtual track width and " << median(detected_told) << " m told the course's 5 m\n";
    require(median(detected_told) < median(detected_default)/2, "the virtual cones' width is what moves the port off the middle");
    // Cones 2.5 m from a centreline that bends: both methods stay inside the 5 m course on the cones it laid.
    require(*std::max_element(course_port.begin(), course_port.end()) < 1.0, "the port keeps within a metre of the centreline on the course's cones");
    require(median(course_delaunay) < 0.5, "and Delaunay's median is within half a metre there");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"the_fitpack_port_is_scipy_s_fit", the_fitpack_port_is_scipy_s_fit},
        {"the_port_reproduces_fasttube_on_recorded_cones", the_port_reproduces_fasttube_on_recorded_cones},
        {"the_triangulation_is_delaunay", the_triangulation_is_delaunay},
        {"a_second_method_cross_checks_the_port", a_second_method_cross_checks_the_port}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " cone path groups passed\n";
    return failures ? 1 : 0;
}
