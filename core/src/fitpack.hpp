#pragma once

#include <array>
#include <vector>

// A port of the parametric smoothing curve fit of Paul Dierckx's FITPACK (parcur, fppara and the routines they call,
// from netlib's dierckx collection, as SciPy bundles it under the BSD-3-Clause licence), in double precision as SciPy
// compiles it, for the cone-to-path planner of decision 0031, whose reference, FaSTTUBe's path planner, fits its paths
// with scipy.interpolate.splprep and evaluates them with splev. See THIRD_PARTY_NOTICES.md.
namespace fd::fitpack {

// A fitted plane curve: knots, one coefficient array per coordinate, and its degree.
struct Curve {
    std::vector<double> knots;
    std::array<std::vector<double>, 2> coefficients;
    int degree{3};
    // How the fit ended, FITPACK's ier: 0 met the smoothing condition, -1 interpolates, -2 is the least-squares
    // polynomial, 1 to 3 are the warnings SciPy passes on and still returns the curve for.
    int status{};
};

// scipy.interpolate.splprep(points.T, u=u, k=degree, s=smoothing) with its defaults: unit weights, the parameter's
// ends as the curve's, and room for len(points) + 2 degree knots. Refuses, as SciPy raises ValueError, a parameter that
// does not increase strictly, fewer points than degree + 1, a degree outside 1..5 or a negative smoothing.
Curve fit_curve(const std::vector<double>& u, const std::vector<std::array<double, 2>>& points, int degree, double smoothing);

// scipy.interpolate.splev at one parameter value, extrapolating beyond the knots with the end polynomial pieces as
// splev's default does.
std::array<double, 2> evaluate(const Curve& curve, double u);

}  // namespace fd::fitpack
