// A port of Paul Dierckx's FITPACK routines parcur, fppara, fpback, fpbspl, fpdisc, fpgivs, fpknot, fprati, fprota and
// SciPy's extrapolating splev, from netlib's dierckx collection as SciPy bundles it under the BSD-3-Clause licence. The
// control flow keeps the Fortran's labels and one-based indices so that it can be read against the original line by
// line; see THIRD_PARTY_NOTICES.md.
#include "fitpack.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fd::fitpack {
namespace {

// A Fortran array a(rows, cols), indexed from one.
class Matrix {
public:
    Matrix(int rows, int cols) : cols_(cols), values_(static_cast<std::size_t>((rows+1)*(cols+1)), 0.0) {}
    double& operator()(int i, int j) { return values_[static_cast<std::size_t>(i*(cols_+1)+j)]; }
private:
    int cols_;
    std::vector<double> values_;
};

// Values of the k+1 B-splines of degree k that are not zero at x, t(l) <= x < t(l+1), into h(1..k+1).
void fpbspl(const std::vector<double>& t, int k, double x, int l, double* h) {
    double hh[8];
    h[1] = 1.0;
    for (int j = 1; j <= k; ++j) {
        for (int i = 1; i <= j; ++i) hh[i] = h[i];
        h[1] = 0.0;
        for (int i = 1; i <= j; ++i) {
            const int li = l+i, lj = li-j;
            const double f = hh[i]/(t[static_cast<std::size_t>(li)]-t[static_cast<std::size_t>(lj)]);
            h[i] = h[i]+f*(t[static_cast<std::size_t>(li)]-x);
            h[i+1] = f*(x-t[static_cast<std::size_t>(lj)]);
        }
    }
}

// The parameters of a Givens transformation.
void fpgivs(double piv, double& ww, double& cos, double& sin) {
    const double store = std::abs(piv);
    double dd = 0;
    if (store >= ww) dd = store*std::sqrt(1.0+(ww/piv)*(ww/piv));
    if (store < ww) dd = ww*std::sqrt(1.0+(piv/ww)*(piv/ww));
    cos = ww/dd;
    sin = piv/dd;
    ww = dd;
}

// Applies a Givens rotation to a and b.
void fprota(double cos, double sin, double& a, double& b) {
    const double stor1 = a, stor2 = b;
    b = cos*stor2+sin*stor1;
    a = cos*stor1-sin*stor2;
}

// Solves the upper triangular a c = z of bandwidth k; z and c are one-based views and may be the same array.
void fpback(Matrix& a, const double* z, int n, int k, double* c) {
    const int k1 = k-1;
    c[n] = z[n]/a(n, 1);
    int i = n-1;
    if (i == 0) return;
    for (int j = 2; j <= n; ++j) {
        double store = z[i];
        int i1 = k1;
        if (j <= k1) i1 = j-1;
        int m = i;
        for (int l = 1; l <= i1; ++l) {
            ++m;
            store = store-c[m]*a(i, l+1);
        }
        c[i] = store/a(i, 1);
        --i;
    }
}

// The discontinuity jumps of the k-th derivative of the B-splines of degree k at the interior knots.
void fpdisc(const std::vector<double>& t, int n, int k2, Matrix& b) {
    double h[13];
    const int k1 = k2-1, k = k1-1, nk1 = n-k1, nrint = nk1-k;
    const double an = nrint;
    const double fac = an/(t[static_cast<std::size_t>(nk1+1)]-t[static_cast<std::size_t>(k1)]);
    for (int l = k2; l <= nk1; ++l) {
        const int lmk = l-k1;
        for (int j = 1; j <= k1; ++j) {
            const int ik = j+k1, lj = l+j, lk = lj-k2;
            h[j] = t[static_cast<std::size_t>(l)]-t[static_cast<std::size_t>(lk)];
            h[ik] = t[static_cast<std::size_t>(l)]-t[static_cast<std::size_t>(lj)];
        }
        int lp = lmk;
        for (int j = 1; j <= k2; ++j) {
            int jk = j;
            double prod = h[j];
            for (int i = 1; i <= k; ++i) {
                ++jk;
                prod = prod*h[jk]*fac;
            }
            const int lk = lp+k1;
            b(lmk, j) = (t[static_cast<std::size_t>(lk)]-t[static_cast<std::size_t>(lp)])/prod;
            ++lp;
        }
    }
}

// Adds a knot in the knot interval with the largest share of the residual that still holds data points.
void fpknot(const std::vector<double>& x, std::vector<double>& t, int& n, std::vector<double>& fpint, std::vector<int>& nrdata,
            int& nrint, int istart) {
    const int k = (n-nrint-1)/2;
    double fpmax = 0;
    int jbegin = istart, number = 0, maxpt = 0, maxbeg = 0;
    for (int j = 1; j <= nrint; ++j) {
        const int jpoint = nrdata[static_cast<std::size_t>(j)];
        if (!(fpmax >= fpint[static_cast<std::size_t>(j)] || jpoint == 0)) {
            fpmax = fpint[static_cast<std::size_t>(j)];
            number = j;
            maxpt = jpoint;
            maxbeg = jbegin;
        }
        jbegin = jbegin+jpoint+1;
    }
    if (number == 0) throw std::runtime_error("FITPACK found no knot interval holding data to split");
    const int ihalf = maxpt/2+1;
    const int nrx = maxbeg+ihalf;
    const int next = number+1;
    if (next <= nrint)
        for (int j = next; j <= nrint; ++j) {
            const int jj = next+nrint-j;
            fpint[static_cast<std::size_t>(jj+1)] = fpint[static_cast<std::size_t>(jj)];
            nrdata[static_cast<std::size_t>(jj+1)] = nrdata[static_cast<std::size_t>(jj)];
            const int jk = jj+k;
            t[static_cast<std::size_t>(jk+1)] = t[static_cast<std::size_t>(jk)];
        }
    nrdata[static_cast<std::size_t>(number)] = ihalf-1;
    nrdata[static_cast<std::size_t>(next)] = maxpt-ihalf;
    const double am = maxpt;
    double an = nrdata[static_cast<std::size_t>(number)];
    fpint[static_cast<std::size_t>(number)] = fpmax*an/am;
    an = nrdata[static_cast<std::size_t>(next)];
    fpint[static_cast<std::size_t>(next)] = fpmax*an/am;
    const int jk = next+k;
    t[static_cast<std::size_t>(jk)] = x[static_cast<std::size_t>(nrx)];
    ++n;
    ++nrint;
}

// Rational interpolation of the smoothing parameter's effect, and the bracket it narrows.
double fprati(double& p1, double& f1, double p2, double f2, double& p3, double& f3) {
    double p;
    if (p3 > 0) {
        const double h1 = f1*(f2-f3), h2 = f2*(f3-f1), h3 = f3*(f1-f2);
        p = -(p1*p2*h3+p2*p3*h1+p3*p1*h2)/(p1*h1+p2*h2+p3*h3);
    } else {
        p = (p1*(f1-f3)*f2-p2*(f2-f3)*f1)/((f1-f2)*f3);
    }
    if (f2 < 0) {
        p3 = p2;
        f3 = f2;
    } else {
        p1 = p2;
        f1 = f2;
    }
    return p;
}

// fppara for iopt = 0, the smoothing curve of degree k through m points of idim coordinates, in the Fortran's own
// labels. Arrays are one-based: element zero is unused.
int fppara(int idim, int m, const std::vector<double>& u, const std::vector<double>& x, const std::vector<double>& w,
           double ub, double ue, int k, double s, int nest, double tol, int maxit, int& n, std::vector<double>& t,
           std::vector<double>& c, double& fp) {
    const int k1 = k+1, k2 = k1+1, nc = nest*idim;
    std::vector<double> fpint(static_cast<std::size_t>(nest+2), 0.0), z(static_cast<std::size_t>(nc+1), 0.0);
    std::vector<int> nrdata(static_cast<std::size_t>(nest+2), 0);
    Matrix a(nest, k1), b(nest, k2), g(nest, k2), q(m, k1);
    double h[8] = {}, xi[11] = {};
    const double one = 1.0, con1 = 0.1, con9 = 0.9, con4 = 0.04, half = 0.5;
    const int iopt = 0;
    int ier = 0;
    double acc = 0, cos = 0, sin = 0, fac = 0, fpart = 0, fpms = 0, fpold = 0, fp0 = 0, f1 = 0, f2 = 0, f3 = 0;
    double p = 0, pinv = 0, piv = 0, p1 = 0, p2 = 0, p3 = 0, rn = 0, store = 0, term = 0, ui = 0, wi = 0;
    int i = 0, ich1 = 0, ich3 = 0, iter = 0, i1 = 0, i2 = 0, i3 = 0, j = 0, jj = 0, j1 = 0, j2 = 0, k3 = 0, l = 0, l0 = 0;
    int mk1 = 0, newknot = 0, nk1 = 0, nmax = 0, nplus = 0, npl1 = 0, nrint = 0, n8 = 0;
    const int nmin = 2*k1;
    const auto T = [&](int index) -> double& { return t[static_cast<std::size_t>(index)]; };
    const auto U = [&](int index) { return u[static_cast<std::size_t>(index)]; };
    const auto C = [&](int index) -> double& { return c[static_cast<std::size_t>(index)]; };
    const auto Z = [&](int index) -> double& { return z[static_cast<std::size_t>(index)]; };
    const auto X = [&](int index) { return x[static_cast<std::size_t>(index)]; };
    const auto W = [&](int index) { return w[static_cast<std::size_t>(index)]; };
    const auto FPINT = [&](int index) -> double& { return fpint[static_cast<std::size_t>(index)]; };
    const auto NRDATA = [&](int index) -> int& { return nrdata[static_cast<std::size_t>(index)]; };

    acc = tol*s;
    nmax = m+k1;
    if (s > 0) goto L45;
    n = nmax;
    if (nmax > nest) goto L420;
L10:
    mk1 = m-k1;
    if (mk1 == 0) goto L60;
    k3 = k/2;
    i = k2;
    j = k3+2;
    if (k3*2 == k) goto L30;
    for (l = 1; l <= mk1; ++l) {
        T(i) = U(j);
        ++i;
        ++j;
    }
    goto L60;
L30:
    for (l = 1; l <= mk1; ++l) {
        T(i) = (U(j)+U(j-1))*half;
        ++i;
        ++j;
    }
    goto L60;
L45:
    // iopt = 0: a fresh fit starts from the least-squares polynomial.
    n = nmin;
    fpold = 0;
    nplus = 0;
    NRDATA(1) = m-2;
L60:
    for (iter = 1; iter <= m; ++iter) {
        if (n == nmin) ier = -2;
        nrint = n-nmin+1;
        nk1 = n-k1;
        i = n;
        for (j = 1; j <= k1; ++j) {
            T(j) = ub;
            T(i) = ue;
            --i;
        }
        fp = 0;
        for (i = 1; i <= nc; ++i) Z(i) = 0;
        for (i = 1; i <= nk1; ++i)
            for (j = 1; j <= k1; ++j) a(i, j) = 0;
        l = k1;
        jj = 0;
        for (int it = 1; it <= m; ++it) {
            ui = U(it);
            wi = W(it);
            for (j = 1; j <= idim; ++j) {
                ++jj;
                xi[j] = X(jj)*wi;
            }
            while (!(ui < T(l+1) || l == nk1)) ++l;
            fpbspl(t, k, ui, l, h);
            for (i = 1; i <= k1; ++i) {
                q(it, i) = h[i];
                h[i] = h[i]*wi;
            }
            j = l-k1;
            for (i = 1; i <= k1; ++i) {
                ++j;
                piv = h[i];
                if (piv == 0) continue;
                fpgivs(piv, a(j, 1), cos, sin);
                j1 = j;
                for (j2 = 1; j2 <= idim; ++j2) {
                    fprota(cos, sin, xi[j2], Z(j1));
                    j1 = j1+n;
                }
                if (i == k1) break;
                i2 = 1;
                i3 = i+1;
                for (i1 = i3; i1 <= k1; ++i1) {
                    ++i2;
                    fprota(cos, sin, h[i1], a(j, i2));
                }
            }
            for (j2 = 1; j2 <= idim; ++j2) fp = fp+xi[j2]*xi[j2];
        }
        if (ier == -2) fp0 = fp;
        FPINT(n) = fp0;
        FPINT(n-1) = fpold;
        NRDATA(n) = nplus;
        j1 = 1;
        for (j2 = 1; j2 <= idim; ++j2) {
            fpback(a, &Z(j1)-1, nk1, k1, &C(j1)-1);
            j1 = j1+n;
        }
        fpms = fp-s;
        if (std::abs(fpms) < acc) goto L440;
        if (fpms < 0) goto L250;
        if (n == nmax) goto L430;
        if (n == nest) goto L420;
        if (ier == 0) goto L140;
        nplus = 1;
        ier = 0;
        goto L150;
    L140:
        npl1 = nplus*2;
        rn = nplus;
        if (fpold-fp > acc) npl1 = static_cast<int>(rn*fpms/(fpold-fp));
        nplus = std::min(nplus*2, std::max(std::max(npl1, nplus/2), 1));
    L150:
        fpold = fp;
        fpart = 0;
        i = 1;
        l = k2;
        newknot = 0;
        jj = 0;
        for (int it = 1; it <= m; ++it) {
            if (!(U(it) < T(l) || l > nk1)) {
                newknot = 1;
                ++l;
            }
            term = 0;
            l0 = l-k2;
            for (j2 = 1; j2 <= idim; ++j2) {
                fac = 0;
                j1 = l0;
                for (j = 1; j <= k1; ++j) {
                    ++j1;
                    fac = fac+C(j1)*q(it, j);
                }
                ++jj;
                term = term+(W(it)*(fac-X(jj)))*(W(it)*(fac-X(jj)));
                l0 = l0+n;
            }
            fpart = fpart+term;
            if (newknot == 0) continue;
            store = term*half;
            FPINT(i) = fpart-store;
            ++i;
            fpart = store;
            newknot = 0;
        }
        FPINT(nrint) = fpart;
        for (l = 1; l <= nplus; ++l) {
            fpknot(u, t, n, fpint, nrdata, nrint, 1);
            if (n == nmax) goto L10;
            if (n == nest) break;
        }
    }
L250:
    if (ier == -2) goto L440;
    fpdisc(t, n, k2, b);
    p1 = 0;
    f1 = fp0-s;
    p3 = -one;
    f3 = fpms;
    p = 0;
    for (i = 1; i <= nk1; ++i) p = p+a(i, 1);
    rn = nk1;
    p = rn/p;
    ich1 = 0;
    ich3 = 0;
    n8 = n-nmin;
    for (iter = 1; iter <= maxit; ++iter) {
        pinv = one/p;
        for (i = 1; i <= nc; ++i) C(i) = Z(i);
        for (i = 1; i <= nk1; ++i) {
            g(i, k2) = 0;
            for (j = 1; j <= k1; ++j) g(i, j) = a(i, j);
        }
        for (int it = 1; it <= n8; ++it) {
            for (i = 1; i <= k2; ++i) h[i] = b(it, i)*pinv;
            for (j = 1; j <= idim; ++j) xi[j] = 0;
            for (j = it; j <= nk1; ++j) {
                piv = h[1];
                fpgivs(piv, g(j, 1), cos, sin);
                j1 = j;
                for (j2 = 1; j2 <= idim; ++j2) {
                    fprota(cos, sin, xi[j2], C(j1));
                    j1 = j1+n;
                }
                if (j == nk1) break;
                i2 = k1;
                if (j > n8) i2 = nk1-j;
                for (i = 1; i <= i2; ++i) {
                    i1 = i+1;
                    fprota(cos, sin, h[i1], g(j, i1));
                    h[i] = h[i1];
                }
                h[i2+1] = 0;
            }
        }
        j1 = 1;
        for (j2 = 1; j2 <= idim; ++j2) {
            fpback(g, &C(j1)-1, nk1, k2, &C(j1)-1);
            j1 = j1+n;
        }
        fp = 0;
        l = k2;
        jj = 0;
        for (int it = 1; it <= m; ++it) {
            if (!(U(it) < T(l) || l > nk1)) ++l;
            l0 = l-k2;
            term = 0;
            for (j2 = 1; j2 <= idim; ++j2) {
                fac = 0;
                j1 = l0;
                for (j = 1; j <= k1; ++j) {
                    ++j1;
                    fac = fac+C(j1)*q(it, j);
                }
                ++jj;
                term = term+(fac-X(jj))*(fac-X(jj));
                l0 = l0+n;
            }
            fp = fp+term*W(it)*W(it);
        }
        fpms = fp-s;
        if (std::abs(fpms) < acc) goto L440;
        if (iter == maxit) goto L400;
        p2 = p;
        f2 = fpms;
        if (ich3 != 0) goto L340;
        if ((f2-f3) > acc) goto L335;
        p3 = p2;
        f3 = f2;
        p = p*con4;
        if (p <= p1) p = p1*con9+p2*con1;
        continue;
    L335:
        if (f2 < 0) ich3 = 1;
    L340:
        if (ich1 != 0) goto L350;
        if ((f1-f2) > acc) goto L345;
        p1 = p2;
        f1 = f2;
        p = p/con4;
        if (p3 < 0) continue;
        if (p >= p3) p = p2*con1+p3*con9;
        continue;
    L345:
        if (f2 > 0) ich1 = 1;
    L350:
        if (f2 >= f1 || f2 <= f3) goto L410;
        p = fprati(p1, f1, p2, f2, p3, f3);
    }
L400:
    ier = 3;
    goto L440;
L410:
    ier = 2;
    goto L440;
L420:
    ier = 1;
    goto L440;
L430:
    ier = -1;
L440:
    (void)iopt;
    return ier;
}

}  // namespace

Curve fit_curve(const std::vector<double>& u, const std::vector<std::array<double, 2>>& points, int degree, double smoothing) {
    const int m = static_cast<int>(points.size());
    const int k = degree, k1 = k+1, idim = 2;
    if (k < 1 || k > 5) throw std::invalid_argument("FITPACK fits curves of degree 1 to 5");
    if (static_cast<int>(u.size()) != m || m < k1) throw std::invalid_argument("FITPACK needs at least degree + 1 points, each with a parameter");
    if (!std::isfinite(smoothing) || smoothing < 0) throw std::invalid_argument("FITPACK's smoothing must not be negative");
    for (int i = 1; i < m; ++i)
        if (!(u[static_cast<std::size_t>(i-1)] < u[static_cast<std::size_t>(i)]))
            throw std::invalid_argument("FITPACK needs a parameter that increases strictly");
    const int nest = m+2*k;
    if (smoothing == 0 && nest < m+k1) throw std::invalid_argument("FITPACK has no room to interpolate");
    std::vector<double> uu(static_cast<std::size_t>(m+1)), xx(static_cast<std::size_t>(m*idim+1)), ww(static_cast<std::size_t>(m+1), 1.0);
    for (int i = 1; i <= m; ++i) {
        uu[static_cast<std::size_t>(i)] = u[static_cast<std::size_t>(i-1)];
        xx[static_cast<std::size_t>(2*i-1)] = points[static_cast<std::size_t>(i-1)][0];
        xx[static_cast<std::size_t>(2*i)] = points[static_cast<std::size_t>(i-1)][1];
    }
    std::vector<double> t(static_cast<std::size_t>(nest+2), 0.0), c(static_cast<std::size_t>(nest*idim+1), 0.0);
    int n = 0;
    double fp = 0;
    Curve curve;
    curve.degree = k;
    curve.status = fppara(idim, m, uu, xx, ww, u.front(), u.back(), k, smoothing, nest, 0.001, 20, n, t, c, fp);
    curve.knots.assign(t.begin()+1, t.begin()+1+n);
    for (int d = 0; d < idim; ++d)
        curve.coefficients[static_cast<std::size_t>(d)].assign(c.begin()+1+d*n, c.begin()+1+d*n+n);
    return curve;
}

std::array<double, 2> evaluate(const Curve& curve, double u) {
    const int n = static_cast<int>(curve.knots.size()), k = curve.degree, k1 = k+1, nk1 = n-k1;
    // One-based knots, as fpbspl indexes them.
    std::vector<double> t(static_cast<std::size_t>(n+1));
    std::copy(curve.knots.begin(), curve.knots.end(), t.begin()+1);
    int l = k1;
    while (!(u < t[static_cast<std::size_t>(l+1)] || l == nk1)) ++l;
    double h[8] = {};
    fpbspl(t, k, u, l, h);
    std::array<double, 2> value{};
    for (std::size_t d = 0; d < 2; ++d) {
        double sp = 0;
        int ll = l-k1;
        for (int j = 1; j <= k1; ++j) {
            ++ll;
            sp = sp+curve.coefficients[d][static_cast<std::size_t>(ll-1)]*h[j];
        }
        value[d] = sp;
    }
    return value;
}

}  // namespace fd::fitpack
