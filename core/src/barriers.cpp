#include "fd/barriers.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fd {
namespace {
Vec2 add(Vec2 a, Vec2 b) { return {a.x+b.x,a.y+b.y}; }
Vec2 sub(Vec2 a, Vec2 b) { return {a.x-b.x,a.y-b.y}; }
Vec2 mul(Vec2 a, double b) { return {a.x*b,a.y*b}; }
double dot(Vec2 a, Vec2 b) { return a.x*b.x+a.y*b.y; }
double cross(Vec2 a, Vec2 b) { return a.x*b.y-a.y*b.x; }
double length(Vec2 a) { return std::hypot(a.x,a.y); }
Vec2 closest(Vec2 p, BarrierSegment s) {
    const auto d=sub(s.b,s.a);
    return add(s.a,mul(d,std::clamp(dot(sub(p,s.a),d)/dot(d,d),0.0,1.0)));
}
BarrierSegment bumper(const State& s, const Config& c) {
    return {{s.x_m,s.y_m},{s.x_m+(c.wheelbase_m-0.08)*std::cos(s.yaw_rad),
                           s.y_m+(c.wheelbase_m-0.08)*std::sin(s.yaw_rad)}};
}
struct Contact { Vec2 body, wall; double distance{std::numeric_limits<double>::infinity()}; };
Contact contact(BarrierSegment body, BarrierSegment wall) {
    Contact best;
    const auto consider=[&](Vec2 a, Vec2 b) {
        const double d=length(sub(a,b));
        if (d<best.distance) best={a,b,d};
    };
    consider(body.a,closest(body.a,wall)); consider(body.b,closest(body.b,wall));
    consider(closest(wall.a,body),wall.a); consider(closest(wall.b,body),wall.b);
    const auto a=sub(body.b,body.a), b=sub(wall.b,wall.a), offset=sub(wall.a,body.a);
    const double det=cross(a,b);
    if (std::abs(det)>1e-12) {
        const double t=cross(offset,b)/det, u=cross(offset,a)/det;
        if (t>=0 && t<=1 && u>=0 && u<=1) { const auto p=add(body.a,mul(a,t)); best={p,p,0}; }
    }
    return best;
}
void impulse(PlantState& s, const FourWheelCar& car, Vec2 point, Vec2 normal) {
    const Vec2 forward{std::cos(s.pose.yaw_rad),std::sin(s.pose.yaw_rad)}, left{-forward.y,forward.x};
    const Vec2 cg=add({s.pose.x_m,s.pose.y_m},mul(forward,car.cg_to_rear_m));
    const Vec2 r=sub(point,cg);
    Vec2 velocity=add(mul(forward,s.pose.speed_mps),mul(left,s.lateral_velocity_mps+car.cg_to_rear_m*s.yaw_rate_radps));
    double yaw=s.yaw_rate_radps;
    const auto at_point=[&] { return add(velocity,Vec2{-yaw*r.y,yaw*r.x}); };
    const auto apply=[&](Vec2 direction,double amount) {
        velocity=add(velocity,mul(direction,amount/car.mass_kg));
        yaw+=cross(r,direction)*amount/car.yaw_inertia_kgm2;
    };
    const double vn=dot(at_point(),normal);
    if (vn>=0) return;
    const double rn=cross(r,normal);
    // Inelastic rubber barrier, with bounded tangential friction. Neither impulse adds energy.
    const double j=-vn/(1/car.mass_kg+rn*rn/car.yaw_inertia_kgm2);
    apply(normal,j);
    const Vec2 tangent{-normal.y,normal.x};
    const double rt=cross(r,tangent);
    apply(tangent,std::clamp(-dot(at_point(),tangent)/(1/car.mass_kg+rt*rt/car.yaw_inertia_kgm2),-0.25*j,0.25*j));
    s.pose.speed_mps=std::max(0.0,dot(velocity,forward)); // plant has no reverse gear
    s.yaw_rate_radps=yaw;
    s.lateral_velocity_mps=dot(velocity,left)-car.cg_to_rear_m*yaw;
}
} // namespace

KartEnvironment::KartEnvironment(std::vector<BarrierSegment> walls)
    : walls_(std::move(walls)) {
    for (const auto& w:walls_)
        if (!std::isfinite(w.a.x) || !std::isfinite(w.a.y) || !std::isfinite(w.b.x) || !std::isfinite(w.b.y) ||
            length(sub(w.b,w.a))<1e-6)
            throw std::invalid_argument("Barrier segments must be finite and have positive length");
}

std::vector<BarrierSegment> track_barriers(const Track& track) {
    validate_track(track);
    const auto n=track.points.size();
    std::vector<Vec2> left(n),right(n);
    for (std::size_t i=0;i<n;++i) {
        const auto& before=track.points[(i+n-1)%n]; const auto& after=track.points[(i+1)%n];
        const double dx=after.x_m-before.x_m,dy=after.y_m-before.y_m,chord=std::hypot(dx,dy);
        if (chord<1e-9) throw std::invalid_argument("Barrier track has an undefined normal");
        const Vec2 p{track.points[i].x_m,track.points[i].y_m},normal{-dy/chord,dx/chord};
        left[i]=add(p,mul(normal,(track.left_edge_m.empty()?track.width_m/2:track.left_edge_m[i])+KartEnvironment::barrier_face_m));
        right[i]=add(p,mul(normal,-(track.right_edge_m.empty()?track.width_m/2:track.right_edge_m[i])-KartEnvironment::barrier_face_m));
    }
    std::vector<BarrierSegment> walls; walls.reserve(2*n);
    for (std::size_t i=0;i<n;++i) { walls.push_back({left[i],left[(i+1)%n]}); walls.push_back({right[i],right[(i+1)%n]}); }
    return walls;
}

double barrier_clearance(const KartEnvironment& world,const State& s,const Config& config) {
    double distance=std::numeric_limits<double>::infinity();
    for (const auto& wall:world.walls()) distance=std::min(distance,contact(bumper(s,config),wall).distance);
    return distance-KartEnvironment::bumper_radius_m;
}

PlantState advance_with_barriers(const VehicleModel& model,const PlantState& initial,Command command,
                                 const Config& config,double dt,DrivingAssistance assistance,const KartEnvironment& world) {
    const auto* car=std::get_if<FourWheelCar>(&model);
    if (!car || config.wheelbase_m<=0.08) throw std::invalid_argument("Kart walls require a four-wheel kart");
    if (!std::isfinite(dt) || dt<=0 || dt>0.1) throw std::invalid_argument("Kart wall ticks must be within (0, 0.1] seconds");
    const auto integrate=[&](const PlantState& s,double h) {
        return advance_assisted(model,s,command,config,h,assistance);
    };
    // A free tick is exactly the original plant call. Broad phase includes the
    // entire body's rotational reach and the tick's possible travel.
    auto candidate=integrate(initial,dt);
    const double reach=config.wheelbase_m+KartEnvironment::bumper_radius_m+
        dt*(std::hypot(initial.pose.speed_mps,initial.lateral_velocity_mps)+
            std::hypot(candidate.pose.speed_mps,candidate.lateral_velocity_mps)+1);
    std::vector<const BarrierSegment*> nearby;
    for (const auto& wall:world.walls())
        if (std::max(wall.a.x,wall.b.x)>=initial.pose.x_m-reach && std::min(wall.a.x,wall.b.x)<=initial.pose.x_m+reach &&
            std::max(wall.a.y,wall.b.y)>=initial.pose.y_m-reach && std::min(wall.a.y,wall.b.y)<=initial.pose.y_m+reach)
            nearby.push_back(&wall);
    if (nearby.empty()) return candidate;
    auto s=initial;
    double remaining=dt;
    while (remaining>1e-12) {
        // At most 2.5 cm of bumper travel per contact step, including rotation:
        // substantially less than the bumper radius, so a thin wall cannot be skipped.
        const double travel=std::hypot(s.pose.speed_mps,s.lateral_velocity_mps)+
            (config.wheelbase_m+KartEnvironment::bumper_radius_m)*std::abs(s.yaw_rate_radps)+
            std::abs(command.acceleration_mps2)*remaining+1;
        const double h=std::min({remaining,car->substep_s,0.025/travel});
        const auto before=s;
        s=integrate(s,h);
        for (int iteration=0;iteration<12;++iteration) {
            bool touched=false;
            for (const auto* wall:nearby) {
                const auto hit=contact(bumper(s.pose,config),*wall);
                const double penetration=KartEnvironment::bumper_radius_m-hit.distance;
                if (penetration<=0) continue;
                Vec2 normal;
                if (hit.distance>1e-9) normal=mul(sub(hit.body,hit.wall),1/hit.distance);
                else {
                    const auto old=contact(bumper(before.pose,config),*wall);
                    if (old.distance<1e-9) throw std::runtime_error("Kart starts inside a barrier centreline");
                    normal=mul(sub(old.body,old.wall),1/old.distance);
                }
                const auto surface=sub(hit.body,mul(normal,KartEnvironment::bumper_radius_m));
                impulse(s,*car,surface,normal);
                s.pose.x_m+=normal.x*(penetration+1e-7);
                s.pose.y_m+=normal.y*(penetration+1e-7);
                touched=true;
            }
            if (!touched) break;
        }
        // A wedged corner must never launch the kart or push it through a wall.
        for (const auto* wall:nearby) if (contact(bumper(s.pose,config),*wall).distance<KartEnvironment::bumper_radius_m-1e-6) {
            const double steering=s.pose.steering_rad,time=s.pose.time_s;
            s=before; s.pose.steering_rad=steering; s.pose.time_s=time;
            s.pose.speed_mps=0; s.lateral_velocity_mps=0; s.yaw_rate_radps=0; s.wheel_speeds_radps.fill(0);
            break;
        }
        remaining-=h;
    }
    s.pose.time_s=initial.pose.time_s+dt;
    return s;
}
} // namespace fd
