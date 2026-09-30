#include "track_geometry.hpp"
#include "bridge.hpp"
#include "fd/cones.hpp"
#include <QColor>
#include <QFile>
#include <QTextStream>
#include <QVector3D>
#include <algorithm>
#include <functional>
#include <utility>
#include <vector>
#include <cmath>

namespace {
struct Vertex {float x,y,z,r,g,b,a;};
Vertex vertex(double x,double y,double z,const QColor& c,double alpha=1) {
    const auto linear=[](float v) {return v<=0.04045f?v/12.92f:std::pow((v+0.055f)/1.055f,2.4f);};
    return {float(x),float(y),float(z),linear(c.redF()),linear(c.greenF()),linear(c.blueF()),float(alpha)};
}
struct CoachingPoint { fd::Vec2 position; QColor color; };
const std::vector<CoachingPoint>& kart_guide() {
    // Bundled display data from the user's poster, never passed to a driver or plant.
    static const auto points=[] {
        QFile file(":/kart/gokartcentralen-goteborg-guide.csv");
        if(!file.open(QIODevice::ReadOnly)) qFatal("Missing bundled kart coaching line");
        QTextStream input(&file);
        std::vector<CoachingPoint> result;
        while(!input.atEnd()) {
            const auto line=input.readLine();
            if(line.startsWith('#') || line.startsWith("x_m,") || line.isEmpty())continue;
            const auto fields=line.split(',');
            if(fields.size()!=3) qFatal("Invalid bundled kart coaching line");
            bool x_ok{},y_ok{};
            const double x=fields[0].toDouble(&x_ok),y=fields[1].toDouble(&y_ok);
            if(!x_ok || !y_ok || !std::isfinite(x) || !std::isfinite(y) ||
               (fields[2]!="accelerate" && fields[2]!="coast" && fields[2]!="brake"))
                qFatal("Invalid bundled kart coaching point");
            result.push_back({{x,y},QColor(fields[2]=="accelerate"?"#89eab5":fields[2]=="brake"?"#f47770":"#36adf1")});
        }
        if(result.size()<3) qFatal("Incomplete bundled kart coaching line");
        return result;
    }();
    return points;
}
}
QObject* TrackGeometry::bridge() const {return bridge_.data();}
void TrackGeometry::setKind(int value) {kind_=value;rebuild();emit kindChanged();}
void TrackGeometry::setBridge(QObject* object) {
    if(bridge_) disconnect(bridge_,nullptr,this,nullptr);
    bridge_=qobject_cast<Bridge*>(object);
    if(bridge_) {
        connect(bridge_,&Bridge::planChanged,this,&TrackGeometry::rebuild);
        connect(bridge_,&Bridge::modeChanged,this,&TrackGeometry::rebuild);
        connect(bridge_,&Bridge::updated,this,[this]{
            // The lattice ahead changes only when the car passes a layer, so it is redrawn only then.
            if(kind_==2||kind_==4||kind_==7||kind_==8||kind_==9||kind_==10||(kind_==6&&(bridge_->latticeLayersAhead()!=lattice_layers_||bridge_->lattice()!=lattice_drawn_)))rebuild();
        });
    }
    rebuild();emit bridgeChanged();
}
void TrackGeometry::rebuild() {
    if(!bridge_)return;
    const auto& t=bridge_->track();
    std::vector<Vertex> vertices;
    // The corridor's edges at each sample, measured from the reference along its normal: a track that records its own
    // edges, such as a racing line (decision 0020), is drawn between them, any other half its width to each side.
    const auto left=[&](size_t k) {return t.left_edge_m.empty()?t.width_m/2:t.left_edge_m[k];};
    const auto right=[&](size_t k) {return t.right_edge_m.empty()?t.width_m/2:t.right_edge_m[k];};
    // A band from one offset to another, each taken at both ends of the span, so a corridor whose edges move along
    // the line is drawn without gaps.
    auto band=[&](size_t i,const std::function<double(size_t)>& from,const std::function<double(size_t)>& to,double h,QColor color) {
        const size_t j=(i+1)%t.points.size();
        const auto& p=t.points[i]; const auto& q=t.points[j];
        auto normal=[&](size_t k) {
            const auto& before=t.points[(k+t.points.size()-1)%t.points.size()];
            const auto& after=t.points[(k+1)%t.points.size()];
            const double dx=after.x_m-before.x_m,dy=after.y_m-before.y_m,n=std::hypot(dx,dy);
            return fd::Vec2{-dy/n,dx/n};
        };
        const auto n=normal(i),m=normal(j);
        const auto a=vertex(p.x_m+n.x*from(i),h,-p.y_m-n.y*from(i),color);
        const auto b=vertex(p.x_m+n.x*to(i),h,-p.y_m-n.y*to(i),color);
        const auto c=vertex(q.x_m+m.x*from(j),h,-q.y_m-m.y*from(j),color);
        const auto d=vertex(q.x_m+m.x*to(j),h,-q.y_m-m.y*to(j),color);
        for(auto v:{a,b,c,b,d,c})vertices.push_back(v);
    };
    for(size_t i=0;(kind_==0||kind_==1||kind_==11) && i<t.points.size();++i) {
        if(kind_==0) band(i,[&](size_t k){return -right(k);},left,0.01,QColor("#202329"));
        else if(kind_==11) {
            // Beyond the edges, ground in the backdrop's own colour, unseen: it hides what lies under the surface, the
            // car's reflection, wherever the road does not reach.
            band(i,[&](size_t k){return -right(k)-4.0;},[&](size_t k){return -right(k)-0.05;},0.0,QColor("#08090b"));
            band(i,[&](size_t k){return left(k)+0.05;},[&](size_t k){return left(k)+4.0;},0.0,QColor("#08090b"));
        }
        else if(kind_==1) {
            // The edges read as a road's lines; the kerbs beyond them stay faint, a sense of speed rather than a feature.
            band(i,[&](size_t k){return -right(k)-0.055;},[&](size_t k){return -right(k)+0.055;},0.025,QColor("#838c94"));
            band(i,[&](size_t k){return left(k)-0.055;},[&](size_t k){return left(k)+0.055;},0.025,QColor("#838c94"));
            if(i%8<4) {
                band(i,[&](size_t k){return -right(k)-0.39;},[&](size_t k){return -right(k)-0.05;},0.028,QColor("#1d2226"));
                band(i,[&](size_t k){return left(k)+0.05;},[&](size_t k){return left(k)+0.39;},0.028,QColor("#1d2226"));
            }
        }
    }
    // A polyline drawn as a flat ribbon. Normals come from neighbouring points rather than a
    // vehicle yaw, because recorded rollouts do not carry yaw; for a rear-axle path without
    // slip the two agree. The ribbon's centre stays exactly on the points.
    auto ribbon=[&](const std::vector<fd::Vec2>& points,double half_width,double height,
                    const std::function<QColor(std::size_t)>& color_of,bool closed=false) {
        const auto normal=[&](std::size_t k) {
            const auto& before=points[k>0?k-1:closed?points.size()-2:k];
            const auto& after=points[k+1<points.size()?k+1:closed?1:k];
            const double dx=after.x-before.x,dy=after.y-before.y,n=std::hypot(dx,dy);
            return n>1e-12?fd::Vec2{-dy/n,dx/n}:fd::Vec2{0,0};
        };
        for (std::size_t i=1; i<points.size(); ++i) {
            const auto& p=points[i-1];
            const auto& q=points[i];
            if (std::hypot(q.x-p.x,q.y-p.y)<1e-8) continue;
            const QColor color=color_of(i-1);
            const auto n=normal(i-1),m=normal(i);
            const auto a=vertex(p.x-n.x*half_width,height,-p.y+n.y*half_width,color);
            const auto b=vertex(p.x+n.x*half_width,height,-p.y-n.y*half_width,color);
            const auto c=vertex(q.x-m.x*half_width,height,-q.y+m.y*half_width,color);
            const auto right_end=vertex(q.x+m.x*half_width,height,-q.y-m.y*half_width,color);
            for (auto v:{a,b,c,b,right_end,c}) vertices.push_back(v);
        }
    };
    if(kind_==12 && bridge_->kartMode()) {
        const auto& guide=kart_guide();
        std::vector<fd::Vec2> points;
        points.reserve(guide.size()+1);
        for(const auto& p:guide)points.push_back(p.position);
        points.push_back(points.front());
        ribbon(points,0.2,0.04,[&](std::size_t i){return guide[i].color;},true);
    }
    if (kind_==2) {
        // A finite controller rollout, live or recorded with its decision, in actual world
        // coordinates. Overview changes the camera only; it never becomes a closed circuit line.
        const auto& samples=bridge_->prediction();
        const auto color_of=[&](std::size_t i) {
            const double acceleration=samples[i].acceleration_mps2;
            return acceleration>fd::plan_color_deadband_mps2?QColor("#89eab5"):
                   acceleration < -fd::plan_color_deadband_mps2?QColor("#f47770"):QColor("#e6cf78");
        };
        // A band about the car's width: a solid core with soft edges, each sample in the colour of its planned motion so
        // the colours blend where the plan changes, fading out towards the horizon's end. The core comes first, so the
        // ribbon's first two vertices are the core's either side of the actual rear axle.
        // A kart is 1.35 m wide, not a car's 1.9 m (decision 0038).
        const double narrow=bridge_->kartMode()?0.71:1.0;
        const double core=0.62*narrow,edge=0.95*narrow,height=0.05;
        std::vector<double> along(samples.size(),0);
        for (std::size_t i=1; i<samples.size(); ++i)
            along[i]=along[i-1]+std::hypot(samples[i].state.x_m-samples[i-1].state.x_m,samples[i].state.y_m-samples[i-1].state.y_m);
        const double length=samples.empty()?0:along.back();
        const auto alpha=[&](std::size_t i) {return length>1e-9?0.85*(1-std::pow(along[i]/length,1.6)):0.85;};
        const auto normal=[&](std::size_t k) {
            const auto& before=samples[k>0?k-1:k].state;
            const auto& after=samples[k+1<samples.size()?k+1:k].state;
            const double dx=after.x_m-before.x_m,dy=after.y_m-before.y_m,n=std::hypot(dx,dy);
            return n>1e-12?fd::Vec2{-dy/n,dx/n}:fd::Vec2{0,0};
        };
        const auto strip=[&](double from,double to,bool soft_from,bool soft_to) {
            for (std::size_t i=1; i<samples.size(); ++i) {
                const auto& p=samples[i-1].state;
                const auto& q=samples[i].state;
                if (std::hypot(q.x_m-p.x_m,q.y_m-p.y_m)<1e-8) continue;
                const auto n=normal(i-1),m=normal(i);
                const auto at=[&](const fd::State& s,fd::Vec2 k,double offset,std::size_t index,bool soft) {
                    return vertex(s.x_m-k.x*offset,height,-s.y_m+k.y*offset,color_of(index),soft?0:alpha(index));
                };
                const auto a=at(p,n,from,i-1,soft_from),b=at(p,n,to,i-1,soft_to);
                const auto c=at(q,m,from,i,soft_from),d=at(q,m,to,i,soft_to);
                for (auto v:{a,b,c,b,d,c}) vertices.push_back(v);
            }
        };
        strip(core,-core,false,false);
        strip(edge,core,true,false);
        strip(-core,-edge,false,true);
    }
    if (kind_==3) {
        // The blocked regions exactly as the scenario stated them, with no vehicle-margin
        // inflation: the viewer sees the ground truth, not the planner's working allowance.
        const auto frame=[&](double s) {
            const auto here=fd::sample(t,s);
            const auto ahead=fd::sample(t,s+0.05);
            const double tx=ahead.x_m-here.x_m,ty=ahead.y_m-here.y_m,n=std::hypot(tx,ty);
            const fd::Vec2 normal=n>1e-12?fd::Vec2{-ty/n,tx/n}:fd::Vec2{0,0};
            return std::pair<fd::PathPoint,fd::Vec2>{here,normal};
        };
        for(const auto& o:bridge_->obstructions()) {
            const double span=std::fmod(o.to_s_m-o.from_s_m+t.length_m,t.length_m);
            const int cells=std::max(1,static_cast<int>(std::ceil(span)));
            for(int k=0;k<cells;++k) {
                const auto [p,np]=frame(o.from_s_m+span*k/cells);
                const auto [q,nq]=frame(o.from_s_m+span*(k+1)/cells);
                const QColor color("#d8574f");
                const auto a=vertex(p.x_m+np.x*o.from_offset_m,0.09,-p.y_m-np.y*o.from_offset_m,color);
                const auto b=vertex(p.x_m+np.x*o.to_offset_m,0.09,-p.y_m-np.y*o.to_offset_m,color);
                const auto c=vertex(q.x_m+nq.x*o.from_offset_m,0.09,-q.y_m-nq.y*o.from_offset_m,color);
                const auto far_end=vertex(q.x_m+nq.x*o.to_offset_m,0.09,-q.y_m-nq.y*o.to_offset_m,color);
                for(auto v:{a,b,c,b,far_end,c})vertices.push_back(v);
            }
        }
    }
    if (kind_==5) {
        // The line the car is not driving, beside the one it is: the racing line on a centreline run, the centreline on
        // a racing line run. Never the car's path; the prediction ribbon is.
        const QColor color=bridge_->lineMode()==1?QColor("#9aa7ad"):QColor("#7fc4e8");
        ribbon(bridge_->comparisonLine(),0.14,0.03,[&](std::size_t){return color;});
    }
    if (kind_==6) {
        // The offline lattice's edges leaving the layers within the horizon ahead of the car, as faint lines at their
        // own samples: what a local planner could choose among, not a path the car takes.
        lattice_layers_=bridge_->latticeLayersAhead();
        lattice_drawn_=bridge_->lattice();
        if(const auto* lattice=bridge_->lattice()) {
            const QColor color("#4d6470");
            for(const auto layer:lattice_layers_)
                for(std::size_t node=0;node<lattice->layers[layer].offsets_m.size();++node)
                    for(const auto& edge:lattice->edges_from(layer,node))
                        for(std::size_t k=1;k<edge.samples.size();++k) {
                            const auto& a=edge.samples[k-1];
                            const auto& b=edge.samples[k];
                            vertices.push_back(vertex(a.x_m,0.02,-a.y_m,color));
                            vertices.push_back(vertex(b.x_m,0.02,-b.y_m,color));
                        }
        }
    }
    if (kind_==4) {
        // Alternatives evaluated and rejected by the decision in force, live or recorded.
        // Drawing them is the point: the choice is only visible next to what it was chosen over.
        // Under the lattice planner each action's line has its own colour, darker when it was rejected.
        for(const auto& line:bridge_->alternatives()) {
            const QColor color=Bridge::actionColor(line.action,line.clear);
            ribbon(line.points,0.07,0.035,[&](std::size_t){return color;});
        }
    }
    // A flat marker on the ground: a square, a diamond, a ring of a width, or a cross, as triangles.
    const auto quad=[&](fd::Vec2 a,fd::Vec2 b,fd::Vec2 c,fd::Vec2 e,double h,const QColor& color) {
        const auto va=vertex(a.x,h,-a.y,color),vb=vertex(b.x,h,-b.y,color),vc=vertex(c.x,h,-c.y,color),vd=vertex(e.x,h,-e.y,color);
        for(auto v:{va,vb,vc,va,vc,vd})vertices.push_back(v);
    };
    if (kind_==13 && bridge_->kartMode()) {
        const auto gate=fd::make_gates(t).front();
        const double dx=gate.right.x-gate.left.x,dy=gate.right.y-gate.left.y,n=std::hypot(dx,dy);
        const fd::Vec2 centre{(gate.left.x+gate.right.x)/2,(gate.left.y+gate.right.y)/2};
        const auto at=[&](double across,double ahead) {
            return fd::Vec2{centre.x+dx/n*across-dy/n*ahead,centre.y+dy/n*across+dx/n*ahead};
        };
        for(int row=0;row<2;++row) for(int col=0;col<12;++col) {
            const double a=-t.width_m/2+t.width_m*col/12,b=a+t.width_m/12;
            const double near_edge=-0.3+row*0.3,far_edge=near_edge+0.3;
            quad(at(a,near_edge),at(b,near_edge),at(b,far_edge),at(a,far_edge),0.055,
                 (row+col)%2 ? QColor("#17191b") : QColor("#ffffff"));
        }
    }
    const auto segment=[&](fd::Vec2 a,fd::Vec2 b,double width,double h,const QColor& color) {
        const double dx=b.x-a.x,dy=b.y-a.y,n=std::hypot(dx,dy);
        if(n<1e-9)return;
        const double nx=-dy/n*width/2,ny=dx/n*width/2;
        quad({a.x+nx,a.y+ny},{b.x+nx,b.y+ny},{b.x-nx,b.y-ny},{a.x-nx,a.y-ny},h,color);
    };
    const auto ring=[&](fd::Vec2 centre,double radius,double width,double h,const QColor& color) {
        constexpr int sides=12;
        for(int k=0;k<sides;++k) {
            const double a0=2*3.14159265358979*k/sides,a1=2*3.14159265358979*(k+1)/sides;
            segment({centre.x+radius*std::cos(a0),centre.y+radius*std::sin(a0)},
                    {centre.x+radius*std::cos(a1),centre.y+radius*std::sin(a1)},width,h,color);
        }
    };
    const auto cone_colour=[](int colour) {
        switch(colour){case 0:return QColor("#3d7ff2");case 1:return QColor("#f2d23d");case 2:return QColor("#f28c3d");
                       case 3:return QColor("#f2663d");default:return QColor("#c9d1d4");}
    };
    if (kind_==7) {
        // The course's cones where they truly stand: ground truth, drawn as small squares in their colours. A cone the
        // newest frame had in view but missed carries a grey ring.
        const auto& cones=bridge_->perceivedCones();
        for(const auto& cone:cones) {
            const auto p=cone.position;
            const double r=cone.colour==fd::ConeColour::big_orange?0.3:0.2;
            quad({p.x-r,p.y-r},{p.x+r,p.y-r},{p.x+r,p.y+r},{p.x-r,p.y+r},0.05,cone_colour(static_cast<int>(cone.colour)));
        }
        for(const auto i:bridge_->perceptionView().missed)
            if(i<cones.size()) ring(cones[i].position,0.55,0.08,0.06,QColor("#8c979c"));
    }
    if (kind_==8) {
        // Simulated detections where the sensor put them, from where the car truly was when the frame was sampled: a
        // diamond in the colour reported, white when unknown, a red cross over one whose colour is wrong, and the ellipse
        // of twice its standard deviation.
        for(const auto& det:bridge_->perceptionView().detections) {
            const auto p=det.position;
            const QColor color=det.colour==fd::ObservedColour::unknown?QColor("#e8eef0"):cone_colour(static_cast<int>(det.colour)-1);
            const double r=0.28;
            segment({p.x-r,p.y},{p.x,p.y+r},0.07,0.08,color);
            segment({p.x,p.y+r},{p.x+r,p.y},0.07,0.08,color);
            segment({p.x+r,p.y},{p.x,p.y-r},0.07,0.08,color);
            segment({p.x,p.y-r},{p.x-r,p.y},0.07,0.08,color);
            if(det.wrong_colour) {
                segment({p.x-0.4,p.y-0.4},{p.x+0.4,p.y+0.4},0.08,0.09,QColor("#f45b5b"));
                segment({p.x-0.4,p.y+0.4},{p.x+0.4,p.y-0.4},0.08,0.09,QColor("#f45b5b"));
            }
            // The ellipse's axes from the covariance's eigenvalues, drawn at two standard deviations.
            const double a=det.covariance_xx,b=det.covariance_xy,c=det.covariance_yy;
            const double mean=(a+c)/2,spread=std::sqrt(std::max(0.0,(a-c)*(a-c)/4+b*b));
            const double major=2*std::sqrt(std::max(0.0,mean+spread)),minor=2*std::sqrt(std::max(0.0,mean-spread));
            const double angle=0.5*std::atan2(2*b,a-c);
            constexpr int sides=16;
            for(int k=0;k<sides;++k) {
                const auto at=[&](int j) {
                    const double t=2*3.14159265358979*j/sides;
                    const double u=major*std::cos(t),v=minor*std::sin(t);
                    return fd::Vec2{p.x+u*std::cos(angle)-v*std::sin(angle),p.y+u*std::sin(angle)+v*std::cos(angle)};
                };
                segment(at(k),at(k+1),0.03,0.07,QColor("#a9b8bf"));
            }
        }
    }
    if (kind_==9) {
        // The path the car believes from its own detections, on the map where its believed pose put them, the corridor it
        // believes either side, and a ring where it believes it is (decision 0031): beside the true track, the belief.
        const auto view=bridge_->believedView();
        const QColor path_colour("#c89cf5"),edge_colour("#6f5a8a");
        for(std::size_t k=1;k<view.path.size();++k) {
            const auto a=view.path[k-1],b=view.path[k];
            segment(a,b,0.16,0.11,path_colour);
            const double dx=b.x-a.x,dy=b.y-a.y,n=std::hypot(dx,dy);
            if(n<1e-9)continue;
            const double nx=-dy/n*view.width_m/2,ny=dx/n*view.width_m/2;
            segment({a.x+nx,a.y+ny},{b.x+nx,b.y+ny},0.05,0.1,edge_colour);
            segment({a.x-nx,a.y-ny},{b.x-nx,b.y-ny},0.05,0.1,edge_colour);
        }
        if(view.has_belief) ring({view.believed.x_m,view.believed.y_m},0.6,0.1,0.12,path_colour);
    }
    if (kind_==10) {
        // Cones the judge saw knocked down or out (decision 0032): a red base with a cross, where each stood.
        const auto& cones=bridge_->courseCones();
        for(const auto i:bridge_->timingView().hit) {
            if(i>=cones.size())continue;
            const auto p=cones[i].position;
            const double r=0.35;
            quad({p.x-r,p.y-r},{p.x+r,p.y-r},{p.x+r,p.y+r},{p.x-r,p.y+r},0.13,QColor("#f45b5b"));
            segment({p.x-0.55,p.y-0.55},{p.x+0.55,p.y+0.55},0.09,0.14,QColor("#ffd6d6"));
            segment({p.x-0.55,p.y+0.55},{p.x+0.55,p.y-0.55},0.09,0.14,QColor("#ffd6d6"));
        }
    }
    clear();
    setVertexData(QByteArray(reinterpret_cast<const char*>(vertices.data()),qsizetype(vertices.size()*sizeof(Vertex))));
    setStride(sizeof(Vertex));
    setPrimitiveType(kind_==6?PrimitiveType::Lines:PrimitiveType::Triangles);
    addAttribute(Attribute::PositionSemantic,0,Attribute::F32Type);
    addAttribute(Attribute::ColorSemantic,3*sizeof(float),Attribute::F32Type);
    setBounds(QVector3D(-1000,-1,-1000),QVector3D(1000,5,1000));
    update();
}
