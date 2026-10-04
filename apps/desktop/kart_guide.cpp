#include "kart_guide.hpp"

#include <QFile>
#include <QTextStream>
#include <cmath>

const std::vector<fd::CoachSample>& kart_guide_line() {
    static const auto line=[] {
        QFile file(":/kart/gokartcentralen-goteborg-guide.csv");
        if(!file.open(QIODevice::ReadOnly)) qFatal("Missing bundled kart coaching line");
        QTextStream input(&file);
        std::vector<fd::CoachSample> result;
        while(!input.atEnd()) {
            const auto text=input.readLine();
            if(text.startsWith('#') || text.startsWith("x_m,") || text.isEmpty())continue;
            const auto fields=text.split(',');
            if(fields.size()!=3) qFatal("Invalid bundled kart coaching line");
            bool x_ok{},y_ok{};
            const double x=fields[0].toDouble(&x_ok),y=fields[1].toDouble(&y_ok);
            if(!x_ok || !y_ok || !std::isfinite(x) || !std::isfinite(y) ||
               (fields[2]!="accelerate" && fields[2]!="coast" && fields[2]!="brake"))
                qFatal("Invalid bundled kart coaching point");
            result.push_back({{x,y},fields[2]=="accelerate"?fd::Advice::Accelerate:fields[2]=="brake"?fd::Advice::Brake:fd::Advice::Coast});
        }
        if(result.size()<3) qFatal("Incomplete bundled kart coaching line");
        return result;
    }();
    return line;
}
