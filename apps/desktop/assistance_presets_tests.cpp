#include "assistance_presets.hpp"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QStandardPaths>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    app.setApplicationName("Formula Driverless");
    app.setOrganizationName("Formula Driverless");
    std::cout << "Local preset path: " << (QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)+"/forgiveness-presets.json").toStdString() << '\n';
    int checks=0;
    const auto check=[&](bool value,const char* message) { ++checks; if(!value) throw std::runtime_error(message); };
    try {
        QTemporaryDir dir;
        check(dir.isValid(),"temporary test storage");
        const auto path=dir.filePath("nested/presets.json");
        AssistancePresets store(path);
        fd::DrivingAssistance a{true,72,true,34,14,40,100,79};
        for(int i=0;i<5;++i) {
            check(!store.slot(i),"starts empty");
            a.acceleration=34+i;
            check(store.save(i,a),"save each of five slots");
        }
        check(!store.save(5,a) && !store.save(-1,a),"no sixth slot or negative slot");
        AssistancePresets reopened(path);
        check(reopened.current() && *reopened.current()==a,"Save remembers current setup across restart");
        check(reopened.saveCurrent(*reopened.slot(0)),"remember a loaded preset");
        AssistancePresets loaded(path);
        check(loaded.current() && *loaded.current()==*loaded.slot(0),"loaded manual setup survives restart");
        auto badCurrent=a; badCurrent.speed=101;
        check(!loaded.saveCurrent(badCurrent) && *loaded.current()==*loaded.slot(0),"invalid current setup preserves previous setup");
        for(int i=0;i<5;++i) {
            a.acceleration=34+i;
            check(reopened.slot(i) && *reopened.slot(i)==a,"all mode and slider values survive reopening");
        }
        a.manual=false; a.level=45;
        check(reopened.save(2,a),"replace existing slot");
        AssistancePresets replaced(path);
        check(*replaced.slot(2)==a && replaced.slot(1)->acceleration==35,"replacement affects only its slot");
        check(replaced.save(2,std::nullopt),"delete slot");
        AssistancePresets deleted(path);
        check(deleted.current() && *deleted.current()==a,"deleting a slot preserves current setup");
        check(!deleted.slot(2) && deleted.slot(4).has_value(),"deletion persists and preserves neighbours");
        auto invalid=a; invalid.grip=101;
        check(!deleted.save(0,invalid) && deleted.slot(0)->acceleration==34,"invalid value cannot replace saved data");
        const auto broken=dir.filePath("broken.json");
        { QFile f(broken); f.open(QIODevice::WriteOnly); f.write("not JSON"); }
        AssistancePresets corrupt(broken);
        check(!corrupt.error().isEmpty() && !corrupt.save(0,a) && !corrupt.saveCurrent(a),"corrupt file is reported and not overwritten");
        { QFile f(broken); f.open(QIODevice::ReadOnly); check(f.readAll()=="not JSON","corrupt bytes are preserved"); }
        AssistancePresets unwritable(dir.path());
        check(!unwritable.save(0,a) && !unwritable.slot(0),"write failure does not claim a saved preset");
        std::cout << checks << " preset checks passed\n";
    } catch(const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
    return 0;
}
