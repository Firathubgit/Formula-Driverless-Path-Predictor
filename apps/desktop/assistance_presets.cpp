#include "assistance_presets.hpp"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <stdexcept>

namespace {
QJsonObject encode(const fd::DrivingAssistance& a) {
    return {{"enabled",a.enabled},{"level",a.level},{"manual",a.manual},{"acceleration",a.acceleration},
            {"braking",a.braking},{"steering",a.steering},{"grip",a.grip},{"speed",a.speed}};
}
fd::DrivingAssistance decode(const QJsonValue& value) {
    if (!value.isObject()) throw std::invalid_argument("Invalid preset");
    const auto o=value.toObject();
    if (!o["enabled"].isBool() || !o["manual"].isBool()) throw std::invalid_argument("Invalid preset mode");
    const auto number=[&](const char* key) {
        if (!o[key].isDouble()) throw std::invalid_argument("Missing preset value");
        return o[key].toDouble();
    };
    fd::DrivingAssistance a{o["enabled"].toBool(),number("level"),o["manual"].toBool(),number("acceleration"),
                           number("braking"),number("steering"),number("grip"),number("speed")};
    fd::validate_driving_assistance(a);
    return a;
}
}
AssistancePresets::AssistancePresets(QString path):path_(std::move(path)) {
    QFile file(path_);
    if (!file.exists()) return;
    try {
        if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot open presets");
        if (file.size()>16384) throw std::runtime_error("Preset file is too large");
        const auto doc=QJsonDocument::fromJson(file.readAll());
        const auto root=doc.object();
        if (!doc.isObject() || root["version"].toInt()!=1 || !root["slots"].isArray() || root["slots"].toArray().size()!=5)
            throw std::runtime_error("Invalid preset file");
        auto loaded=slots_;
        const auto entries=root["slots"].toArray();
        for(int i=0;i<5;++i) if (!entries[i].isNull()) loaded[i]=decode(entries[i]);
        std::optional<fd::DrivingAssistance> current;
        if (root.contains("current") && !root["current"].isNull()) current=decode(root["current"]);
        slots_=loaded; current_=current;
    } catch (const std::exception&) {
        readable_=false; error_="Could not read local presets. The file was left unchanged.";
    }
}
const std::optional<fd::DrivingAssistance>& AssistancePresets::slot(int index) const {
    if (index<0 || index>=5) throw std::invalid_argument("Choose preset 1 to 5");
    return slots_[index];
}
bool AssistancePresets::save(int index,std::optional<fd::DrivingAssistance> value) {
    if (!readable_) return false;
    if (index<0 || index>=5) { error_="Choose preset 1 to 5."; return false; }
    if (value) {
        try { fd::validate_driving_assistance(*value); }
        catch (const std::exception&) { error_="Invalid preset settings."; return false; }
    }
    auto next=slots_; next[index]=value;
    return write(next,value ? value : current_);
}
bool AssistancePresets::saveCurrent(const fd::DrivingAssistance& value) {
    if (!readable_) return false;
    try { fd::validate_driving_assistance(value); }
    catch (const std::exception&) { error_="Invalid current settings."; return false; }
    return write(slots_,value);
}
bool AssistancePresets::write(const std::array<std::optional<fd::DrivingAssistance>,5>& next,
                              std::optional<fd::DrivingAssistance> current) {
    QJsonArray entries;
    for(const auto& entry:next) entries.append(entry ? QJsonValue(encode(*entry)) : QJsonValue(QJsonValue::Null));
    const auto bytes=QJsonDocument(QJsonObject{{"version",1},{"slots",entries},
        {"current",current ? QJsonValue(encode(*current)) : QJsonValue(QJsonValue::Null)}}).toJson();
    QSaveFile file(path_);
    if (!QDir().mkpath(QFileInfo(path_).absolutePath()) || !file.open(QIODevice::WriteOnly) ||
        file.write(bytes)!=bytes.size() || !file.commit()) {
        error_="Could not save local presets. Your saved slots were not changed."; return false;
    }
    slots_=next; current_=current; error_.clear(); return true;
}
