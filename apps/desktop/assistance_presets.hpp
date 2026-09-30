#pragma once
#include "fd/vehicle.hpp"
#include <QString>
#include <array>
#include <optional>

// Five local slots, atomically saved as one small JSON file. No database service.
class AssistancePresets {
public:
    explicit AssistancePresets(QString path);
    const std::optional<fd::DrivingAssistance>& slot(int index) const;
    bool save(int index, std::optional<fd::DrivingAssistance> value);
    bool saveCurrent(const fd::DrivingAssistance& value);
    const std::optional<fd::DrivingAssistance>& current() const { return current_; }
    QString error() const { return error_; }
private:
    QString path_, error_;
    bool readable_{true};
    std::array<std::optional<fd::DrivingAssistance>,5> slots_{};
    std::optional<fd::DrivingAssistance> current_;
    bool write(const std::array<std::optional<fd::DrivingAssistance>,5>& slots,
               std::optional<fd::DrivingAssistance> current);
};
