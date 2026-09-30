#pragma once
#include "fd/fingerprint.hpp"
#include "fd/vehicle.hpp"

#include <string>

// Shared by the tables derived from a plant (steering tables, decision 0010; performance envelopes, decision 0017).
namespace fd::detail {

// Every configuration field and vehicle parameter a derived table depends on, as exact round-trip text.
std::string model_identity(const VehicleModel& model, const Config& config);

} // namespace fd::detail
