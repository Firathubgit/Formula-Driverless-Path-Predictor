#pragma once

#include <string>

namespace fd {

// 64-bit FNV-1a as 16 hexadecimal digits: an identity check against accidental mismatch, not a cryptographic digest.
// The steering tables, the performance envelopes, the instruments and the perception each carry one of their inputs.
std::string fingerprint_hex(const std::string& text);

}  // namespace fd
