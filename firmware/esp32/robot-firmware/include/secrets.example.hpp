#pragma once

/// @file
/// Template for local credentials. Copy to `secrets.hpp` (git-ignored) and edit.
/// Without `secrets.hpp` the build falls back to these placeholder values.

/// Credentials that must not be committed.
namespace robot::secrets {

/// SSID of the debug access point the ESP32 opens.
inline constexpr const char* kDebugApSsid = "robot-debug";
/// WPA2 password of the debug access point (at least 8 characters).
inline constexpr const char* kDebugApPassword = "change-me-please";

}  // namespace robot::secrets
