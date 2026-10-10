#pragma once

/// @file
/// Selects local credentials if present, so a fresh clone (and CI) still builds.

#if __has_include("secrets.hpp")
#include "secrets.hpp"
#else
#pragma message("secrets.hpp not found: using placeholder credentials from secrets.example.hpp")
#include "secrets.example.hpp"
#endif
