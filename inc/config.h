// This config file sets the configuration to a default, then overrides.

#pragma once

#if __has_include("config_local.h")
    #include "config_local.h"   // local overrides defined first
#endif

#include "config_default.h"     // defaults fill in what's missing