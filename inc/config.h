// This config file sets the configuration to a default, then overrides.

#pragma once

#include "config_default.h"
//#include "config_local.h"   // comment this line out if you don't have a config_local.h file

// if this doesn't work, we will go with the janky "comment out the code you don't need" above
#if __has_include("config_local.h")
    #include "config_local.h"
#endif