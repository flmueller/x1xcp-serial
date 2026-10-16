#pragma once

// Sizes of the XCP slave for this library (the Arduino IDE has no build flags per
// library); -D options of the build (PlatformIO: library.json) win.

#ifndef X1_XCPSLAVE_MAX_CTO
#define X1_XCPSLAVE_MAX_CTO 64
#endif

#ifndef X1_XCPSLAVE_MAX_DTO
#define X1_XCPSLAVE_MAX_DTO 64
#endif

#ifndef X1_XCPSLAVE_MAX_DAQ_LISTS
#define X1_XCPSLAVE_MAX_DAQ_LISTS 4
#endif

#ifndef X1_XCPSLAVE_MAX_ODTS
#define X1_XCPSLAVE_MAX_ODTS 12
#endif

#ifndef X1_XCPSLAVE_MAX_ODT_ENTRIES
#define X1_XCPSLAVE_MAX_ODT_ENTRIES 48
#endif
