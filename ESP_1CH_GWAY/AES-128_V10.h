#ifndef ARDUINO_AES_128_V10_SHIM
#define ARDUINO_AES_128_V10_SHIM
// Local shim to surface the AES implementation for the Arduino builder.
// The actual implementation lives under lib/aes to keep PlatformIO support.
#if defined(__has_include)
#	if __has_include("../lib/aes/AES-128_V10.h")
#		include "../lib/aes/AES-128_V10.h"
#	elif __has_include("AES-128_V10_src.h")
#		include "AES-128_V10_src.h"
#	else
#		include "AES-128_V10_src.h"
#	endif
#else
#	include "AES-128_V10_src.h"
#endif
#endif // ARDUINO_AES_128_V10_SHIM
