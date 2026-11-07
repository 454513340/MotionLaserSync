// third_party/scanCONTROL/source/stdafx.h
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

// Force narrow WinAPI in this translation unit only.
// This avoids Unicode/ANSI mismatch for DllLoader.cpp.
#ifdef UNICODE
#undef UNICODE
#endif
#ifdef _UNICODE
#undef _UNICODE
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
