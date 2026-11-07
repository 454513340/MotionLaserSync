#pragma once
#include "QtShim.h"
namespace LogCategory {
constexpr const char* Controller="Controller";
constexpr const char* Sync="Sync";
}
#define LOG_DEBUG(category, message) ((void)(category), (void)(message))
#define LOG_INFO(category, message) ((void)(category), (void)(message))
#define LOG_WARNING(category, message) ((void)(category), (void)(message))
