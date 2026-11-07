// Standalone vendor SDK type smoke-test (Windows Developer Command Prompt):
// cl /nologo /std:c++17 /EHsc /c /Ithird_party/scanCONTROL/include tools/sdk_cmm_signature_compile.cpp
// This confirms the signature; it does NOT build the full GUI.
#include <type_traits>
#include <utility>
#include "InterfaceLLT_2.h"
static_assert(std::is_same_v<
    decltype(std::declval<CInterfaceLLT&>().Timestamp2CmmTriggerAndInCounter(
        nullptr, nullptr, nullptr, nullptr, nullptr)), void>);
