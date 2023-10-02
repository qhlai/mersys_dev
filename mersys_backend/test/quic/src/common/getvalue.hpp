#pragma once
#include "msquic.h"
BOOLEAN
GetFlag(
    _In_ int argc,
    _In_reads_(argc) _Null_terminated_ char* argv[],
    _In_z_ const char* name
    );
_Ret_maybenull_ _Null_terminated_ const char*
GetValue(
    _In_ int argc,
    _In_reads_(argc) _Null_terminated_ char* argv[],
    _In_z_ const char* name
    );