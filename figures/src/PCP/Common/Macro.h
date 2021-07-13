#pragma once

#include <cstdio>

// macro delimiters
#define PCP_MACRO_START do {
#define PCP_MACRO_END   } while(0)

// Stringification
#define PCP_XSTR(S) #S
#define PCP_STR(S) PCP_XSTR(S)

#define PCP_CRASH                                                              \
    PCP_MACRO_START                                                            \
    asm volatile ("int $3");                                                   \
    PCP_MACRO_END

#define PCP_PRINT_ERROR(MSG)                                                   \
    PCP_MACRO_START                                                            \
    fprintf(stderr,                                                            \
            "%s:%i: [Error] %s\n",                                             \
            __FILE__,__LINE__,MSG);                                            \
    fflush(stderr);                                                            \
    PCP_MACRO_END

#define PCP_PRINT_WARNING(MSG)                                                 \
    PCP_MACRO_START                                                            \
    fprintf(stderr,                                                            \
            "%s:%i: [Warning] %s\n",                                           \
            __FILE__,__LINE__,MSG);                                            \
    fflush(stderr);                                                            \
    PCP_MACRO_END

// turnoff warning
#define PCP_UNUSED(VAR)                                                        \
    PCP_MACRO_START                                                            \
    (void)(VAR);                                                               \
    PCP_MACRO_END

#define PCP_TODO                                                               \
    PCP_MACRO_START                                                            \
    PCP_PRINT_ERROR("TODO");                                                   \
    PCP_CRASH;                                                                 \
    PCP_MACRO_END

template<typename T> struct XXXXXXXXXX_The_unkown_type_is_;
#define WhatIsTheTypeOf(expr) XXXXXXXXXX_The_unkown_type_is_<decltype(expr)> _XXXXXXXXXX
#define WhatIsThisType(type) XXXXXXXXXX_The_unkown_type_is_<type> _XXXXXXXXXX
