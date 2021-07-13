#pragma once

#include <PCP/Common/Macro.h>

#define PCP_ERROR                                                              \
    PCP_MACRO_START                                                            \
    PCP_PRINT_ERROR("");                                                       \
    PCP_CRASH;                                                                 \
    PCP_MACRO_END

#define PCP_ERROR_MSG(MSG)                                                     \
    PCP_MACRO_START                                                            \
    PCP_PRINT_ERROR(MSG);                                                      \
    PCP_CRASH;                                                                 \
    PCP_MACRO_END

#define PCP_ASSERT(EXPR)                                                       \
    PCP_MACRO_START                                                            \
    if(!(EXPR)) {                                                              \
        PCP_ERROR_MSG(PCP_STR(EXPR));                                          \
    }                                                                          \
    PCP_MACRO_END

#define PCP_ASSERT_MSG(EXPR,MSG)                                               \
    PCP_MACRO_START                                                            \
    if(!(EXPR)) {                                                              \
        PCP_ERROR_MSG(MSG);                                                    \
    }                                                                          \
    PCP_MACRO_END

#ifdef PCP_DEBUG
    #define PCP_DEBUG_ASSERT(EXPR)          PCP_ASSERT(EXPR)
    #define PCP_DEBUG_ASSERT_MSG(EXPR,MSG)  PCP_ASSERT_MSG(EXPR,MSG)
    #define PCP_DEBUG_ERROR_MSG(MSG)        PCP_ERROR_MSG(MSG)
    #define PCP_DEBUG_ERROR                 PCP_ERROR
#else
    #define PCP_DEBUG_ASSERT(EXPR)
    #define PCP_DEBUG_ASSERT_MSG(EXPR,MSG)
    #define PCP_DEBUG_ERROR_MSG(MSG)
    #define PCP_DEBUG_ERROR
#endif
