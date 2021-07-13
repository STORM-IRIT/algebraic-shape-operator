#pragma once

#include <PCP/Common/Container/dynamic_stack.h>
#include <PCP/Common/Container/static_stack.h>

#include <type_traits>

namespace pcp {

template<typename T, int N = -1>
using generic_stack = typename std::conditional<N == -1,
                                                dynamic_stack<T>,
                                                static_stack<T,N>>::type;

} // namespace pcp
