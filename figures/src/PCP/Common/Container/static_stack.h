#pragma once

#include <PCP/Common/Assert.h>

namespace pcp {

template<class T, int N>
class static_stack
{
public:
    inline static_stack();

    inline const T& top() const;
    inline       T& top();

    inline bool empty() const;
    inline bool full() const;
    inline int  size() const;

    inline void push(const T& value);
    inline void push();
    inline void pop();
    inline void clear();

protected:
    int m_size;
    T   m_data[N];
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class T, int N>
static_stack<T,N>::static_stack() :
    m_size(0),
    m_data()
{
}

template<class T, int N>
const T& static_stack<T,N>::top() const
{
    PCP_DEBUG_ASSERT(!empty());
    return m_data[m_size-1];
}

template<class T, int N>
T& static_stack<T,N>::top()
{
    PCP_DEBUG_ASSERT(!empty());
    return m_data[m_size-1];
}

template<class T, int N>
bool static_stack<T,N>::empty() const
{
    return m_size == 0;
}

template<class T, int N>
bool static_stack<T,N>::full() const
{
    return m_size == N;
}

template<class T, int N>
int static_stack<T,N>::size() const
{
    return m_size;
}

template<class T, int N>
void static_stack<T,N>::push(const T& value)
{
    PCP_DEBUG_ASSERT(!full());
    m_data[m_size] = value;
    ++m_size;
}

template<class T, int N>
void static_stack<T,N>::push()
{
    PCP_DEBUG_ASSERT(!full());
    m_data[m_size] = T();
    ++m_size;
}

template<class T, int N>
void static_stack<T,N>::pop()
{
    PCP_DEBUG_ASSERT(!empty());
    --m_size;
}

template<class T, int N>
void static_stack<T,N>::clear()
{
    m_size = 0;
}

} // namespace pcp
