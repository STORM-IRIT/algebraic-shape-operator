#pragma once

#include <PCP/Common/Assert.h>

#include <stack>

namespace pcp {

template<class T>
class dynamic_stack
{
public:
    inline dynamic_stack();

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
    std::stack<T> m_data;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class T>
dynamic_stack<T>::dynamic_stack() :
    m_data()
{
}

template<class T>
const T& dynamic_stack<T>::top() const
{
    PCP_DEBUG_ASSERT(!empty());
    return m_data.top();
}

template<class T>
T& dynamic_stack<T>::top()
{
    PCP_DEBUG_ASSERT(!empty());
    return m_data.top();
}

template<class T>
bool dynamic_stack<T>::empty() const
{
    return m_data.empty();
}

template<class T>
bool dynamic_stack<T>::full() const
{
    return false;
}

template<class T>
int dynamic_stack<T>::size() const
{
    return m_data.size();
}

template<class T>
void dynamic_stack<T>::push(const T& value)
{
    PCP_DEBUG_ASSERT(!full());
    m_data.push(value);
}

template<class T>
void dynamic_stack<T>::push()
{
    PCP_DEBUG_ASSERT(!full());
    m_data.push();
}

template<class T>
void dynamic_stack<T>::pop()
{
    PCP_DEBUG_ASSERT(!empty());
    m_data.pop();
}

template<class T>
void dynamic_stack<T>::clear()
{
    while(!m_data.empty())
    {
        m_data.pop();
    }
}

} // namespace pcp
