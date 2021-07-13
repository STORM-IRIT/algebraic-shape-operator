#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>

namespace Log {
namespace internal {

enum LogLevel : int
{
    Critical  =  0,
    Error,    // 1
    Warning,  // 2
    Info,     // 3
    Debug     // 4
};

template<LogLevel level> inline const char* level_string();
template<LogLevel level> inline void print(const std::stringstream& ss);

template<LogLevel level>
class LogMessage
{
public:
    inline LogMessage();
    inline LogMessage(LogMessage&& other);
    inline ~LogMessage();

    template<typename T>
    inline LogMessage& operator << (T val);

    inline LogMessage& _if(bool b);

protected:
    bool              m_on;
    std::stringstream m_ss;
};

class NoMessage
{
public:
    inline NoMessage();
    inline NoMessage(NoMessage&& other);
    inline ~NoMessage();

    template<typename T>
    inline NoMessage& operator << (T val);

    inline NoMessage& _if(bool b);
};

} // namespace internal
} // namespace log

// -----------------------------------------------------------------------------

namespace Log {
namespace internal {

template<> inline const char* level_string<Critical>(){return "[critical]";}
template<> inline const char* level_string<Error   >(){return "[error]";}
template<> inline const char* level_string<Warning >(){return "[warning]";}
template<> inline const char* level_string<Info    >(){return "[info]";}
template<> inline const char* level_string<Debug   >(){return "[debug]";}

template<LogLevel level>
inline void print(const std::stringstream& ss)
{
    std::cout << ss.rdbuf() << std::endl;
}

template<>
inline void print<Critical>(const std::stringstream& ss)
{
    std::cerr << ss.rdbuf() << std::endl;
    std::abort();
}

template<>
inline void print<Error>(const std::stringstream& ss)
{
    std::cerr << ss.rdbuf() << std::endl;
}

template<LogLevel level>
LogMessage<level>::LogMessage() :
    m_on(true),
    m_ss()
{
    const auto t  = std::time(nullptr);
    const auto tm = *std::localtime(&t);

    m_ss << std::put_time(&tm, "%Y-%m-%d %H-%M-%S") << " " << level_string<level>() << " ";
}

template<LogLevel level>
LogMessage<level>::LogMessage(LogMessage&& other) :
    m_on(other.m_on),
    m_ss(std::move(other.m_ss))
{
}

template<LogLevel level>
LogMessage<level>::~LogMessage()
{
    if(m_on)
    {
        print<level>(m_ss);
    }
}

template<LogLevel level>
template<typename T>
LogMessage<level>& LogMessage<level>::operator << (T val)
{
    m_ss << val;
    return *this;
}

template<LogLevel level>
LogMessage<level>& LogMessage<level>::_if(bool b)
{
    m_on = b;
    return *this;
}

NoMessage::NoMessage()
{
}

NoMessage::NoMessage(NoMessage&&)
{
}

NoMessage::~NoMessage()
{
}

template<typename T>
NoMessage& NoMessage::operator << (T)
{
    return *this;
}

NoMessage& NoMessage::_if(bool)
{
    return *this;
}

} // namespace internal

inline internal::LogMessage<internal::Critical> critical() {return internal::LogMessage<internal::Critical>();}
inline internal::LogMessage<internal::Error  >  error()    {return internal::LogMessage<internal::Error>();}
inline internal::LogMessage<internal::Warning>  warning()  {return internal::LogMessage<internal::Warning>();}
inline internal::LogMessage<internal::Info   >  info()     {return internal::LogMessage<internal::Info>();}

#if defined(DEBUG) || defined(_DEBUG)
    inline internal::LogMessage<internal::Debug> debug() {return internal::LogMessage<internal::Debug>();}
#else
    inline internal::NoMessage debug() {return internal::NoMessage();}
#endif

} // namespace Log
