#pragma once

#include <chrono>
#include <cmath>
#include <iostream>

namespace prog {

class Progress
{
public:
    inline Progress();
    inline Progress(int step_count);
    inline Progress(int step_count, int percentage_increment);
    inline Progress(int step_count, int percentage_increment, bool enable);
    inline Progress(int step_count, bool enable);

public:
    inline Progress& operator ++();
    inline Progress& incr(const char* prefix);

protected:
    inline unsigned long long int time() const;
    inline static std::string to_string(long double millisec);

protected:
    int  m_step;
    int  m_step_count;
    int  m_step_increment;
    int  m_percentage_increment;
    int  m_next_step;
    int  m_next_percentage;
    bool m_enabled;
    std::chrono::high_resolution_clock::time_point m_start;
};


} // namespace prog

// -----------------------------------------------------------------------------

namespace prog {

Progress::Progress(int step_count, int percentage_increment, bool enable) :
    m_step(0),
    m_step_count(step_count),
    m_step_increment(std::ceil(double(step_count)/(100.0/percentage_increment))),
    m_percentage_increment(percentage_increment),
    m_next_step(m_step_increment),
    m_next_percentage(percentage_increment),
    m_enabled(enable),
    m_start(std::chrono::high_resolution_clock::now())
{
}

Progress::Progress() :
    Progress(0, 0, false)
{
}

Progress::Progress(int step_count) :
    Progress(step_count, 10, true)
{
}

Progress::Progress(int step_count, int percentage_increment) :
    Progress(step_count, percentage_increment, true)
{
}

Progress::Progress(int step_count, bool enable) :
    Progress(step_count, 10, enable)
{
}

Progress& Progress::operator ++()
{
    return this->incr("");
}

Progress& Progress::incr(const char* prefix)
{
    if(m_enabled)
    {
        #pragma omp critical (prog_Progress_incr)
        {
            ++m_step;
            if(m_step == m_step_count)
            {
                const long double total_time = this->time();
                std::cout << prefix << m_step << "/" << m_step_count
                          << " [" << m_next_percentage << "%]"
                          << " total time = " << Progress::to_string(total_time)
                          << std::endl;
            }
            else if(m_step >= m_next_step)
            {
                const long double elapsed_time = this->time();
                const long double remaining_time = elapsed_time * (100. - m_next_percentage) / m_next_percentage;

                std::cout << prefix << m_step << "/" << m_step_count
                          << " [" << m_next_percentage <<"%]"
                          << " remaining time = " << Progress::to_string(remaining_time)
                          << std::endl;

                // increment current state
                m_next_step       += m_step_increment;
                m_next_percentage += m_percentage_increment;
            }
        }
    }
    return *this;
}

unsigned long long int Progress::time() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - m_start).count();
}

std::string Progress::to_string(long double millisec)
{
    int day = int(millisec/86400000);
    if(day > 0)
    {
        millisec -= day*86400000;
        int hour  = int(millisec/3600000);
        // print day + hour
        return std::to_string(day) + "d " + std::to_string(hour) + "h";
    }

    int hour = int(millisec/3600000);
    if(hour > 0)
    {
        millisec -= hour*3600000;
        int min   = int(millisec/60000);
        // print hour + min
        return std::to_string(hour) + "h " + std::to_string(min) + "min";
    }

    int min = int(millisec/60000);
    if(min > 0)
    {
        millisec -= min*60000;
        int sec   = int(millisec/1000);
        // print min + sec
        return std::to_string(min) + "min " + std::to_string(sec) + "s";
    }

    int sec = int(millisec/1000);
    if(sec > 0)
    {
        millisec -= sec*1000;
        // print sec + milli
        return std::to_string(sec) + "s " + std::to_string(int(millisec)) + "ms";
    }

    // print milli
    return std::to_string(int(millisec)) + "ms";
}

} // namespace prog
