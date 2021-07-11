#pragma once

#include <string>
#include <vector>
#include <any>
#include <iostream>
#include <sstream>
#include <iterator>

namespace opt {

class Option;

namespace internal {

inline bool is_param(const std::string& str);

template<typename T> inline T cast(int val);
template<typename T> inline T cast(float val);
template<typename T> inline T cast(double val);
template<typename T> inline T cast(const std::string& str);
template<typename T> inline T cast(const std::any& any);

template<typename T> inline std::string to_string(const T& val);
template<typename T> inline std::string to_string(const std::vector<T>& vec);

void print_space(int n);
std::vector<std::string> cut_space(const std::string& str);
std::vector<std::string> cut(const std::string& str, int n);
std::string extract_name(const std::string& str);

// see https://stackoverflow.com/a/2616912
const std::string black_foreground   = "30";
const std::string red_foreground     = "31";
const std::string green_foreground   = "32";
const std::string yellow_foreground  = "33";
const std::string blue_foreground    = "34";
const std::string magenta_foreground = "35";
const std::string cyan_foreground    = "36";
const std::string white_foreground   = "37";

const std::string black_background   = "40";
const std::string red_background     = "41";
const std::string green_background   = "42";
const std::string yellow_background  = "43";
const std::string blue_background    = "44";
const std::string magenta_background = "45";
const std::string cyan_background    = "46";
const std::string white_background   = "47";

const std::string reset              =  "0"; // (everything back to normal)
const std::string bold_bright        =  "1"; // (often a brighter shade of the same colour)
const std::string underline          =  "4";
const std::string inverse            =  "7"; // (swap foreground and background colours)
const std::string bold_bright_off    = "21";
const std::string underline_off      = "24";
const std::string inverse_off        = "27";

const std::string BLUE    = "\033[" + cyan_foreground + "m";
const std::string BOLD    = "\033[" + bold_bright + "m";
const std::string RESET   = "\033[" + reset + "m";
const std::string WARNING = "\033[" + yellow_foreground + "m" + "warning" + RESET;
const std::string ERROR   = "\033[" + red_foreground + "m" + "error" + RESET;

class OptionParameter
{
public:
    inline OptionParameter(Option& opt, const std::string& id,
                           const std::string& id2 = "");

    template<typename T>
    inline OptionParameter& set_default(const T& val);
    inline OptionParameter& set_default(const char* val);
    inline OptionParameter& set_required();
    inline OptionParameter& set_brief(const std::string& brief);

    template<typename T>
    inline operator T();
    inline operator bool();

    template<typename T>
    inline operator std::vector<T>();

    inline const std::string& id() const;
    inline const std::string& id2() const;
    inline const std::string& brief() const;
    inline bool               required() const;
    inline const std::any&    default_value() const;
    inline const std::string& default_str() const;
    inline const std::string& value_str() const;

    inline bool has_brief() const;
    inline bool has_id2() const;
    inline bool has_default() const;

    inline std::string help_header() const;
    inline std::string help_header_color() const;

protected:
    std::string m_id;
    std::string m_id2;
    std::string m_brief;
    bool        m_required;
    std::any    m_default;
    std::string m_default_str;
    std::string m_value_str;

private:
    Option& m_opt;
};

} // namespace internal

class Option
{
public:
    inline Option(int argc, char** argv, const std::string& brief = "");

    inline internal::OptionParameter& operator()(const std::string& id,
                                                 const std::string& id2 = "");

    inline bool ok();
    inline operator bool();

    inline void print() const;
    inline void print_help() const;

public:
    inline int size() const;
    inline const std::string& operator[](int i) const;

    inline bool has(const std::string& arg,
                    const std::string& arg2 = "") const;
    inline bool has(const std::string&  arg, int& idx) const;
    inline bool has(const std::string& arg,
                    const std::string& arg2, int& idx) const;
    inline int find(const std::string& arg,
                    const std::string& arg2 = "") const;

protected:
    std::vector<std::string> m_args;
    std::vector<bool>        m_used;
    std::string              m_brief;
    std::string              m_name;
    std::vector<std::string> m_warnings;
    std::vector<std::string> m_errors;

    std::vector<internal::OptionParameter> m_params;


    friend class internal::OptionParameter;
};

} // namespace opt

// -----------------------------------------------------------------------------

namespace opt {
namespace internal {

bool is_param(const std::string& str)
{
    return str.size() >= 2 && str[0] == '-' && !std::isdigit(str[1]);
}

template<> int         cast(int val){return val;}
template<> float       cast(int val){return val;}
template<> double      cast(int val){return val;}
template<> std::string cast(int val){return std::to_string(val);}

template<> int         cast(float val){return val;}
template<> float       cast(float val){return val;}
template<> double      cast(float val){return val;}
template<> std::string cast(float val){return std::to_string(val);}

template<> int         cast(double val){return val;}
template<> float       cast(double val){return val;}
template<> double      cast(double val){return val;}
template<> std::string cast(double val){return std::to_string(val);}

template<> int         cast(const std::string& str){return std::stoi(str);}
template<> float       cast(const std::string& str){return std::stof(str);}
template<> double      cast(const std::string& str){return std::stod(str);}
template<> std::string cast(const std::string& str){return str;}

template<typename T>
T cast(const std::any& any)
{
    static_assert(std::disjunction<
        std::is_same<T,int>,
        std::is_same<T,float>,
        std::is_same<T,double>,
        std::is_same<T,std::string>
        >::value);

         if(any.type() == typeid(int))         return cast<T>(std::any_cast<int>(any));
    else if(any.type() == typeid(float))       return cast<T>(std::any_cast<float>(any));
    else if(any.type() == typeid(double))      return cast<T>(std::any_cast<double>(any));
    else if(any.type() == typeid(std::string)) return cast<T>(std::any_cast<std::string>(any));
    return T();
}

template<typename T>
std::string to_string(const T& val)
{
    std::stringstream ss;
    ss << val;
    return ss.str();
}

template<typename T>
std::string to_string(const std::vector<T>& vec)
{
    constexpr char delimiter = ',';
    std::string str;
    for(const auto& val : vec)
    {
        if(!str.empty()) str += delimiter;
        str += to_string(val);
    }
    return str;
}


void print_space(int n)
{
    std::cout << std::string(n, ' ');
}

std::vector<std::string> cut_space(const std::string& str)
{
    std::istringstream iss(str);
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss),
              std::istream_iterator<std::string>(),
              std::back_inserter(tokens));
    return tokens;
}

std::vector<std::string> cut(const std::string& str, int n)
{
    std::vector<std::string> all = cut_space(str);
    std::vector<std::string> lines;

    lines.push_back("");
    for(const auto& str : all)
    {
        if(lines.back().empty())
        {
            lines.back() += str;
        }
        else if(int(lines.back().size() + str.size() + 1) <= n)
        {
            lines.back() += ' ' + str;
        }
        else
        {
            lines.push_back(str);
        }
    }
    return lines;
}

std::string extract_name(const std::string& str)
{
    const auto it = str.find_last_of('/');
    return it == std::string::npos ? str : str.substr(it+1);
}

OptionParameter::OptionParameter(Option& opt, const std::string& id,
                                 const std::string& id2) :
    m_id(id),
    m_id2(id2),
    m_brief(),
    m_required(false),
    m_default(),
    m_default_str(),
    m_value_str(),
    m_opt(opt)
{
}

template<typename T>
OptionParameter& OptionParameter::set_default(const T& val)
{
    static_assert(std::disjunction<
        std::is_same<T,int>,
        std::is_same<T,float>,
        std::is_same<T,double>,
        std::is_same<T,std::string>
        >::value);

    m_default     = val;
    m_default_str = to_string(val);
    return *this;
}

OptionParameter& OptionParameter::set_default(const char* val)
{
    m_default     = std::string(val);
    m_default_str = std::string(val);
    return *this;
}

OptionParameter& OptionParameter::set_required()
{
    m_required = true;
    return *this;
}

OptionParameter& OptionParameter::set_brief(const std::string& brief)
{
    m_brief = brief;
    return *this;
}

template<typename T>
OptionParameter::operator T()
{
    static_assert(std::disjunction<
        std::is_same<T,int>,
        std::is_same<T,float>,
        std::is_same<T,double>,
        std::is_same<T,std::string>
        >::value);

    int idx = -1;
    const bool found = m_opt.has(m_id, m_id2, idx);

    if(found)
    {
        if(m_opt.m_used[idx])
        {
            m_opt.m_warnings.push_back("parameter '-" + m_opt[idx] +
                                       "' appears twice");
        }

        if(idx+1 < m_opt.size())
        {
            if(!is_param(m_opt[idx+1]))
            {
                m_opt.m_used[idx]   = true;
                m_opt.m_used[idx+1] = true;
//                m_value_str = to_string(cast<T>(m_opt[idx+1]));
                m_value_str = m_opt[idx+1];
                return cast<T>(m_opt[idx+1]);
            }
            else
            {
                m_opt.m_errors.push_back("missing parameter value between '" +
                                         m_opt[idx]   + "' and '" +
                                         m_opt[idx+1] + "'");
                m_opt.m_used[idx] = true;
                return T();
            }
        }
        else
        {
            m_opt.m_errors.push_back("missing parameter value after '" +
                                     m_opt[idx]   + "'");
            m_opt.m_used[idx] = true;
            return T();
        }
    }
    else
    {
        if(m_required)
        {
            m_opt.m_errors.push_back("missing required parameter '-" +
                                     m_id + "'");
            return T();
        }
        else
        {
            if(m_default.has_value())
            {
                m_value_str = m_default_str;
                return cast<T>(m_default);
            }
            else
            {
                // enable this warning for safer default values
//                m_opt.m_warnings.push_back("default value missing for '-" +
//                                           m_id + "'");
                m_value_str = to_string(T());
                return T();
            }
        }
    }
}

template<typename T>
OptionParameter::operator std::vector<T>()
{
    static_assert(std::disjunction<
        std::is_same<T,int>,
        std::is_same<T,float>,
        std::is_same<T,double>,
        std::is_same<T,std::string>
        >::value);

    int idx = -1;
    const bool found = m_opt.has(m_id, m_id2, idx);

//    if(m_required)
//    {
//        //TODO support require property for vector
//        m_opt.m_warnings.push_back("'require' property not supported "
//                                   "for vectors");
//    }
    if(this->has_default())
    {
        //TODO support default values for vector
        m_opt.m_warnings.push_back("default values not supported "
                                   "for vectors");
    }

    if(found)
    {
        if(m_opt.m_used[idx])
        {
            m_opt.m_warnings.push_back("parameter '-" + m_opt[idx] +
                                       "' appears twice");
        }

        m_opt.m_used[idx] = true;

        std::vector<T> values;
        for(int j = idx+1; j < m_opt.size(); ++j)
        {
            if(is_param(m_opt[j])) break;

            m_opt.m_used[j] = true;
            values.push_back(cast<T>(m_opt[j]));
        }

        if(values.empty())
        {
            m_opt.m_errors.push_back("missing parameter value after '" +
                                     m_opt[idx]   + "'");
        }
        m_value_str = to_string(values);
        return values;
    }
    else
    {
        if(m_required)
        {
            m_opt.m_errors.push_back("missing required parameter '-" +
                                     m_id + "'");
            return std::vector<T>();
        }
        else
        {
            // enable this warning for safer default values
//                m_opt.m_warnings.push_back("default value missing for '-" +
//                                           m_id + "'");
            m_value_str = to_string(std::vector<T>());
            return std::vector<T>();
        }
    }
}

OptionParameter::operator bool()
{
    int idx = -1;
    const bool found = m_opt.has(m_id, m_id2, idx);

    if(found)
    {
        m_opt.m_used[idx] = true;
        m_value_str = "true";
        return true;
    }
    else
    {
        m_value_str = "false";
        return false;
    }
}

const std::string& OptionParameter::id() const
{
    return m_id;
}

const std::string& OptionParameter::id2() const
{
    return m_id2;
}

const std::string& OptionParameter::brief() const
{
    return m_brief;
}

bool OptionParameter::required() const
{
    return m_required;
}

const std::any& OptionParameter::default_value() const
{
    return m_default;
}

const std::string& OptionParameter::default_str() const
{
    return m_default_str;
}

const std::string& OptionParameter::value_str() const
{
    return m_value_str;
}

bool OptionParameter::has_brief() const
{
    return !m_brief.empty();
}

bool OptionParameter::has_id2() const
{
    return !m_id2.empty();
}

bool OptionParameter::has_default() const
{
    return !m_default_str.empty();
}

std::string OptionParameter::help_header() const
{
    return '-'+this->id() + (this->has_id2() ? ", -"+this->id2() : "");
}

std::string OptionParameter::help_header_color() const
{
    return BOLD + '-'+this->id() + RESET + (this->has_id2() ? ", " + BOLD + "-"+this->id2() + RESET : "");
}

} // namespace internal

Option::Option(int argc, char** argv, const std::string& brief) :
    m_args(argc-1),
    m_used(argc-1, false),
    m_brief(brief),
    m_name(internal::extract_name(argv[0])),
    m_warnings(),
    m_errors(),
    m_params()
{
    for(int i = 1; i < argc; ++i)
    {
        m_args[i-1] = argv[i];
    }
}

internal::OptionParameter& Option::operator()(const std::string& id,
                                             const std::string& id2)
{
    for(const auto& p : m_params)
    {
        if(p.id() == id)
        {
            m_errors.push_back("parameter '" + id + "' already exists");
        }
        if(!id2.empty() && p.id2() == id2)
        {
            m_errors.push_back("parameter '" + id2 + "' already exists");
        }
    }
    m_params.push_back(internal::OptionParameter(*this, id, id2));
    return m_params.back();
}

bool Option::ok()
{
    const bool help = this->operator()("h", "-help").set_brief("Display this help.");
    if(help)
    {
        this->print_help();
        return false;
    }
    else
    {
        for(const auto& w : m_warnings)
        {
            std::cout << "opt ["+internal::WARNING+"]: " << w << std::endl;
        }
        for (int i = 0; i < this->size(); ++i)
        {
            if(!m_used[i])
            {
                std::cout << "opt ["+internal::WARNING+"]: argument '" << m_args[i]
                          << "' unused" << std::endl;
            }
        }
        for(const auto& e : m_errors)
        {
            std::cout << "opt ["+internal::ERROR+"]: " << e << std::endl;
        }
        return m_errors.empty();
    }
}

Option::operator bool()
{
    return this->ok();
}

void Option::print() const
{
    for(const auto& p : m_params)
    {
        if(p.id() != "h")
        {
            std::cout << p.id() << " = " << p.value_str() << '\n';
        }
    }
}

void Option::print_help() const
{
    int maxcol = 0;
    for(const auto& p : m_params)
    {
        maxcol = std::max<int>(maxcol, p.help_header().size());
    }

    std::cout << internal::BOLD << m_name << internal::RESET << '\n' << '\n';
    if(!m_brief.empty())
    {
        const auto brief_cut = internal::cut(m_brief, maxcol + 2 + 40);
        for(const auto& line : brief_cut)
        {
            std::cout << line << '\n';
        }
        std::cout << '\n';
    }

    for(const auto& p : m_params)
    {
        const auto header = p.help_header();
        const auto header_col = p.help_header_color();
        std::cout << header_col;
        internal::print_space(maxcol+2-header.size());

        const bool has_brief   = p.has_brief();
        const bool is_required = p.required();
        const bool has_default = p.has_default();
        const bool has_prop    = is_required || has_default;

        std::string text;
        if(has_brief) text += p.brief();
        if(has_brief && has_prop)  text += ' ';
        if(is_required) text += "[" + internal::BLUE + "required" + internal::RESET + "]";
        if(has_default) text += "[default=" + internal::BLUE + p.default_str() + internal::RESET + "]";

        const auto text_cut = internal::cut(text, 40);

        if(text_cut.empty())
        {
            std::cout << '\n';
        }
        else
        {
            std::cout << text_cut.front() << '\n';
            for(int i = 1; i < int(text_cut.size()); ++i)
            {
                internal::print_space(maxcol+2);
                std::cout << text_cut[i] << '\n';
            }
        }
    }
}

int Option::size() const
{
    return m_args.size();
}

const std::string& Option::operator[](int i) const
{
    return m_args[i];
}

bool Option::has(const std::string& arg, const std::string& arg2) const
{
    const auto idx = this->find(arg, arg2);
    return idx < this->size();
}

bool Option::has(const std::string& arg, int& idx) const
{
    idx = this->find(arg);
    return idx < this->size();
}

bool Option::has(const std::string& arg, const std::string& arg2,
                 int& idx) const
{
    idx = this->find(arg, arg2);
    return idx < this->size();
}

int Option::find(const std::string& arg, const std::string& arg2) const
{
    int idx = 0;
    for(idx = 0; idx < this->size(); ++idx)
    {
        if(this->operator[](idx) == '-'+arg ||
           (!arg2.empty() && this->operator[](idx) == '-'+arg2) )
        {
            return idx;
        }
    }
    return idx;
}

} // namespace opt
