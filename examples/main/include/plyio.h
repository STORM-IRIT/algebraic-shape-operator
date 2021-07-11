#pragma once

#include <string>
#include <vector>
#include <functional>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>

#ifndef PLYIO_ASSERT
#include <assert.h>
#define PLYIO_ASSERT(expr) assert(expr)
#endif

namespace plyio {

// Types -----------------------------------------------------------------------

using char_t   = signed char;
using uchar_t  = unsigned char;
using short_t  = int16_t;
using ushort_t = uint16_t;
using int_t    = int32_t;
using uint_t   = uint32_t;
using float_t  = float;
using double_t = double;

enum Type
{
    type_char,
    type_uchar,
    type_short,
    type_ushort,
    type_int,
    type_uint,
    type_float,
    type_double,
    type_unkown
};

//! \brief check if type is integral (using std::is_integral)
//! \return false if type is float or double, true otherwise
inline bool is_integral(Type type);

// Reader ----------------------------------------------------------------------

class Reader
{
public:
    inline Reader();

    //! \brief ReadingFuncT must be 'void(int i, T value)'
    template<typename ReadingFuncT>
    inline void read(ReadingFuncT&& f);

    //! \brief ReadingListFuncT must be
    //! 'void(int i, const std::vector<T>& values)'
    template<typename ReadingListFuncT>
    inline void read_list(ReadingListFuncT&& f);

protected:
    template<typename SourceType>
    inline void read(int i, SourceType value) const;
    template<typename SourceType>
    inline void read(int i, const std::vector<SourceType> values) const;

protected:
    template<typename T>
    using ReadingFunction = std::function<void(int i, T value)>;
    template<typename T>
    using ReadingFunctionList = std::function<void(int i, const std::vector<T>& values)>;

protected:
    // set by read/read_list
    Type m_target_type;

    ReadingFunction<char_t>   m_read_char_f;
    ReadingFunction<uchar_t>  m_read_uchar_f;
    ReadingFunction<short_t>  m_read_short_f;
    ReadingFunction<ushort_t> m_read_ushort_f;
    ReadingFunction<int_t>    m_read_int_f;
    ReadingFunction<uint_t>   m_read_uint_f;
    ReadingFunction<float_t>  m_read_float_f;
    ReadingFunction<double_t> m_read_double_f;

    ReadingFunctionList<char_t>   m_read_char_list_f;
    ReadingFunctionList<uchar_t>  m_read_uchar_list_f;
    ReadingFunctionList<short_t>  m_read_short_list_f;
    ReadingFunctionList<ushort_t> m_read_ushort_list_f;
    ReadingFunctionList<int_t>    m_read_int_list_f;
    ReadingFunctionList<uint_t>   m_read_uint_list_f;
    ReadingFunctionList<float_t>  m_read_float_list_f;
    ReadingFunctionList<double_t> m_read_double_list_f;

    friend class PLYReader;
};

// Reading Info ----------------------------------------------------------------

struct ReadingProperty
{
    bool         is_list;
    std::string  name;
    Type         type;
    Type         size_type;

    Reader       reader;
};

struct ReadingElement
{
    std::string name;
    int count;
    std::vector<ReadingProperty> properties;
};

namespace internal {

// Error -----------------------------------------------------------------------

class ErrorManager
{
public:
    inline bool has_error() const;
    inline const std::vector<std::string>& errors() const;
    inline void print_errors() const;

protected:
    std::vector<std::string> m_errors;
};

} // namespace internal

// PLYReader -------------------------------------------------------------------

//!
//! \brief The PLYReader class is the main class for reading ply files.
//!
class PLYReader : public internal::ErrorManager
{
public:
    inline PLYReader();

    // Reading -----------------------------------------------------------------
public:
    inline bool read_header(const std::string& filename);
    inline bool read_header(std::istream& is);
    inline bool read_body(const std::string& filename);
    inline bool read_body(std::istream& is);

    // Internal reading --------------------------------------------------------
protected:
    inline bool read_body_ascii(std::istream& is);
    inline bool read_body_binary(std::istream& is);

    // Reading Info Accessors --------------------------------------------------
public:
    inline bool ascii() const;
    inline bool binary() const;
    inline bool binary_little_endian() const;
    inline bool binary_big_endian() const;
    inline int  version() const;

    inline bool has_element(const std::string& element_name) const;
    inline bool has_property(const std::string& element_name, const std::string& property_name) const;

    inline ReadingElement& element(const std::string& element_name);
    inline ReadingProperty& property(const std::string& element_name, const std::string& property_name);

    inline const std::vector<std::string>& comments() const;

    inline const std::vector<ReadingElement>& elements() const;
    inline       std::vector<ReadingElement>& elements();

    inline int element_count(const std::string& element_name) const;

    // Data --------------------------------------------------------------------
protected:
    bool m_ascii;
    bool m_binary_little_endian;
    bool m_binary_big_endian;
    int  m_version;

    std::vector<std::string> m_comments;
    std::vector<ReadingElement> m_elements;
};


// Writer ----------------------------------------------------------------------

template<typename T>
using WritingFunction = std::function<T(int i)>;
template<typename T>
using SizeFunction = std::function<T(int i)>;
template<typename T>
using WritingFunctionList = std::function<T(int i, int k)>;

using WritingFunctionChar   = WritingFunction<char_t>;
using WritingFunctionUchar  = WritingFunction<uchar_t>;
using WritingFunctionShort  = WritingFunction<short_t>;
using WritingFunctionUshort = WritingFunction<ushort_t>;
using WritingFunctionInt    = WritingFunction<int_t>;
using WritingFunctionUint   = WritingFunction<uint_t>;
using WritingFunctionFloat  = WritingFunction<float_t>;
using WritingFunctionDouble = WritingFunction<double_t>;

using SizeFunctionChar   = SizeFunction<char_t>;
using SizeFunctionUchar  = SizeFunction<uchar_t>;
using SizeFunctionShort  = SizeFunction<short_t>;
using SizeFunctionUshort = SizeFunction<ushort_t>;
using SizeFunctionInt    = SizeFunction<int_t>;
using SizeFunctionUint   = SizeFunction<uint_t>;
using SizeFunctionFloat  = SizeFunction<float_t>;
using SizeFunctionDouble = SizeFunction<double_t>;

using WritingFunctionListChar   = WritingFunctionList<char_t>;
using WritingFunctionListUchar  = WritingFunctionList<uchar_t>;
using WritingFunctionListShort  = WritingFunctionList<short_t>;
using WritingFunctionListUshort = WritingFunctionList<ushort_t>;
using WritingFunctionListInt    = WritingFunctionList<int_t>;
using WritingFunctionListUint   = WritingFunctionList<uint_t>;
using WritingFunctionListFloat  = WritingFunctionList<float_t>;
using WritingFunctionListDouble = WritingFunctionList<double_t>;

class Writer
{
public:
    inline Writer();

    inline Writer(const WritingFunctionChar&   f) : m_write_char_f  (f){}
    inline Writer(const WritingFunctionUchar&  f) : m_write_uchar_f (f){}
    inline Writer(const WritingFunctionShort&  f) : m_write_short_f (f){}
    inline Writer(const WritingFunctionUshort& f) : m_write_ushort_f(f){}
    inline Writer(const WritingFunctionInt&    f) : m_write_int_f   (f){}
    inline Writer(const WritingFunctionUint&   f) : m_write_uint_f  (f){}
    inline Writer(const WritingFunctionFloat&  f) : m_write_float_f (f){}
    inline Writer(const WritingFunctionDouble& f) : m_write_double_f(f){}

    inline Writer(const SizeFunctionChar&   f, const WritingFunctionListChar&   g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionUchar&  f, const WritingFunctionListUchar&  g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionShort&  f, const WritingFunctionListShort&  g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionUshort& f, const WritingFunctionListUshort& g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionInt&    f, const WritingFunctionListInt&    g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionUint&   f, const WritingFunctionListUint&   g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionFloat&  f, const WritingFunctionListFloat&  g) {set_size(f); set_prop(g);}
    inline Writer(const SizeFunctionDouble& f, const WritingFunctionListDouble& g) {set_size(f); set_prop(g);}

protected:
    inline void set_size(const SizeFunctionChar&   f) {m_size_char_f   = f;}
    inline void set_size(const SizeFunctionUchar&  f) {m_size_uchar_f  = f;}
    inline void set_size(const SizeFunctionShort&  f) {m_size_short_f  = f;}
    inline void set_size(const SizeFunctionUshort& f) {m_size_ushort_f = f;}
    inline void set_size(const SizeFunctionInt&    f) {m_size_int_f    = f;}
    inline void set_size(const SizeFunctionUint&   f) {m_size_uint_f   = f;}
    inline void set_size(const SizeFunctionFloat&  f) {m_size_float_f  = f;}
    inline void set_size(const SizeFunctionDouble& f) {m_size_double_f = f;}

    inline void set_prop(const WritingFunctionListChar&   g) {m_write_char_list_f   = g;}
    inline void set_prop(const WritingFunctionListUchar&  g) {m_write_uchar_list_f  = g;}
    inline void set_prop(const WritingFunctionListShort&  g) {m_write_short_list_f  = g;}
    inline void set_prop(const WritingFunctionListUshort& g) {m_write_ushort_list_f = g;}
    inline void set_prop(const WritingFunctionListInt&    g) {m_write_int_list_f    = g;}
    inline void set_prop(const WritingFunctionListUint&   g) {m_write_uint_list_f   = g;}
    inline void set_prop(const WritingFunctionListFloat&  g) {m_write_float_list_f  = g;}
    inline void set_prop(const WritingFunctionListDouble& g) {m_write_double_list_f = g;}

protected:
    inline char_t   write_char  (int i) const;
    inline uchar_t  write_uchar (int i) const;
    inline short_t  write_short (int i) const;
    inline ushort_t write_ushort(int i) const;
    inline int_t    write_int   (int i) const;
    inline uint_t   write_uint  (int i) const;
    inline float_t  write_float (int i) const;
    inline double_t write_double(int i) const;

    inline char_t   size_char  (int i) const;
    inline uchar_t  size_uchar (int i) const;
    inline short_t  size_short (int i) const;
    inline ushort_t size_ushort(int i) const;
    inline int_t    size_int   (int i) const;
    inline uint_t   size_uint  (int i) const;
    inline float_t  size_float (int i) const;
    inline double_t size_double(int i) const;

    inline char_t   write_char  (int i, int k) const;
    inline uchar_t  write_uchar (int i, int k) const;
    inline short_t  write_short (int i, int k) const;
    inline ushort_t write_ushort(int i, int k) const;
    inline int_t    write_int   (int i, int k) const;
    inline uint_t   write_uint  (int i, int k) const;
    inline float_t  write_float (int i, int k) const;
    inline double_t write_double(int i, int k) const;

protected:
    WritingFunctionChar   m_write_char_f;
    WritingFunctionUchar  m_write_uchar_f;
    WritingFunctionShort  m_write_short_f;
    WritingFunctionUshort m_write_ushort_f;
    WritingFunctionInt    m_write_int_f;
    WritingFunctionUint   m_write_uint_f;
    WritingFunctionFloat  m_write_float_f;
    WritingFunctionDouble m_write_double_f;

    SizeFunctionChar   m_size_char_f;
    SizeFunctionUchar  m_size_uchar_f;
    SizeFunctionShort  m_size_short_f;
    SizeFunctionUshort m_size_ushort_f;
    SizeFunctionInt    m_size_int_f;
    SizeFunctionUint   m_size_uint_f;
    SizeFunctionFloat  m_size_float_f;
    SizeFunctionDouble m_size_double_f;

    WritingFunctionListChar   m_write_char_list_f;
    WritingFunctionListUchar  m_write_uchar_list_f;
    WritingFunctionListShort  m_write_short_list_f;
    WritingFunctionListUshort m_write_ushort_list_f;
    WritingFunctionListInt    m_write_int_list_f;
    WritingFunctionListUint   m_write_uint_list_f;
    WritingFunctionListFloat  m_write_float_list_f;
    WritingFunctionListDouble m_write_double_list_f;

    friend class PLYWriter;
};

// Writing Info ----------------------------------------------------------------

struct WritingProperty
{
    std::string  name;
    Type         type;
    bool         is_list;
    Type         size_type;

    Writer       writer;
};

struct WritingElement
{
    std::string name;
    int count;
    std::vector<WritingProperty> properties;
};

// PLYWriter -------------------------------------------------------------------

//!
//! \brief The PLYWriter class is the main class for writing ply files.
//!
class PLYWriter : public internal::ErrorManager
{
public:
    inline PLYWriter();

    // Writing -----------------------------------------------------------------
public:
    inline bool write(const std::string& filename);
    inline bool write(std::ostream& os);

    // Internal writing --------------------------------------------------------
protected:
    inline bool write_header(std::ostream& os);
    inline bool write_body(std::ostream& os);

    inline bool write_body_ascii(std::ostream& os);
    inline bool write_body_binary(std::ostream& os);

    // Writing Info Modifiers --------------------------------------------------
public:
    inline void set_ascii();
    inline void set_binary();
    inline void set_binary_little_endian();
    inline void set_binary_big_endian();
    inline void set_version(int version);

    inline void add_comment(const std::string& comment);

    inline WritingElement& add_element(const std::string& element_name, int element_count);

    //! \brief GetPropFuncT must be 'T(int i)'
    template<typename GetPropFuncT>
    inline void add_property(const std::string& element_name, const std::string& property_name, GetPropFuncT&& get_prop_f);

    //! \brief GetSizeFuncT must be 'int()' and GetPropFuncT must be 'T(int i, int k)'
    template<typename GetSizeFuncT, typename GetPropFuncT>
    inline void add_property(const std::string& element_name, const std::string& property_name, GetSizeFuncT&& get_size_f, GetPropFuncT&& get_prop_f);

    // Data --------------------------------------------------------------------
protected:
    bool m_ascii;
    bool m_binary_little_endian;
    bool m_binary_big_endian;
    int  m_version;

    std::vector<std::string> m_comments;
    std::vector<WritingElement> m_elements;
};

namespace internal {

// Type ------------------------------------------------------------------------

template<typename T> struct is_supported : public std::false_type{};
template<> struct is_supported<char_t  > : public std::true_type{};
template<> struct is_supported<uchar_t > : public std::true_type{};
template<> struct is_supported<short_t > : public std::true_type{};
template<> struct is_supported<ushort_t> : public std::true_type{};
template<> struct is_supported<int_t   > : public std::true_type{};
template<> struct is_supported<uint_t  > : public std::true_type{};
template<> struct is_supported<float_t > : public std::true_type{};
template<> struct is_supported<double_t> : public std::true_type{};

inline Type to_type(const std::string& str);
inline std::string to_string(Type type);

template<typename T> Type get_type();
template<> Type get_type<char_t  >();
template<> Type get_type<uchar_t >();
template<> Type get_type<short_t >();
template<> Type get_type<ushort_t>();
template<> Type get_type<int_t   >();
template<> Type get_type<uint_t  >();
template<> Type get_type<float_t >();
template<> Type get_type<double_t>();

template<typename SourceType, typename TargetType>
inline void cast(SourceType from, TargetType& to);
template<typename SourceType, typename TargetType>
inline void cast_list(const std::vector<SourceType>& from, std::vector<TargetType>& to);

// get type of second argument of function 'void(int,T)'
// see see https://stackoverflow.com/questions/6512019/can-we-get-the-type-of-a-lambda-argument
template<typename Ret, typename Arg1, typename Arg2>
Arg2 argument_helper_2(Ret(*) (Arg1, Arg2));
template<typename Ret, typename F, typename Arg1, typename Arg2>
Arg2 argument_helper_2(Ret(F::*) (Arg1, Arg2));
template<typename Ret, typename F, typename Arg1, typename Arg2>
Arg2 argument_helper_2(Ret(F::*) (Arg1, Arg2) const);
template <typename F>
decltype(argument_helper_2(&F::operator())) argument_helper_2(F);
template <typename T>
using argument_type_2 = decltype(argument_helper_2(std::declval<T>()));

template<typename Ret, typename Arg1, typename Arg2>
Arg2 argument_list_helper_2(Ret(*) (Arg1, const std::vector<Arg2>&));
template<typename Ret, typename F, typename Arg1, typename Arg2>
Arg2 argument_list_helper_2(Ret(F::*) (Arg1, const std::vector<Arg2>&));
template<typename Ret, typename F, typename Arg1, typename Arg2>
Arg2 argument_list_helper_2(Ret(F::*) (Arg1, const std::vector<Arg2>&) const);
template <typename F>
decltype(argument_list_helper_2(&F::operator())) argument_list_helper_2(F);
template <typename T>
using argument_list_type_2 = decltype(argument_list_helper_2(std::declval<T>()));

// String ----------------------------------------------------------------------

//! \brief to_tokens extracts tokens delimited by space in a string
inline std::vector<std::string> to_tokens(const std::string& str);

} // namespace internal

} // namespace plyio













////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////











namespace plyio {

// Types -----------------------------------------------------------------------

bool is_integral(Type type)
{
    switch(type)
    {
    case type_char:   return std::is_integral<char_t  >::value;
    case type_uchar:  return std::is_integral<uchar_t >::value;
    case type_short:  return std::is_integral<short_t >::value;
    case type_ushort: return std::is_integral<ushort_t>::value;
    case type_int:    return std::is_integral<int_t   >::value;
    case type_uint:   return std::is_integral<uint_t  >::value;
    case type_float:  return std::is_integral<float_t >::value;
    case type_double: return std::is_integral<double_t>::value;
    default:          assert(false); return false;
    }
}

// Reader ----------------------------------------------------------------------

// all the functions are initialized to a function that does nothing
// in case the user is not interesting in some properties
Reader::Reader() :
    m_target_type(type_unkown),
    m_read_char_f([](int,char_t){}),
    m_read_uchar_f([](int,uchar_t){}),
    m_read_short_f([](int,short_t){}),
    m_read_ushort_f([](int,ushort_t){}),
    m_read_int_f([](int,int_t){}),
    m_read_uint_f([](int,uint_t){}),
    m_read_float_f([](int,float_t){}),
    m_read_double_f([](int,double_t){}),
    m_read_char_list_f([](int,const std::vector<char_t>&){}),
    m_read_uchar_list_f([](int,const std::vector<uchar_t>&){}),
    m_read_short_list_f([](int,const std::vector<short_t>&){}),
    m_read_ushort_list_f([](int,const std::vector<ushort_t>&){}),
    m_read_int_list_f([](int,const std::vector<int_t>&){}),
    m_read_uint_list_f([](int,const std::vector<uint_t>&){}),
    m_read_float_list_f([](int,const std::vector<float_t>&){}),
    m_read_double_list_f([](int,const std::vector<double_t>&){})
{
}

template<typename ReadingFuncT>
void Reader::read(ReadingFuncT&& f)
{
    using TargetType = internal::argument_type_2<ReadingFuncT>;
    static_assert(internal::is_supported<TargetType>::value, "Unsupported type");
    m_target_type = internal::get_type<TargetType>();

    if constexpr(std::is_same<TargetType, char_t  >::value) {m_read_char_f   = f; return;}
    if constexpr(std::is_same<TargetType, uchar_t >::value) {m_read_uchar_f  = f; return;}
    if constexpr(std::is_same<TargetType, short_t >::value) {m_read_short_f  = f; return;}
    if constexpr(std::is_same<TargetType, ushort_t>::value) {m_read_ushort_f = f; return;}
    if constexpr(std::is_same<TargetType, int_t   >::value) {m_read_int_f    = f; return;}
    if constexpr(std::is_same<TargetType, uint_t  >::value) {m_read_uint_f   = f; return;}
    if constexpr(std::is_same<TargetType, float_t >::value) {m_read_float_f  = f; return;}
    if constexpr(std::is_same<TargetType, double_t>::value) {m_read_double_f = f; return;}
}

template<typename ReadingFuncT>
void Reader::read_list(ReadingFuncT&& f)
{
    using TargetType = internal::argument_list_type_2<ReadingFuncT>;
    static_assert(internal::is_supported<TargetType>::value, "Unsupported type");
    m_target_type = internal::get_type<TargetType>();

    if constexpr(std::is_same<TargetType, char_t  >::value) {m_read_char_list_f    = f; return;}
    if constexpr(std::is_same<TargetType, uchar_t >::value) {m_read_uchar_list_f   = f; return;}
    if constexpr(std::is_same<TargetType, short_t >::value) {m_read_short_list_f   = f; return;}
    if constexpr(std::is_same<TargetType, ushort_t>::value) {m_read_ushort_list_f  = f; return;}
    if constexpr(std::is_same<TargetType, int_t   >::value) {m_read_int_list_f     = f; return;}
    if constexpr(std::is_same<TargetType, uint_t  >::value) {m_read_uint_list_f    = f; return;}
    if constexpr(std::is_same<TargetType, float_t >::value) {m_read_float_list_f   = f; return;}
    if constexpr(std::is_same<TargetType, double_t>::value) {m_read_double_list_f  = f; return;}
}

template<typename SourceType>
void Reader::read(int i, SourceType value) const
{
    char_t   val0;
    uchar_t  val1;
    short_t  val2;
    ushort_t val3;
    int_t    val4;
    uint_t   val5;
    float_t  val6;
    double_t val7;

    switch (m_target_type) {
    case type_char:       internal::cast(value,val0); m_read_char_f  (i,val0); break;
    case type_uchar:      internal::cast(value,val1); m_read_uchar_f (i,val1); break;
    case type_short:      internal::cast(value,val2); m_read_short_f (i,val2); break;
    case type_ushort:     internal::cast(value,val3); m_read_ushort_f(i,val3); break;
    case type_int:        internal::cast(value,val4); m_read_int_f   (i,val4); break;
    case type_uint:       internal::cast(value,val5); m_read_uint_f  (i,val5); break;
    case type_float:      internal::cast(value,val6); m_read_float_f (i,val6); break;
    case type_double:     internal::cast(value,val7); m_read_double_f(i,val7); break;
    default:              assert(false);
    }
}

template<typename SourceType>
void Reader::read(int i, const std::vector<SourceType> values) const
{
    std::vector<char_t>   vec0;
    std::vector<uchar_t>  vec1;
    std::vector<short_t>  vec2;
    std::vector<ushort_t> vec3;
    std::vector<int_t>    vec4;
    std::vector<uint_t>   vec5;
    std::vector<float_t>  vec6;
    std::vector<double_t> vec7;

    switch (m_target_type) {
    case type_char:       internal::cast_list(values,vec0); m_read_char_list_f  (i,vec0); break;
    case type_uchar:      internal::cast_list(values,vec1); m_read_uchar_list_f (i,vec1); break;
    case type_short:      internal::cast_list(values,vec2); m_read_short_list_f (i,vec2); break;
    case type_ushort:     internal::cast_list(values,vec3); m_read_ushort_list_f(i,vec3); break;
    case type_int:        internal::cast_list(values,vec4); m_read_int_list_f   (i,vec4); break;
    case type_uint:       internal::cast_list(values,vec5); m_read_uint_list_f  (i,vec5); break;
    case type_float:      internal::cast_list(values,vec6); m_read_float_list_f (i,vec6); break;
    case type_double:     internal::cast_list(values,vec7); m_read_double_list_f(i,vec7); break;
    default:              assert(false);
    }
}

namespace internal {

// Error -----------------------------------------------------------------------

bool ErrorManager::has_error() const
{
    return not m_errors.empty();
}

const std::vector<std::string>& ErrorManager::errors() const
{
    return m_errors;
}

void ErrorManager::print_errors() const
{
    for(const auto& error : m_errors)
    {
        std::cerr << error << std::endl;
    }
}

} // namespace internal

// PLYReader -------------------------------------------------------------------

PLYReader::PLYReader() :
    m_ascii(false),
    m_binary_little_endian(false),
    m_binary_big_endian(false),
    m_version(0),
    m_comments(0),
    m_elements(0)
{
}

// Reading ---------------------------------------------------------------------

bool PLYReader::read_header(const std::string& filename)
{
    m_errors.clear();
    std::ifstream ifs(filename);
    if(ifs.is_open())
    {
        return this->read_header(ifs);
    }
    else
    {
        m_errors.push_back("Failed to open file '" + filename + "'");
        return false;
    }
}

bool PLYReader::read_header(std::istream& is)
{
    // reset
    m_ascii = false;
    m_binary_little_endian = true;
    m_binary_big_endian = false;
    m_version = 0;
    m_comments.clear();
    m_elements.clear();
    m_errors.clear();

    auto line_num = 1;
    std::string line;

    std::getline(is, line);
    if(line != "ply")
    {
        m_errors.push_back("Line " + std::to_string(line_num) +
                           ": expected 'ply', found '" + line + "'");
        return false;
    }

    auto end_header_found = false;
    while(std::getline(is, line))
    {
        ++line_num;
        const auto tokens = internal::to_tokens(line);

        if(tokens.empty())
        {
            continue; // skip empty lines
        }
        else if(line == "end_header")
        {
            end_header_found = true;
            break;
        }
        else if(tokens.front() == "format")
        {
            if(tokens.size() != 3) // format ascii 1.0
            {
                m_errors.push_back("Line " + std::to_string(line_num) +
                                   ": expected 3 tokens (e.g. 'format ascii 1.0')," +
                                   " found " + std::to_string(tokens.size()));
                return false;
            }
            m_ascii                 = tokens[1] == "ascii";
            m_binary_little_endian  = tokens[1] == "binary_little_endian";
            m_binary_big_endian     = tokens[1] == "binary_big_endian";
            if(not (m_ascii || m_binary_little_endian || m_binary_big_endian))
            {
                m_errors.push_back("Line " + std::to_string(line_num) +
                                   ": 'ascii', 'binary_big_endian', or 'binary_little_endian' required,"
                                   " found '" + tokens[1] + "'");
                return false;
            }
            m_version = std::stoi(tokens[2]); //TODO version is not one single integer
        }
        else if(tokens.front() == "comment")
        {
            m_comments.emplace_back(line.substr(8));
        }
        else if(tokens.front() == "element")
        {
            if(tokens.size() != 3) // element vertex 128
            {
                m_errors.push_back("Line " + std::to_string(line_num) +
                                   ": expected 3 tokens (e.g. 'element vertex 128')," +
                                   " found " + std::to_string(tokens.size()));
                return false;
            }
            const auto& name = tokens[1];
            const auto count = std::stoi(tokens[2]);
            m_elements.push_back({name,count,{/*empty properties*/}});
        }
        else if(tokens.front() == "property")
        {
            if(m_elements.empty())
            {
                m_errors.push_back("Line " + std::to_string(line_num) +
                                   ": element required before property declaration");
                return false;
            }
            else if(tokens.size() != 3 && // property float x
                    tokens.size() != 5)   // property list int int vertex_indices
            {
                m_errors.push_back("Line " + std::to_string(line_num) +
                                   ": expected 3 or 5 tokens (e.g. 'property float x'" +
                                   " or 'property list int int vertex_indices')," +
                                   " found " + std::to_string(tokens.size()));
                return false;
            }

            if(tokens[1] == "list")
            {
                if(tokens.size() != 5) // property list int int vertex_indices
                {
                    m_errors.push_back("Line " + std::to_string(line_num) +
                                       ": expected 5 tokens (e.g. 'property list int int vertex_indices')," +
                                       " found " + std::to_string(tokens.size()));
                    return false;
                }
                constexpr auto is_list = true;
                const auto type = internal::to_type(tokens[3]);
                const auto& name = tokens[4];
                const auto size_type = internal::to_type(tokens[2]);
                m_elements.back().properties.push_back({is_list,name,type,size_type,{/*reader*/}});
            }
            else
            {
                if(tokens.size() != 3) // property float x
                {
                    m_errors.push_back("Line " + std::to_string(line_num) +
                                       ": expected 3 tokens (e.g. 'property float x')," +
                                       " found " + std::to_string(tokens.size()));
                    return false;
                }
                constexpr auto is_list = false;
                const auto type = internal::to_type(tokens[1]);
                const auto name = tokens[2];
                constexpr auto size_type = type_unkown; // not used
                m_elements.back().properties.push_back({is_list,name,type,size_type,{/*reader*/}});
            }
        }
        else
        {
            // unkown line
            // TODO add warning
        }
    } // while

    if(not end_header_found)
    {
        m_errors.push_back("Line 'end_header' not found");
        return false;
    }

    return end_header_found;
}

bool PLYReader::read_body(const std::string& filename)
{
//    m_errors.clear();
    std::ifstream ifs(filename);
    if(ifs.is_open())
    {
        // jump header
        std::string line;
        while(std::getline(ifs, line))
        {
            if(line == "end_header")
            {
                return this->read_body(ifs);
            }
        }
//        m_errors.push_back("Line 'end_header' not found");
        return false;
    }
    else
    {
        m_errors.push_back("Failed to open file '" + filename + "'");
        return false;
    }
}

bool PLYReader::read_body(std::istream& is)
{
    if(m_ascii)
    {
        return this->read_body_ascii(is);
    }
    else if(m_binary_big_endian || m_binary_little_endian)
    {
        return this->read_body_binary(is);
    }
    else
    {
        m_errors.push_back("ascii, binary_big_endian, or binary_little_endian required");
        return false;
    }
}

// Internal reading ------------------------------------------------------------

bool PLYReader::read_body_ascii(std::istream& is)
{
    std::vector<char_t>   vec0;
    std::vector<uchar_t>  vec1;
    std::vector<short_t>  vec2;
    std::vector<ushort_t> vec3;
    std::vector<int_t>    vec4;
    std::vector<uint_t>   vec5;
    std::vector<float_t>  vec6;
    std::vector<double_t> vec7;

    char_t   val0;
    uchar_t  val1;
    short_t  val2;
    ushort_t val3;
    int_t    val4;
    uint_t   val5;
    float_t  val6;
    double_t val7;

    short_t  tmp0;
    ushort_t tmp1;

    for(auto idx_element = 0u; idx_element < m_elements.size(); ++idx_element)
    {
        const auto& element = m_elements[idx_element];
        for(auto i = 0; i < element.count; ++i)
        {
            for(auto idx_property = 0u; idx_property < element.properties.size(); ++idx_property)
            {
                const auto& property = element.properties[idx_property];
                if(property.is_list)
                {
                    size_t size = 0;
                    switch (property.size_type)
                    {
                    case type_char:    is >> tmp0; val0 = tmp0; size = val0; break;
                    case type_uchar:   is >> tmp1; val1 = tmp1; size = val1; break;
                    case type_short:   is >> val2;              size = val2; break;
                    case type_ushort:  is >> val3;              size = val3; break;
                    case type_int:     is >> val4;              size = val4; break;
                    case type_uint:    is >> val5;              size = val5; break;
                    case type_float:   is >> val6;              size = val6; break;
                    case type_double:  is >> val7;              size = val7; break;
                    default:           assert(false);
                    }

                    switch (property.type)
                    {
                    case type_char:   vec0.resize(size); for(size_t k=0; k<size; ++k) {is >> tmp0; vec0[k] = tmp0;} property.reader.read(i, vec0); break;
                    case type_uchar:  vec1.resize(size); for(size_t k=0; k<size; ++k) {is >> tmp1; vec1[k] = tmp1;} property.reader.read(i, vec1); break;
                    case type_short:  vec2.resize(size); for(size_t k=0; k<size; ++k) {is >> vec2[k];             } property.reader.read(i, vec2); break;
                    case type_ushort: vec3.resize(size); for(size_t k=0; k<size; ++k) {is >> vec3[k];             } property.reader.read(i, vec3); break;
                    case type_int:    vec4.resize(size); for(size_t k=0; k<size; ++k) {is >> vec4[k];             } property.reader.read(i, vec4); break;
                    case type_uint:   vec5.resize(size); for(size_t k=0; k<size; ++k) {is >> vec5[k];             } property.reader.read(i, vec5); break;
                    case type_float:  vec6.resize(size); for(size_t k=0; k<size; ++k) {is >> vec6[k];             } property.reader.read(i, vec6); break;
                    case type_double: vec7.resize(size); for(size_t k=0; k<size; ++k) {is >> vec7[k];             } property.reader.read(i, vec7); break;
                    default:          assert(false);
                    }
                }
                else
                {
                    switch (property.type)
                    {
                    case type_char:   is >> tmp0; val0 = tmp0; property.reader.read(i, val0); break;
                    case type_uchar:  is >> tmp1; val1 = tmp1; property.reader.read(i, val1); break;
                    case type_short:  is >> val2;              property.reader.read(i, val2); break;
                    case type_ushort: is >> val3;              property.reader.read(i, val3); break;
                    case type_int:    is >> val4;              property.reader.read(i, val4); break;
                    case type_uint:   is >> val5;              property.reader.read(i, val5); break;
                    case type_float:  is >> val6;              property.reader.read(i, val6); break;
                    case type_double: is >> val7;              property.reader.read(i, val7); break;
                    case type_unkown: assert(false);
                    }
                }
            }
        }
    }
    return true;
}

bool PLYReader::read_body_binary(std::istream& is)
{
    std::vector<char_t>   vec0;
    std::vector<uchar_t>  vec1;
    std::vector<short_t>  vec2;
    std::vector<ushort_t> vec3;
    std::vector<int_t>    vec4;
    std::vector<uint_t>   vec5;
    std::vector<float_t>  vec6;
    std::vector<double_t> vec7;

    char_t   val0;
    uchar_t  val1;
    short_t  val2;
    ushort_t val3;
    int_t    val4;
    uint_t   val5;
    float_t  val6;
    double_t val7;

//    short_t  tmp0;
//    ushort_t tmp1;

    for(auto idx_element = 0u; idx_element < m_elements.size(); ++idx_element)
    {
        const auto& element = m_elements[idx_element];
        for(auto i = 0; i < element.count; ++i)
        {
            for(auto idx_property = 0u; idx_property < element.properties.size(); ++idx_property)
            {
                const auto& property = element.properties[idx_property];
                if(property.is_list)
                {
                    size_t size = 0;
                    switch (property.size_type)
                    {
                    case type_char:       is.read(reinterpret_cast<char*>(&val0), 1); size = val0; vec0.resize(size); break;
                    case type_uchar:      is.read(reinterpret_cast<char*>(&val1), 1); size = val1; vec1.resize(size); break;
                    case type_short:      is.read(reinterpret_cast<char*>(&val2), 2); size = val2; vec2.resize(size); break;
                    case type_ushort:     is.read(reinterpret_cast<char*>(&val3), 2); size = val3; vec3.resize(size); break;
                    case type_int:        is.read(reinterpret_cast<char*>(&val4), 4); size = val4; vec4.resize(size); break;
                    case type_uint:       is.read(reinterpret_cast<char*>(&val5), 4); size = val5; vec5.resize(size); break;
                    case type_float:      is.read(reinterpret_cast<char*>(&val6), 4); size = val6; vec6.resize(size); break;
                    case type_double:     is.read(reinterpret_cast<char*>(&val7), 8); size = val7; vec7.resize(size); break;
                    default:              assert(false);
                    }

                    switch (property.type)
                    {
                    case type_char:       vec0.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec0[k]), 1);} property.reader.read(i, vec0); break;
                    case type_uchar:      vec1.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec1[k]), 1);} property.reader.read(i, vec1); break;
                    case type_short:      vec2.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec2[k]), 2);} property.reader.read(i, vec2); break;
                    case type_ushort:     vec3.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec3[k]), 2);} property.reader.read(i, vec3); break;
                    case type_int:        vec4.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec4[k]), 4);} property.reader.read(i, vec4); break;
                    case type_uint:       vec5.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec5[k]), 4);} property.reader.read(i, vec5); break;
                    case type_float:      vec6.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec6[k]), 4);} property.reader.read(i, vec6); break;
                    case type_double:     vec7.resize(size); for(size_t k=0; k<size; ++k) {is.read(reinterpret_cast<char*>(&vec7[k]), 8);} property.reader.read(i, vec7); break;
                    default:              assert(false);
                    }
                }
                else
                {
                    switch (property.type)
                    {
                    case type_char:       is.read(reinterpret_cast<char*>(&val0), 1); property.reader.read(i,val0); break;
                    case type_uchar:      is.read(reinterpret_cast<char*>(&val1), 1); property.reader.read(i,val1); break;
                    case type_short:      is.read(reinterpret_cast<char*>(&val2), 2); property.reader.read(i,val2); break;
                    case type_ushort:     is.read(reinterpret_cast<char*>(&val3), 2); property.reader.read(i,val3); break;
                    case type_int:        is.read(reinterpret_cast<char*>(&val4), 4); property.reader.read(i,val4); break;
                    case type_uint:       is.read(reinterpret_cast<char*>(&val5), 4); property.reader.read(i,val5); break;
                    case type_float:      is.read(reinterpret_cast<char*>(&val6), 4); property.reader.read(i,val6); break;
                    case type_double:     is.read(reinterpret_cast<char*>(&val7), 8); property.reader.read(i,val7); break;
                    default:              assert(false);
                    }
                }
            }
        }
    }
    return true;
}

// Reading Info Accessors ------------------------------------------------------

bool PLYReader::ascii() const
{
    return m_ascii;
}

bool PLYReader::binary() const
{
    return m_binary_big_endian || m_binary_little_endian;
}

bool PLYReader::binary_little_endian() const
{
    return m_binary_little_endian;
}

bool PLYReader::binary_big_endian() const
{
    return m_binary_big_endian;
}

int PLYReader::version() const
{
    return m_version;
}

bool PLYReader::has_element(const std::string& element_name) const
{
    auto it = std::find_if(m_elements.begin(), m_elements.end(), [&element_name](const ReadingElement& e)
    {
        return e.name == element_name;
    });
    return it != m_elements.end();
}

bool PLYReader::has_property(const std::string& element_name, const std::string& property_name) const
{
    auto it = std::find_if(m_elements.begin(), m_elements.end(), [&element_name](const ReadingElement& e)
    {
        return e.name == element_name;
    });
    if(it == m_elements.end())
    {
        return false;
    }
    const auto& e = *it;
    auto it2 = std::find_if(e.properties.begin(), e.properties.end(), [&property_name](const ReadingProperty& p)
    {
        return p.name == property_name;
    });
    return it2 != e.properties.end();
}

ReadingElement& PLYReader::element(const std::string& element_name)
{
    auto it = std::find_if(m_elements.begin(), m_elements.end(), [&element_name](const ReadingElement& e)
    {
        return e.name == element_name;
    });
    assert(it != m_elements.end()); // element does not exist
    return *it;
}

ReadingProperty& PLYReader::property(const std::string& element_name, const std::string& property_name)
{
    auto& e = element(element_name);
    auto it = std::find_if(e.properties.begin(), e.properties.end(), [&property_name](const ReadingProperty& p)
    {
        return p.name == property_name;
    });
    assert(it != e.properties.end()); // property does not exist
    return *it;
}

const std::vector<std::string>& PLYReader::comments() const
{
    return m_comments;
}

const std::vector<ReadingElement>& PLYReader::elements() const
{
    return m_elements;
}

std::vector<ReadingElement>& PLYReader::elements()
{
    return m_elements;
}

int PLYReader::element_count(const std::string& element_name) const
{
    auto it = std::find_if(m_elements.begin(), m_elements.end(),
                           [&element_name](const ReadingElement& e) -> bool
    {
        return e.name == element_name;
    });
    return it == m_elements.end() ? 0 : it->count;
}

// Writer ----------------------------------------------------------------------

Writer::Writer() :
    m_write_char_f(),
    m_write_uchar_f(),
    m_write_short_f(),
    m_write_ushort_f(),
    m_write_int_f(),
    m_write_uint_f(),
    m_write_float_f(),
    m_write_double_f(),
    m_size_char_f(),
    m_size_uchar_f(),
    m_size_short_f(),
    m_size_ushort_f(),
    m_size_int_f(),
    m_size_uint_f(),
    m_size_float_f(),
    m_size_double_f(),
    m_write_char_list_f(),
    m_write_uchar_list_f(),
    m_write_short_list_f(),
    m_write_ushort_list_f(),
    m_write_int_list_f(),
    m_write_uint_list_f(),
    m_write_float_list_f(),
    m_write_double_list_f()
{
}

char_t   Writer::write_char  (int i) const {return m_write_char_f(i);  }
uchar_t  Writer::write_uchar (int i) const {return m_write_uchar_f(i); }
short_t  Writer::write_short (int i) const {return m_write_short_f(i); }
ushort_t Writer::write_ushort(int i) const {return m_write_ushort_f(i);}
int_t    Writer::write_int   (int i) const {return m_write_int_f(i);   }
uint_t   Writer::write_uint  (int i) const {return m_write_uint_f(i);  }
float_t  Writer::write_float (int i) const {return m_write_float_f(i); }
double_t Writer::write_double(int i) const {return m_write_double_f(i);}

char_t   Writer::size_char  (int i) const {return m_size_char_f(i);  }
uchar_t  Writer::size_uchar (int i) const {return m_size_uchar_f(i); }
short_t  Writer::size_short (int i) const {return m_size_short_f(i); }
ushort_t Writer::size_ushort(int i) const {return m_size_ushort_f(i);}
int_t    Writer::size_int   (int i) const {return m_size_int_f(i);   }
uint_t   Writer::size_uint  (int i) const {return m_size_uint_f(i);  }
float_t  Writer::size_float (int i) const {return m_size_float_f(i); }
double_t Writer::size_double(int i) const {return m_size_double_f(i);}

char_t   Writer::write_char  (int i, int k) const {return m_write_char_list_f(i,k);  }
uchar_t  Writer::write_uchar (int i, int k) const {return m_write_uchar_list_f(i,k); }
short_t  Writer::write_short (int i, int k) const {return m_write_short_list_f(i,k); }
ushort_t Writer::write_ushort(int i, int k) const {return m_write_ushort_list_f(i,k);}
int_t    Writer::write_int   (int i, int k) const {return m_write_int_list_f(i,k);   }
uint_t   Writer::write_uint  (int i, int k) const {return m_write_uint_list_f(i,k);  }
float_t  Writer::write_float (int i, int k) const {return m_write_float_list_f(i,k); }
double_t Writer::write_double(int i, int k) const {return m_write_double_list_f(i,k);}

// PLYWriter -------------------------------------------------------------------

PLYWriter::PLYWriter() :
    m_ascii(false),
    m_binary_little_endian(true),
    m_binary_big_endian(false),
    m_version(1),
    m_comments(0),
    m_elements(0)
{
}

// Writing ---------------------------------------------------------------------

bool PLYWriter::write(const std::string& filename)
{
    std::ofstream ofs(filename);
    if(ofs.is_open())
    {
        return this->write(ofs);
    }
    else
    {
        m_errors.push_back("Failed to open file '" + filename + "'");
        return false;
    }
}

bool PLYWriter::write(std::ostream& os)
{
    return this->write_header(os) && this->write_body(os);
}

// Internal writing ------------------------------------------------------------

bool PLYWriter::write_header(std::ostream& os)
{
    os << "ply\n";
    os << "format ";

    if(m_ascii)
    {
        os << "ascii";
    }
    else if(m_binary_little_endian)
    {
        os << "binary_little_endian";
    }
    else if(m_binary_big_endian)
    {
        os << "binary_big_endian";
    }
    else
    {
        assert(false); // neither ascii not binary
    }

    os << " " << m_version << ".0\n";   //TODO make this format clear: 1.0?

    for(const auto& comment : m_comments)
    {
        os << "comment " << comment << "\n";
    }

    for(const auto& element : m_elements)
    {
        os << "element " << element.name << " " << element.count << "\n";
        for(const auto& property : element.properties)
        {
            if(property.is_list)
            {
                os << "property list " << internal::to_string(property.size_type) << " " << internal::to_string(property.type) << " " << property.name << "\n";
            }
            else
            {
                os << "property " << internal::to_string(property.type) << " " << property.name << "\n";
            }
        }
    }

    os << "end_header\n";
    return true;
}

bool PLYWriter::write_body(std::ostream& os)
{
    if(m_ascii)
    {
        return this->write_body_ascii(os);
    }
    else if(m_binary_big_endian || m_binary_little_endian)
    {
        return this->write_body_binary(os);
    }
    else
    {
        assert(false); // neither ascii not binary
        return false;
    }
}

bool PLYWriter::write_body_ascii(std::ostream& os)
{
    char_t   size0;
    uchar_t  size1;
    short_t  size2;
    ushort_t size3;
    int_t    size4;
    uint_t   size5;
    float_t  size6;
    double_t size7;

    for(auto idx_element = 0u; idx_element < m_elements.size(); ++idx_element)
    {
        const auto& element = m_elements[idx_element];
        for(auto i = 0; i < element.count; ++i)
        {
            for(auto idx_property = 0u; idx_property < element.properties.size(); ++idx_property)
            {
                const auto& property = element.properties[idx_property];
                if(property.is_list)
                {
                    size_t size = 0;
                    switch (property.size_type)
                    {
                    case type_char:    size0 = property.writer.size_char(i);   size = size0; os <<  short_t(size0); break;
                    case type_uchar:   size1 = property.writer.size_uchar(i);  size = size1; os << ushort_t(size1); break;
                    case type_short:   size2 = property.writer.size_short(i);  size = size2; os << size2; break;
                    case type_ushort:  size3 = property.writer.size_ushort(i); size = size3; os << size3; break;
                    case type_int:     size4 = property.writer.size_int(i);    size = size4; os << size4; break;
                    case type_uint:    size5 = property.writer.size_uint(i);   size = size5; os << size5; break;
                    case type_float:   size6 = property.writer.size_float(i);  size = size6; os << size6; break;
                    case type_double:  size7 = property.writer.size_double(i); size = size7; os << size7; break;
                    default:           assert(false);
                    }
                    os << " ";

                    switch (property.type)
                    {
                    case type_char:   for(size_t k=0; k<size; ++k) {os <<  short_t(property.writer.write_char  (i,k)) << " ";} break;
                    case type_uchar:  for(size_t k=0; k<size; ++k) {os << ushort_t(property.writer.write_uchar (i,k)) << " ";} break;
                    case type_short:  for(size_t k=0; k<size; ++k) {os << property.writer.write_short (i,k) << " ";} break;
                    case type_ushort: for(size_t k=0; k<size; ++k) {os << property.writer.write_ushort(i,k) << " ";} break;
                    case type_int:    for(size_t k=0; k<size; ++k) {os << property.writer.write_int   (i,k) << " ";} break;
                    case type_uint:   for(size_t k=0; k<size; ++k) {os << property.writer.write_uint  (i,k) << " ";} break;
                    case type_float:  for(size_t k=0; k<size; ++k) {os << property.writer.write_float (i,k) << " ";} break;
                    case type_double: for(size_t k=0; k<size; ++k) {os << property.writer.write_double(i,k) << " ";} break;
                    default:          assert(false);
                    }
                }
                else
                {
                    switch (property.type)
                    {
                    case type_char:   os <<  short_t(property.writer.write_char  (i)); break;
                    case type_uchar:  os << ushort_t(property.writer.write_uchar (i)); break;
                    case type_short:  os << property.writer.write_short (i); break;
                    case type_ushort: os << property.writer.write_ushort(i); break;
                    case type_int:    os << property.writer.write_int   (i); break;
                    case type_uint:   os << property.writer.write_uint  (i); break;
                    case type_float:  os << property.writer.write_float (i); break;
                    case type_double: os << property.writer.write_double(i); break;
                    default:          assert(false);
                    }
                }
                os << " ";
            }
            os << "\n";
        }
    }
    return true;
}

bool PLYWriter::write_body_binary(std::ostream& os)
{
    char_t   val0;
    uchar_t  val1;
    short_t  val2;
    ushort_t val3;
    int_t    val4;
    uint_t   val5;
    float_t  val6;
    double_t val7;

    for(auto idx_element = 0u; idx_element < m_elements.size(); ++idx_element)
    {
        const auto& element = m_elements[idx_element];
        for(auto i = 0; i < element.count; ++i)
        {
            for(auto idx_property = 0u; idx_property < element.properties.size(); ++idx_property)
            {
                const auto& property = element.properties[idx_property];
                if(property.is_list)
                {
                    size_t size = 0;
                    switch (property.size_type)
                    {
                    case type_char:    val0 = property.writer.size_char(i);   size = val0; os.write(reinterpret_cast<const char*>(&val0), 1); break;
                    case type_uchar:   val1 = property.writer.size_uchar(i);  size = val1; os.write(reinterpret_cast<const char*>(&val1), 1); break;
                    case type_short:   val2 = property.writer.size_short(i);  size = val2; os.write(reinterpret_cast<const char*>(&val2), 2); break;
                    case type_ushort:  val3 = property.writer.size_ushort(i); size = val3; os.write(reinterpret_cast<const char*>(&val3), 2); break;
                    case type_int:     val4 = property.writer.size_int(i);    size = val4; os.write(reinterpret_cast<const char*>(&val4), 4); break;
                    case type_uint:    val5 = property.writer.size_uint(i);   size = val5; os.write(reinterpret_cast<const char*>(&val5), 4); break;
                    case type_float:   val6 = property.writer.size_float(i);  size = val6; os.write(reinterpret_cast<const char*>(&val6), 4); break;
                    case type_double:  val7 = property.writer.size_double(i); size = val7; os.write(reinterpret_cast<const char*>(&val7), 8); break;
                    default:           assert(false);
                    }

                    switch (property.type)
                    {
                    case type_char:   for(size_t k=0; k<size; ++k) {val0 = property.writer.write_char  (i,k); os.write(reinterpret_cast<const char*>(&val0), 1);} break;
                    case type_uchar:  for(size_t k=0; k<size; ++k) {val1 = property.writer.write_uchar (i,k); os.write(reinterpret_cast<const char*>(&val1), 1);} break;
                    case type_short:  for(size_t k=0; k<size; ++k) {val2 = property.writer.write_short (i,k); os.write(reinterpret_cast<const char*>(&val2), 2);} break;
                    case type_ushort: for(size_t k=0; k<size; ++k) {val3 = property.writer.write_ushort(i,k); os.write(reinterpret_cast<const char*>(&val3), 2);} break;
                    case type_int:    for(size_t k=0; k<size; ++k) {val4 = property.writer.write_int   (i,k); os.write(reinterpret_cast<const char*>(&val4), 4);} break;
                    case type_uint:   for(size_t k=0; k<size; ++k) {val5 = property.writer.write_uint  (i,k); os.write(reinterpret_cast<const char*>(&val5), 4);} break;
                    case type_float:  for(size_t k=0; k<size; ++k) {val6 = property.writer.write_float (i,k); os.write(reinterpret_cast<const char*>(&val6), 4);} break;
                    case type_double: for(size_t k=0; k<size; ++k) {val7 = property.writer.write_double(i,k); os.write(reinterpret_cast<const char*>(&val7), 8);} break;
                    default:          assert(false);
                    }
                }
                else
                {
                    switch (property.type)
                    {
                    case type_char:   val0 = property.writer.write_char  (i); os.write(reinterpret_cast<const char*>(&val0), 1); break;
                    case type_uchar:  val1 = property.writer.write_uchar (i); os.write(reinterpret_cast<const char*>(&val1), 1); break;
                    case type_short:  val2 = property.writer.write_short (i); os.write(reinterpret_cast<const char*>(&val2), 2); break;
                    case type_ushort: val3 = property.writer.write_ushort(i); os.write(reinterpret_cast<const char*>(&val3), 2); break;
                    case type_int:    val4 = property.writer.write_int   (i); os.write(reinterpret_cast<const char*>(&val4), 4); break;
                    case type_uint:   val5 = property.writer.write_uint  (i); os.write(reinterpret_cast<const char*>(&val5), 4); break;
                    case type_float:  val6 = property.writer.write_float (i); os.write(reinterpret_cast<const char*>(&val6), 4); break;
                    case type_double: val7 = property.writer.write_double(i); os.write(reinterpret_cast<const char*>(&val7), 8); break;
                    default:          assert(false);
                    }
                }
            }
        }
    }
    return true;
}

// Writing Info Modifiers ------------------------------------------------------

void PLYWriter::set_ascii()
{
    m_ascii                 = true;
    m_binary_little_endian  = false;
    m_binary_big_endian     = false;
}

void PLYWriter::set_binary()
{
    m_ascii                 = false;
    m_binary_little_endian  = true;
    m_binary_big_endian     = false;
}

void PLYWriter::set_binary_little_endian()
{
    m_ascii                 = false;
    m_binary_little_endian  = true;
    m_binary_big_endian     = false;
}

void PLYWriter::set_binary_big_endian()
{
    m_ascii                 = false;
    m_binary_little_endian  = false;
    m_binary_big_endian     = true;
}

void PLYWriter::set_version(int version)
{
    m_version = version;
}

void PLYWriter::add_comment(const std::string& comment)
{
    m_comments.emplace_back(comment);
}

WritingElement& PLYWriter::add_element(const std::string& element_name, int element_count)
{
    m_elements.push_back({element_name, element_count, {/*properties*/}});
    return m_elements.back();
}

template<typename GetPropFuncT>
void PLYWriter::add_property(const std::string& element_name, const std::string& property_name, GetPropFuncT&& get_prop_f)
{
    using PropertyType = typename std::result_of<GetPropFuncT(int)>::type;

    const auto it = std::find_if(m_elements.begin(), m_elements.end(), [&element_name](const auto& e)
    {
        return e.name == element_name;
    });

    constexpr auto is_list = false;
    constexpr auto size_type = type_unkown; // not used
    const auto property_type = internal::get_type<PropertyType>();
    const WritingFunction<PropertyType> get_prop = get_prop_f;
    it->properties.push_back({property_name,property_type,is_list,size_type,get_prop});
}

template<typename GetSizeFuncT, typename GetPropFuncT>
void PLYWriter::add_property(const std::string& element_name, const std::string& property_name, GetSizeFuncT&& get_size_f, GetPropFuncT&& get_prop_f)
{
    using PropertyType = typename std::result_of<GetPropFuncT(int,int)>::type;
    using SizeType = typename std::result_of<GetSizeFuncT(int)>::type;

    const auto it = std::find_if(m_elements.begin(), m_elements.end(), [&element_name](const auto& e)
    {
        return e.name == element_name;
    });


    constexpr auto is_list = true;
    const auto size_type = internal::get_type<SizeType>();
    const auto property_type = internal::get_type<PropertyType>();
    const SizeFunction<SizeType> get_size = get_size_f;
    const WritingFunctionList<PropertyType> get_prop = get_prop_f;
    it->properties.push_back({property_name,property_type,is_list,size_type,{get_size,get_prop}});
}

namespace internal {

// Types -----------------------------------------------------------------------

Type to_type(const std::string& str)
{
    if(str == "char"  ) return type_char;
    if(str == "uchar" ) return type_uchar;
    if(str == "short" ) return type_short;
    if(str == "ushort") return type_ushort;
    if(str == "int"   ) return type_int;
    if(str == "uint"  ) return type_uint;
    if(str == "float" ) return type_float;
    if(str == "double") return type_double;
    else                return type_unkown;
}

std::string to_string(Type type)
{
    switch(type)
    {
    case type_char:   return "char";
    case type_uchar:  return "uchar";
    case type_short:  return "short";
    case type_ushort: return "ushort";
    case type_int:    return "int";
    case type_uint:   return "uint";
    case type_float:  return "float";
    case type_double: return "double";
    default:          return "unkown";
    }
}

template<> Type get_type<char_t  >() {return type_char  ;}
template<> Type get_type<uchar_t >() {return type_uchar ;}
template<> Type get_type<short_t >() {return type_short ;}
template<> Type get_type<ushort_t>() {return type_ushort;}
template<> Type get_type<int_t   >() {return type_int   ;}
template<> Type get_type<uint_t  >() {return type_uint  ;}
template<> Type get_type<float_t >() {return type_float ;}
template<> Type get_type<double_t>() {return type_double;}

template<typename SourceType, typename TargetType>
void cast(SourceType from, TargetType& to)
{
    to = TargetType(from);
}

template<typename SourceType, typename TargetType>
void cast_list(const std::vector<SourceType>& from, std::vector<TargetType>& to)
{
    to.resize(from.size());
    for(auto i = 0u; i < from.size(); ++i)
    {
        cast(from[i], to[i]);
    }
}

// String ----------------------------------------------------------------------

std::vector<std::string> to_tokens(const std::string& str)
{
    std::stringstream ss(str);
    std::string token;
    std::vector<std::string> tokens;
    while(std::getline(ss, token, ' '))
    {
        if(!token.empty())
        {
            tokens.emplace_back(std::move(token));
        }
    }
    return tokens;
}

} // namespace internal

} // namespace plyio
