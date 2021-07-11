#pragma once

#include <cmath>
#include <vector>
#include <functional>
#include <numeric>
#include <sstream>
#include <memory>

#ifndef KDTREE_ASSERT
    #include <assert.h>
    #define KDTREE_ASSERT(expr) assert(expr)
#endif

namespace kdtree {

template<typename PointT,
         int static_dim,
         typename ScalarT = float,
         typename IndexT = int,
         int static_max_depth = 32>
class KdTree;

enum class Source {index, point};
enum class Search {k_nearest, nearest, range};

namespace container {

template<class T, class CompareT = std::less<T>>
class limited_priority_queue
{
public:
    using value_type      = T;
    using container_type  = std::vector<T>;
    using compare         = CompareT;
    using iterator        = typename container_type::iterator;
    using const_iterator  = typename container_type::const_iterator;
    using this_type       = limited_priority_queue<T,CompareT>;

    // limited_priority_queue --------------------------------------------------
public:
    inline limited_priority_queue();
    inline limited_priority_queue(const this_type& other);
    inline explicit limited_priority_queue(int capacity);
    template<class InputIt>
    inline limited_priority_queue(int capacity, InputIt first, InputIt last);

    inline ~limited_priority_queue();

    inline limited_priority_queue& operator=(const this_type& other);

    // Iterator ----------------------------------------------------------------
public:
    inline iterator begin();
    inline const_iterator begin() const;
    inline const_iterator cbegin() const;
    inline iterator end();
    inline const_iterator end() const;
    inline const_iterator cend() const;

    // Element access ----------------------------------------------------------
public:
    inline const T& top() const;
    inline const T& bottom() const;
    inline T& top();
    inline T& bottom();

    // Capacity ----------------------------------------------------------------
public:
    inline bool empty() const;
    inline bool full() const;
    inline int size() const;
    inline int capacity() const;

    // Modifiers ---------------------------------------------------------------
public:
    inline bool push(const T& value);
    inline bool push(T&& value);
    inline void pop();
    inline void reserve(int capacity);
    inline void clear();

    // Data --------------------------------------------------------------------
public:
    inline const container_type& container() const;

protected:
    container_type  m_c;
    compare         m_comp;
    int             m_size;
};

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

} // namespace container

template<class IndexT, class ScalarT>
struct IndexSquaredDistance
{
    using Index = IndexT;
    using Scalar = ScalarT;

    Index index;
    Scalar squared_distance;

    inline bool operator < (const IndexSquaredDistance& other) const
    {
        return squared_distance < other.squared_distance;
    }
}; // struct IndexSquaredDistance

namespace query {

// Sources ---------------------------------------------------------------------

template<Source, class IndexT, class PointT>
class QuerySource;

template<class IndexT, class PointT>
class QuerySource<Source::index, IndexT, PointT>
{
public:
  using Index = IndexT;
public:
  inline Index index() const {return m_index;}
  inline void set_index(Index index) {m_index = index;}
protected:
  Index m_index;
};

template<class IndexT, class PointT>
class QuerySource<Source::point, IndexT, PointT>
{
public:
  using Point = PointT;
public:
  inline const Point& point() const {return m_point;}
  inline void set_point(const Point& point) {m_point = point;}
protected:
  Point m_point;
};

// Iterators -------------------------------------------------------------------

template<Search, class IndexT, class QueryT> //TODO get Types from QueryT typename only
class Iterator;

template<class IndexT, class QueryT>
class Iterator<Search::k_nearest, IndexT, QueryT>
{
public:
  using Index = IndexT;
  using Scalar = typename QueryT::Scalar;
  using InternalIterator = typename container::limited_priority_queue<IndexSquaredDistance<Index,Scalar>>::iterator;
public:
  inline Iterator() = default;
  inline Iterator(InternalIterator iterator) : m_iterator(iterator){}
  inline bool operator != (const Iterator& other) const {return m_iterator != other.m_iterator;}
  inline void operator ++ () {++m_iterator;}
  inline Index operator * () const {return m_iterator->index;}
  inline void operator += (Index i) {m_iterator += i;}
protected:
  InternalIterator m_iterator;
};

template<class IndexT, class QueryT>
class Iterator<Search::nearest, IndexT, QueryT>
{
public:
  using Index = IndexT;
public:
  inline Iterator() = default;
  inline Iterator(Index index) : m_index(index) {}
  inline bool operator != (const Iterator& other) const {return m_index != other.m_index;}
  inline void operator ++ () {++m_index;}
  inline Index operator * () const {return m_index;}
protected:
  Index m_index;
};

template<class IndexT, class QueryT>
class Iterator<Search::range, IndexT, QueryT>
{
public:
  using Index = IndexT;
  using Query = QueryT;
public:
  inline Iterator() = default; // TODO set index to -1 ?????????????????????????
  inline Iterator(Query* query) : Iterator(query, 0) {}
  inline Iterator(Query* query, Index index) :
    m_query(query), m_index(index), m_start(0), m_end(0) {}
  inline bool operator != (const Iterator& other) const {return m_index != other.m_index;}
  inline void operator ++ () {m_query->advance(*this);}
  inline Index operator * () const {return m_index;}
protected:
  Query* m_query;
public: // TODO protect this
  Index m_index;
  Index m_start;
  Index m_end;
};

// Queries ---------------------------------------------------------------------

template<Search, class IndexT, class ScalarT>
class NeighborQuery;

template<class IndexT, class ScalarT>
class NeighborQuery<Search::k_nearest, IndexT, ScalarT>
{
public:
  using Index = IndexT;
  using Scalar = ScalarT;
  using Queue = container::limited_priority_queue<IndexSquaredDistance<Index,Scalar>>;
public:
  inline Index k() const {return m_queue.capacity();}
  inline void set_k(Index k) {m_queue.reserve(k);}
  inline Queue& queue() {return m_queue;}
public: //TODO protected
  inline Scalar squared_distance_ref() const {return m_queue.bottom().squared_distance;}
protected:
  Queue m_queue;
};

template<class IndexT, class ScalarT>
class NeighborQuery<Search::nearest, IndexT, ScalarT>
{
public:
  using Index = IndexT;
  using Scalar = ScalarT;
public:
  inline Index nearest_index() const {return m_nearest_index;}
  inline Scalar squared_distance() const {return m_squared_distance;}
public: //TODO protected
  inline Scalar squared_distance_ref() const {return m_squared_distance;}
protected:
    Index m_nearest_index;
    Scalar m_squared_distance;
};

template<class IndexT, class ScalarT>
class NeighborQuery<Search::range, IndexT, ScalarT>
{
public:
  using Index = IndexT;
  using Scalar = ScalarT;
public:
  inline Scalar radius() const {return std::sqrt(m_squared_radius);}
  inline Scalar squared_radius() const {return m_squared_radius;}
  inline void set_radius(Scalar radius) {m_squared_radius = radius * radius;}
public: //TODO protected
  inline Scalar squared_distance_ref() const {return m_squared_radius;}
protected:
  Scalar m_squared_radius;
};

// -----------------------------------------------------------------------------

template<class KdTreeT, Search search_type, Source source_type>
class Query : public NeighborQuery<search_type, typename KdTreeT::Index, typename KdTreeT::Scalar>,
              public QuerySource<source_type, typename KdTreeT::Index, typename KdTreeT::Point>
{
public:
  using BaseQuery = NeighborQuery<search_type, typename KdTreeT::Index, typename KdTreeT::Scalar>;
  using BaseSource = QuerySource<source_type, typename KdTreeT::Index, typename KdTreeT::Point>;
  using KdTree = KdTreeT;
  using Index = typename KdTree::Index;
  using Scalar = typename KdTree::Scalar;
  using Point = typename KdTree::Point;
  using Stack = container::static_stack<IndexSquaredDistance<Index,Scalar>, 2 * KdTreeT::max_depth()>;
  using IteratorType = Iterator<search_type, Index, Query>;
public:
  inline Query(const KdTree* kdtree = nullptr) : m_kdtree(kdtree), m_stack() {}
  inline IteratorType begin()
  {
    if constexpr(search_type == Search::k_nearest) {
      search();
      return IteratorType(BaseQuery::m_queue.begin());
    } else if constexpr(search_type == Search::nearest) {
      search();
      return IteratorType(BaseQuery::m_nearest_index);
    } else if constexpr(search_type == Search::range) {
      IteratorType it(this);
      initialize(it);
      advance(it);
      return it;
    }
  }
  inline IteratorType end()
  {
    if constexpr(search_type == Search::k_nearest) {
      return IteratorType(BaseQuery::m_queue.end());
    } else if constexpr(search_type == Search::nearest) {
      return IteratorType(BaseQuery::m_nearest_index+1);
    } else if constexpr(search_type == Search::range) {
      return IteratorType(this, m_kdtree->point_count());
    }
  }
  inline const Query& search()
  {
    struct DummyIterator {};
    if constexpr(search_type == Search::k_nearest) {
      DummyIterator dummy;
      do_search(dummy);
    } else if(search_type == Search::nearest) {
      DummyIterator dummy;
      do_search(dummy);
    } else if(search_type == Search::range) {
//      nothing to do
    }
    return *this;
  }
public: //TODO protect this
  inline void initialize(IteratorType& it)
  {
    //TODO range only, how to check ?
    //TODO put this in do_search, width -1 as uninitialised index
    m_stack.clear();
    m_stack.push();
    m_stack.top().index = 0;
    m_stack.top().squared_distance = 0;
    it.m_index = -1;
    it.m_start = 0;
    it.m_end   = 0;
  }
  inline void advance(IteratorType& it)
  {
    //TODO range only, how to check ?
    do_search(it);
  }
protected:
  //! \brief get the query source point depending on the source type
  inline const Point& point() const
  {
    if constexpr(source_type == Source::index) {
      return m_kdtree->point_data()[BaseSource::m_index];
    } else if constexpr(source_type == Source::point) {
      return BaseSource::m_point;
    } else {
      // TODO static error
    }
  }
  //! \brief says if idx must be skipped or can be included in a search result
  inline bool must_skip(Index idx) const
  {
    if constexpr(source_type == Source::index) {
      return idx == BaseSource::m_index;
    } else if constexpr(source_type == Source::point) {
      return false;
    } else {
      //TODO static error
    }
  }

  template<class GenericIteratorT>
  inline void do_search(GenericIteratorT& it)
  {
    const auto& nodes   = m_kdtree->node_data();
    const auto& points  = m_kdtree->point_data();
    const auto& indices = m_kdtree->index_data();

    // initialization
    if constexpr(search_type == Search::k_nearest) {
      m_stack.clear();
      m_stack.push({0,0});
      BaseQuery::m_queue.clear();
      BaseQuery::m_queue.push({-1,std::numeric_limits<Scalar>::max()}); //TODO WARNING -1 cannot be uint ?
    } else if constexpr(search_type == Search::nearest) {
      m_stack.clear();
      m_stack.push({0,0});
      if constexpr(source_type == Source::index) {
        const auto& point = points[BaseSource::m_index];
        BaseQuery::m_nearest_index = BaseSource::m_index == indices[0] ? indices[1] : indices[0]; // TODO check number of points ?
        BaseQuery::m_squared_distance = (point - points[BaseQuery::m_nearest_index]).squaredNorm();
      } else if constexpr(source_type == Source::point) {
        BaseQuery::m_nearest_index = indices[0];
        BaseQuery::m_squared_distance = (BaseSource::m_point - points[BaseQuery::m_nearest_index]).squaredNorm();
      } else {
        //TODO static error
      }
    } else if constexpr(search_type == Search::range) {
      for(auto i = it.m_start; i < it.m_end; ++i)
      {
        const auto idx = indices[i];
        if(must_skip(idx)) continue;

        if((point() - points[idx]).squaredNorm() < BaseQuery::m_squared_radius)
        {
          it.m_index = idx;
          it.m_start = i+1;
          return;
        }
      }
    } else {
      //TODO static error
    }

    while(!m_stack.empty())
    {
      auto& qnode = m_stack.top();
      const auto& node  = nodes[qnode.index];

      if(qnode.squared_distance < BaseQuery::squared_distance_ref())
      {
        if(node.leaf)
        {
          m_stack.pop();
          const auto start = node.start;
          const auto end = start + node.size;
          for(auto i = start; i < end; ++i)
          {
            const auto idx = indices[i];
            if(must_skip(idx)) continue;

            const auto d2 = (point() - points[idx]).squaredNorm();

            if constexpr(search_type == Search::k_nearest) {
              BaseQuery::m_queue.push({idx, d2});
            } else if constexpr(search_type == Search::nearest) {
              if(d2 < BaseQuery::m_squared_distance)
              {
                BaseQuery::m_nearest_index = idx;
                BaseQuery::m_squared_distance = d2;
              }
            } else if constexpr(search_type == Search::range) {
              if(d2 < BaseQuery::m_squared_radius)
              {
                it.m_end = end;
                it.m_index = idx;
                it.m_start = i+1;
                return;
              }
            } else {
              // TODO static error
            }
          }
        }
        else
        {
          // replace the stack top by the farthest and push the closest
          const auto new_offset = point()[node.dim] - node.split_value;
          m_stack.push();
          if(new_offset < 0)
          {
            m_stack.top().index = node.first_child_id;
            qnode.index         = node.first_child_id+1;
          }
          else
          {
            m_stack.top().index = node.first_child_id+1;
            qnode.index         = node.first_child_id;
          }
          m_stack.top().squared_distance = qnode.squared_distance;
          qnode.squared_distance         = new_offset * new_offset;
        }
      }
      else
      {
        m_stack.pop();
      }
    } // while(!m_stack.empty())
    if constexpr(search_type == Search::range) {
      // when the range search is complete
      // range iterator must point to the end point
      it.m_index = points.size();
    }
  }
protected:
  const KdTree* m_kdtree;
  Stack m_stack;
};

} // query

namespace container {

// limited_priority_queue ------------------------------------------------------

template<class T, class Cmp>
limited_priority_queue<T,Cmp>::limited_priority_queue() :
    m_c(),
    m_comp(),
    m_size(0)
{
}


template<class T, class Cmp>
limited_priority_queue<T,Cmp>::limited_priority_queue(const this_type& other) :
    m_c(other.m_c),
    m_comp(other.m_comp),
    m_size(other.m_size)
{
}

template<class T, class Cmp>
limited_priority_queue<T,Cmp>::limited_priority_queue(int capacity) :
    m_c(capacity),
    m_comp(),
    m_size(0)
{
}

template<class T, class Cmp>
template<class InputIt>
limited_priority_queue<T,Cmp>::limited_priority_queue(int capacity, InputIt first, InputIt last) :
    m_c(capacity),
    m_comp(),
    m_size(0)
{
    for(InputIt it=first; it<last; ++it)
    {
        push(*it);
    }
}

template<class T, class Cmp>
limited_priority_queue<T,Cmp>::~limited_priority_queue()
{
}

template<class T, class Cmp>
limited_priority_queue<T,Cmp>& limited_priority_queue<T,Cmp>::operator=(const this_type& other)
{
    m_c    = other.m_c;
    m_comp = other.m_comp;
    m_size = other.m_size;
    return *this;
}

// Iterator --------------------------------------------------------------------

template<class T, class Cmp>
typename limited_priority_queue<T,Cmp>::iterator limited_priority_queue<T,Cmp>::begin()
{
    return m_c.begin();
}

template<class T, class Cmp>
typename limited_priority_queue<T,Cmp>::const_iterator limited_priority_queue<T,Cmp>::begin() const
{
    return m_c.begin();
}

template<class T, class Cmp>
typename limited_priority_queue<T,Cmp>::const_iterator limited_priority_queue<T,Cmp>::cbegin() const
{
    return m_c.cbegin();
}

template<class T, class Cmp>
typename limited_priority_queue<T,Cmp>::iterator limited_priority_queue<T,Cmp>::end()
{
    return m_c.begin() + m_size;
}

template<class T, class Cmp>
typename limited_priority_queue<T,Cmp>::const_iterator limited_priority_queue<T,Cmp>::end() const
{
    return m_c.begin() + m_size;
}

template<class T, class Cmp>
typename limited_priority_queue<T,Cmp>::const_iterator limited_priority_queue<T,Cmp>::cend() const
{
    return m_c.cbegin() + m_size;
}

// Element access --------------------------------------------------------------

template<class T, class Cmp>
const T& limited_priority_queue<T,Cmp>::top() const
{
    return m_c[0];
}

template<class T, class Cmp>
const T& limited_priority_queue<T,Cmp>::bottom() const
{
    return m_c[m_size-1];
}

template<class T, class Cmp>
T& limited_priority_queue<T,Cmp>::top()
{
    return m_c[0];
}

template<class T, class Cmp>
T& limited_priority_queue<T,Cmp>::bottom()
{
    return m_c[m_size-1];
}

// Capacity --------------------------------------------------------------------

template<class T, class Cmp>
bool limited_priority_queue<T,Cmp>::empty() const
{
    return m_size == 0;
}

template<class T, class Cmp>
bool limited_priority_queue<T,Cmp>::full() const
{
    return m_size == capacity();
}

template<class T, class Cmp>
int limited_priority_queue<T,Cmp>::size() const
{
    return m_size;
}

template<class T, class Cmp>
int limited_priority_queue<T,Cmp>::capacity() const
{
    return m_c.size();
}

// Modifiers -------------------------------------------------------------------

template<class T, class Cmp>
bool limited_priority_queue<T,Cmp>::push(const T& value)
{
    if(empty())
    {
        if(capacity()>0)
        {
            m_c.front() = value;
            ++m_size;
            return true;
        }
    }
    else
    {
        iterator it = std::upper_bound(begin(), end(), value, m_comp);
        if(it==end())
        {
            if(!full())
            {
                *it = value;
                ++m_size;
                return true;
            }
        }
        else
        {
            if(full())
            {
                std::copy_backward(it, end()-1, end());
                *it = value;
            }
            else
            {
                std::copy_backward(it, end(), end()+1);
                *it = value;
                ++m_size;
            }
            return true;
        }
    }
    return false;
}

template<class T, class Cmp>
bool limited_priority_queue<T,Cmp>::push(T&& value)
{
    if(empty())
    {
        if(capacity()>0)
        {
            m_c.front() = std::move(value);
            ++m_size;
            return true;
        }
    }
    else
    {
        iterator it = std::upper_bound(begin(), end(), std::move(value), m_comp);
        if(it==end())
        {
            if(!full())
            {
                *it = std::move(value);
                ++m_size;
                return true;
            }
        }
        else
        {
            if(full())
            {
                std::copy_backward(it, end()-1, end());
                *it = std::move(value);
            }
            else
            {
                std::copy_backward(it, end(), end()+1);
                *it = std::move(value);
                ++m_size;
            }
            return true;
        }
    }
    return false;
}

template<class T, class Cmp>
void limited_priority_queue<T,Cmp>::pop()
{
    --m_size;
}

template<class T, class Cmp>
void limited_priority_queue<T,Cmp>::reserve(int capacity)
{
    if(m_size>capacity)
    {
        m_size = capacity;
    }
    m_c.resize(capacity);
}

template<class T, class Cmp>
void limited_priority_queue<T,Cmp>::clear()
{
    m_size = 0;
}

// Data ------------------------------------------------------------------------

template<class T, class Cmp>
const typename limited_priority_queue<T,Cmp>::container_type& limited_priority_queue<T,Cmp>::container() const
{
    return m_c;
}

// static_stack ----------------------------------------------------------------

template<class T, int N>
static_stack<T,N>::static_stack() :
    m_size(0),
    m_data()
{
}

template<class T, int N>
const T& static_stack<T,N>::top() const
{
    KDTREE_ASSERT(!empty());
    return m_data[m_size-1];
}

template<class T, int N>
T& static_stack<T,N>::top()
{
    KDTREE_ASSERT(!empty());
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
    KDTREE_ASSERT(!full());
    m_data[m_size] = value;
    ++m_size;
}

template<class T, int N>
void static_stack<T,N>::push()
{
    KDTREE_ASSERT(!full());
    m_data[m_size] = T();
    ++m_size;
}

template<class T, int N>
void static_stack<T,N>::pop()
{
    KDTREE_ASSERT(!empty());
    --m_size;
}

template<class T, int N>
void static_stack<T,N>::clear()
{
    m_size = 0;
}

} // namespace container

namespace node {
struct Node
{
    union {
        struct {
            float          split_value;
            unsigned int   first_child_id:24;
            unsigned int   dim:2;
            unsigned int   leaf:1;
        };
        struct {
            unsigned int   start;
            unsigned short size;
        };
    };
}; // struct Node
} // namespace node

template<typename PointT,
         int static_dim,
         typename ScalarT,
         typename IndexT,
         int static_max_depth>
class KdTree
{
    // Types -------------------------------------------------------------------
public:
    using KNearestPointQuery = query::Query<KdTree, Search::k_nearest, Source::point>;
    using KNearestIndexQuery = query::Query<KdTree, Search::k_nearest, Source::index>;
    using NearestIndexQuery  = query::Query<KdTree, Search::nearest, Source::index>;
    using NearestPointQuery  = query::Query<KdTree, Search::nearest, Source::point>;
    using RangePointQuery    = query::Query<KdTree, Search::range, Source::point>;
    using RangeIndexQuery    = query::Query<KdTree, Search::range, Source::index>;

    using Point = PointT;
    using Scalar = ScalarT;
    using Index = IndexT;
    using Node = node::Node;

    inline static constexpr int max_depth() {return static_max_depth;}
    inline static constexpr int dim() {return static_dim;}

    static_assert(std::is_same<Index,int>::value, "only 'int' is supported as index type");

    // KdTree ------------------------------------------------------------------
public:
    inline KdTree();
    inline KdTree(std::shared_ptr<std::vector<Point>>& points);
    inline KdTree(std::shared_ptr<std::vector<Point>>& points, const std::vector<int>& sampling);

    inline void clear();
    inline void build(std::shared_ptr<std::vector<Point>>& points);
    inline void build(std::shared_ptr<std::vector<Point>>& points, const std::vector<int>& sampling);
    inline void rebuild(const std::vector<int>& sampling);

    inline bool valid() const; //TODO keep ?
    inline std::string to_string() const; //TODO keep ?

    // Query -------------------------------------------------------------------
public:
    inline KNearestPointQuery k_nearest_neighbors(const Point& point, int k) const;
    inline KNearestIndexQuery k_nearest_neighbors(int index, int k) const;
    inline Index nearest_neighbor(const Point& point) const;
    inline Index nearest_neighbor(int index) const;
    inline RangePointQuery    range_neighbors(const Point& point, Scalar r) const;
    inline RangeIndexQuery    range_neighbors(int index, Scalar r) const;

    // Empty Query -------------------------------------------------------------
public:
    inline KNearestPointQuery k_nearest_point_query(int k = 0) const;
    inline KNearestIndexQuery k_nearest_index_query(int k = 0) const;
    inline NearestPointQuery  nearest_point_query() const;
    inline NearestIndexQuery  nearest_index_query() const;
    inline RangePointQuery    range_point_query(Scalar r = 0) const;
    inline RangeIndexQuery    range_index_query(Scalar r = 0) const;

    // Accessors ---------------------------------------------------------------
public:
    inline int node_count() const;
    inline int index_count() const;
    inline int point_count() const;

    inline const std::vector<Point>& point_data() const;
    inline       std::vector<Point>& point_data();

    inline const std::shared_ptr<std::vector<Point>>& point_ptr() const;
    inline       std::shared_ptr<std::vector<Point>>& point_ptr();

    inline const std::vector<Node>& node_data() const;
    inline       std::vector<Node>& node_data();

    inline const std::vector<int>& index_data() const;
    inline       std::vector<int>& index_data();

    // Parameters --------------------------------------------------------------
public:
    inline int min_cell_size() const;
    inline void set_min_cell_size(int min_cell_size);

    // Internal ----------------------------------------------------------------
public:
    inline void build_rec(int node_id, int start, int end, int level);
    inline int partition(int start, int end, int dim, Scalar value);

    // Data --------------------------------------------------------------------
protected:
    std::shared_ptr<std::vector<Point>> m_points;
    std::shared_ptr<std::vector<Node>>  m_nodes;
    std::shared_ptr<std::vector<int>>   m_indices;

    int m_min_cell_size;

}; // class KdTree

} // namespace kdtree

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace kdtree {

// KdTree ----------------------------------------------------------------------

template<typename P, int d, typename S, typename I, int md>
KdTree<P,d,S,I,md>::KdTree() :
    m_points(nullptr),
    m_nodes(nullptr),
    m_indices(nullptr),
    m_min_cell_size(64)
{
}

template<typename P, int d, typename S, typename I, int md>
KdTree<P,d,S,I,md>::KdTree(std::shared_ptr<std::vector<Point>>& points) :
    m_points(nullptr),
    m_nodes(nullptr),
    m_indices(nullptr),
    m_min_cell_size(64)
{
    this->build(points);
}

template<typename P, int d, typename S, typename I, int md>
KdTree<P,d,S,I,md>::KdTree(std::shared_ptr<std::vector<Point>>& points, const std::vector<int>& sampling) :
    m_points(nullptr),
    m_nodes(nullptr),
    m_indices(nullptr),
    m_min_cell_size(64)
{
    this->build(points, sampling);
}

template<typename P, int d, typename S, typename I, int md>
void KdTree<P,d,S,I,md>::clear()
{
    m_points  = nullptr;
    m_nodes   = nullptr;
    m_indices = nullptr;
}

template<typename P, int d, typename S, typename I, int md>
bool KdTree<P,d,S,I,md>::valid() const
{
    if(m_points == nullptr)
        return m_nodes == nullptr && m_indices == nullptr;

    if(m_nodes == nullptr || m_indices == nullptr)
    {
        KDTREE_ASSERT(false);
        return false;
    }

    if(m_points->size() < m_indices->size())
    {
        KDTREE_ASSERT(false);
        return false;
    }

    std::vector<bool> b(m_points->size(), false);
    for(int idx : *m_indices.get())
    {
        if(idx < 0 || int(m_points->size()) <= idx || b[idx])
        {
            KDTREE_ASSERT(false);
            return false;
        }
        b[idx] = true;
    }

    for(uint n=0; n<m_nodes->size(); ++n)
    {
        const Node& node = m_nodes->operator [](n);
        if(node.leaf)
        {
            if(m_indices->size() <= node.start || m_indices->size() < node.start+node.size)
            {
                KDTREE_ASSERT(false);
                return false;
            }
        }
        else
        {
            if(node.dim < 0 || 2 < node.dim)
            {
                KDTREE_ASSERT(false);
                return false;
            }
            if(m_nodes->size() <= node.first_child_id || m_nodes->size() <= node.first_child_id+1u)
            {
                KDTREE_ASSERT(false);
                return false;
            }
        }
    }

    return true;
}

template<typename P, int d, typename S, typename I, int md>
std::string KdTree<P,d,S,I,md>::to_string() const
{
    if(!m_indices) return "";

    std::stringstream str;
    str << "indices (" << m_indices->size() << ") :\n";
    for(uint i=0; i<m_indices->size(); ++i)
    {
        str << "  " << i << ": " << m_indices->operator[](i) << "\n";
    }
    str << "nodes (" << m_nodes->size() << ") :\n";
    for(uint n=0; n<m_nodes->size(); ++n)
    {
        const Node& node = m_nodes->operator[](n);
        if(node.leaf)
        {
            int end = node.start + node.size;
            str << "  leaf: start=" << node.start << " end=" << end << " (size=" << node.size << ")\n";
        }
        else
        {
            str << "  node: dim=" << node.dim << " split=" << node.split_value << " child=" << node.first_child_id << "\n";
        }
    }
    return str.str();
}

template<typename P, int d, typename S, typename I, int md>
void KdTree<P,d,S,I,md>::build(std::shared_ptr<std::vector<Point>>& points)
{
    this->clear();

    m_points = points;

    m_nodes = std::make_shared<std::vector<Node>>();
    m_nodes->reserve(4 * m_points->size() / m_min_cell_size);
    m_nodes->emplace_back();
    m_nodes->back().leaf = false;

    m_indices = std::make_shared<std::vector<int>>(m_points->size());
    std::iota(m_indices->begin(), m_indices->end(), 0);

    this->build_rec(0, 0, m_points->size(), 1);

    KDTREE_ASSERT(this->valid());
}

template<typename P, int d, typename S, typename I, int md>
void KdTree<P,d,S,I,md>::build(std::shared_ptr<std::vector<Point>>& points, const std::vector<int>& sampling)
{
    this->clear();

    m_points = points;

    m_nodes = std::make_shared<std::vector<Node>>();
    m_nodes->reserve(4 * m_points->size() / m_min_cell_size);
    m_nodes->emplace_back();
    m_nodes->back().leaf = false;

    m_indices = std::make_shared<std::vector<int>>(sampling);

    this->build_rec(0, 0, m_indices->size(), 1);

    KDTREE_ASSERT(this->valid());
}

template<typename P, int d, typename S, typename I, int md>
void KdTree<P,d,S,I,md>::rebuild(const std::vector<int>& sampling)
{
    KDTREE_ASSERT(sampling.size() <= m_points->size());

    m_nodes->clear();
    m_nodes->emplace_back();
    m_nodes->back().leaf = false;

    *m_indices = sampling;

    this->build_rec(0, 0, m_indices->size(), 1);

    KDTREE_ASSERT(this->valid());
}

// Query -----------------------------------------------------------------------

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::KNearestPointQuery
KdTree<P,d,S,I,md>::k_nearest_neighbors(const Point& point, int k) const
{
    KdTree::KNearestPointQuery q(this);
    q.set_point(point);
    q.set_k(k);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::KNearestIndexQuery
KdTree<P,d,S,I,md>::k_nearest_neighbors(int index, int k) const
{
    KdTree::KNearestIndexQuery q(this);
    q.set_index(index);
    q.set_k(k);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::Index
KdTree<P,d,S,I,md>::nearest_neighbor(const Point& point) const
{
    NearestPointQuery q(this);
    q.set_point(point);
    return *q.begin();
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::Index
KdTree<P,d,S,I,md>::nearest_neighbor(int index) const
{
    NearestIndexQuery q(this);
    q.set_index(index);
    return *q.begin();
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::RangePointQuery
KdTree<P,d,S,I,md>::range_neighbors(const Point& point, Scalar r) const
{
    KdTree::RangePointQuery q(this);
    q.set_point(point);
    q.set_radius(r);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::RangeIndexQuery
KdTree<P,d,S,I,md>::range_neighbors(int index, Scalar r) const
{
    KdTree::RangeIndexQuery q(this);
    q.set_index(index);
    q.set_radius(r);
    return q;
}

// Empty Query -----------------------------------------------------------------

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::KNearestPointQuery
KdTree<P,d,S,I,md>::k_nearest_point_query(int k) const
{
    KdTree::KNearestPointQuery q(this);
    q.set_k(k);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::KNearestIndexQuery
KdTree<P,d,S,I,md>::k_nearest_index_query(int k) const
{
    KdTree::KNearestIndexQuery q(this);
    q.set_k(k);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::NearestPointQuery
KdTree<P,d,S,I,md>::nearest_point_query() const
{
    NearestPointQuery q(this);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::NearestIndexQuery
KdTree<P,d,S,I,md>::nearest_index_query() const
{
    NearestIndexQuery q(this);
    return *q.begin();
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::RangePointQuery
KdTree<P,d,S,I,md>::range_point_query(Scalar r) const
{
    KdTree::RangePointQuery q(this);
    q.set_radius(r);
    return q;
}

template<typename P, int d, typename S, typename I, int md>
typename KdTree<P,d,S,I,md>::RangeIndexQuery
KdTree<P,d,S,I,md>::range_index_query(Scalar r) const
{
    KdTree::RangeIndexQuery q(this);
    q.set_radius(r);
    return q;
}

// Accessors -------------------------------------------------------------------

template<typename P, int d, typename S, typename I, int md>
int KdTree<P,d,S,I,md>::node_count() const
{
    return m_nodes->size();
}

template<typename P, int d, typename S, typename I, int md>
int KdTree<P,d,S,I,md>::index_count() const
{
    return m_indices->size();
}

template<typename P, int d, typename S, typename I, int md>
int KdTree<P,d,S,I,md>::point_count() const
{
    return m_points->size();
}

template<typename P, int d, typename S, typename I, int md>
const std::vector<typename KdTree<P,d,S,I,md>::Point>&
KdTree<P,d,S,I,md>::point_data() const
{
    return *m_points.get();
}

template<typename P, int d, typename S, typename I, int md>
std::vector<typename KdTree<P,d,S,I,md>::Point>&
KdTree<P,d,S,I,md>::point_data()
{
    return *m_points.get();
}

template<typename P, int d, typename S, typename I, int md>
const std::shared_ptr<std::vector<typename KdTree<P,d,S,I,md>::Point>>&
KdTree<P,d,S,I,md>::point_ptr() const
{
    return m_points;
}

template<typename P, int d, typename S, typename I, int md>
std::shared_ptr<std::vector<typename KdTree<P,d,S,I,md>::Point>>&
KdTree<P,d,S,I,md>::point_ptr()
{
    return m_points;
}

template<typename P, int d, typename S, typename I, int md>
const std::vector<typename KdTree<P,d,S,I,md>::Node>&
KdTree<P,d,S,I,md>::node_data() const
{
    return *m_nodes.get();
}

template<typename P, int d, typename S, typename I, int md>
std::vector<typename KdTree<P,d,S,I,md>::Node>&
KdTree<P,d,S,I,md>::node_data()
{
    return *m_nodes.get();
}

template<typename P, int d, typename S, typename I, int md>
const std::vector<int>& KdTree<P,d,S,I,md>::index_data() const
{
    return *m_indices.get();
}

template<typename P, int d, typename S, typename I, int md>
std::vector<int>& KdTree<P,d,S,I,md>::index_data()
{
    return *m_indices.get();
}

// Parameters ------------------------------------------------------------------

template<typename P, int d, typename S, typename I, int md>
int KdTree<P,d,S,I,md>::min_cell_size() const
{
    return m_min_cell_size;
}

template<typename P, int d, typename S, typename I, int md>
void KdTree<P,d,S,I,md>::set_min_cell_size(int min_cell_size)
{
    m_min_cell_size = min_cell_size;
}

// Internal --------------------------------------------------------------------

template<typename P, int d, typename S, typename I, int md>
void KdTree<P,d,S,I,md>::build_rec(int node_id, int start, int end, int level)
{
    auto& nodes = *m_nodes.get();
    const auto& points  = *m_points.get();
    const auto& indices = *m_indices.get();

    auto& node = nodes[node_id];

    Point aabb_min, aabb_max;
    for(auto j = 0; j < dim(); ++j)
    {
        aabb_min[j] = +std::numeric_limits<Scalar>::max();
        aabb_max[j] = -std::numeric_limits<Scalar>::max();
    }
    for(auto i = start; i < end; ++i)
    {
        for(auto j = 0; j < dim(); ++j)
        {
            aabb_min[j] = std::min(aabb_min[j], points[indices[i]][j]);
            aabb_max[j] = std::max(aabb_max[j], points[indices[i]][j]);
        }
    }

    const Point aabb_diag = (aabb_max - aabb_min) * Scalar(0.5);
    const Point aabb_center = (aabb_max + aabb_min) * Scalar(0.5);

    int dim = -1;
    aabb_diag.maxCoeff(&dim);

    node.dim = dim;
    node.split_value = aabb_center[dim];

    int midId = this->partition(start, end, dim, node.split_value);
    node.first_child_id = nodes.size();

    {
        Node n;
        n.size = 0;
        nodes.push_back(n);
        nodes.push_back(n);
    }
    {
        // left child
        int childId = nodes[node_id].first_child_id;
        Node& child = nodes[childId];
        if(midId-start <= m_min_cell_size || level >= max_depth())
        {
            child.leaf = 1;
            child.start = start;
            child.size = midId-start;
        }
        else
        {
            child.leaf = 0;
            this->build_rec(childId, start, midId, level+1);
        }
    }
    {
        // right child
        int childId = nodes[node_id].first_child_id+1;
        Node& child = nodes[childId];
        if(end-midId <= m_min_cell_size || level >= max_depth())
        {
            child.leaf = 1;
            child.start = midId;
            child.size = end-midId;
        }
        else
        {
            child.leaf = 0;
            this->build_rec(childId, midId, end, level+1);
        }
    }
}

template<typename P, int d, typename S, typename I, int md>
int KdTree<P,d,S,I,md>::partition(int start, int end, int dim, Scalar value)
{
    const auto& points = *m_points.get();
    auto& indices  = *m_indices.get();

    auto it = std::partition(indices.begin()+start, indices.begin()+end, [&](int i)
    {
        return points[i][dim] < value;
    });
    return std::distance(m_indices->begin(), it);
}

} // namespace kdtree
