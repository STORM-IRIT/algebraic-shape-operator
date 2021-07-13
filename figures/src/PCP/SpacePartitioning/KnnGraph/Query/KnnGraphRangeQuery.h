#pragma once

#include <PCP/SpacePartitioning/Query/RangeIndexQuery.h>
#include <PCP/SpacePartitioning/KnnGraph/Iterator/KnnGraphRangeIterator.h>

#include <vector>
#include <stack>

namespace pcp {

class KnnGraph;

class KnnGraphRangeQuery : public RangeIndexQuery
{
protected:
    friend class KnnGraphRangeIterator;

public:
    KnnGraphRangeQuery();
    KnnGraphRangeQuery(const KnnGraph* graph);
    KnnGraphRangeQuery(const KnnGraph* graph, Scalar radius);
    KnnGraphRangeQuery(const KnnGraph* graph, Scalar radius, int index);

public:
    KnnGraphRangeIterator begin();
    KnnGraphRangeIterator end();

protected:
    void initialize(KnnGraphRangeIterator& iterator);
    void advance(KnnGraphRangeIterator& iterator);

protected:
    const KnnGraph*   m_graph;
    std::vector<bool> m_flag;
    std::stack<int>   m_stack;
};

} // namespace pcp
