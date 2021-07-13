#include <PCP/Geometry/Normal.h>
#include <PCP/Geometry/Geometry.h>

#include <PCP/Common/Assert.h>
#include <PCP/Common/Progress.h>
#include <PCP/Common/Log.h>

#include <PCP/SpacePartitioning/KnnGraph.h>

#include <set>

#include <Eigen/Eigenvalues>

// https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-using-stl-in-c/
namespace mst {

using namespace std;

// Creating shortcut for an integer pair
typedef pair<int, int> iPair;

// Structure to represent a graph
struct Graph
{
    int V, E;
    vector< pair<float, iPair> > edges;

    // Constructor
    Graph(int V, int E)
    {
        this->V = V;
        this->E = E;
    }

    // Utility function to add an edge
    void addEdge(int u, int v, float w)
    {
        edges.push_back({w, {u, v}});
    }

    // Function to find MST using Kruskal's
    // MST algorithm
    float kruskalMST(std::vector<iPair>& result);
};

// To represent Disjoint Sets
struct DisjointSets
{
    int *parent, *rnk;
    int n;

    // Constructor.
    DisjointSets(int n)
    {
        // Allocate memory
        this->n = n;
        parent = new int[n+1];
        rnk = new int[n+1];

        // Initially, all vertices are in
        // different sets and have rank 0.
        for (int i = 0; i <= n; i++)
        {
            rnk[i] = 0;

            //every element is parent of itself
            parent[i] = i;
        }
    }

    // Find the parent of a node 'u'
    // Path Compression
    int find(int u)
    {
        /* Make the parent of the nodes in the path
           from u--> parent[u] point to parent[u] */
        if (u != parent[u])
            parent[u] = find(parent[u]);
        return parent[u];
    }

    // Union by rank
    void merge(int x, int y)
    {
        x = find(x), y = find(y);

        /* Make tree with smaller height
           a subtree of the other tree  */
        if (rnk[x] > rnk[y])
            parent[y] = x;
        else // If rnk[x] <= rnk[y]
            parent[x] = y;

        if (rnk[x] == rnk[y])
            rnk[y]++;
    }
};

 /* Functions returns weight of the MST*/

float Graph::kruskalMST(std::vector<iPair>& result)
{
    result.clear();
    float mst_wt = 0; // Initialize result

    // Sort edges in increasing order on basis of cost
    sort(edges.begin(), edges.end());

    // Create disjoint sets
    DisjointSets ds(V);

    // Iterate through all sorted edges
    vector< pair<float, iPair> >::iterator it;
    for (it=edges.begin(); it!=edges.end(); it++)
    {
        int u = it->second.first;
        int v = it->second.second;

        int set_u = ds.find(u);
        int set_v = ds.find(v);

        // Check if the selected edge is creating
        // a cycle or not (Cycle is created if u
        // and v belong to same set)
        if (set_u != set_v)
        {
            // Current edge will be in the MST
            // so print it
//            cout << u << " - " << v << endl;
            result.push_back(std::make_pair(u, v));

            // Update MST weight
            mst_wt += it->first;

            // Merge two sets
            ds.merge(set_u, set_v);
        }
    }

    return mst_wt;
}

// for my std::set
struct Comp
{
    // compare only the first index
    bool operator()(const pair<float, iPair>& e1, const pair<float, iPair>& e2)
    {
        return e1.second.first < e2.second.first;
    }
};

} // namespace mst

namespace pcp {

void compute_normals(Geometry& g, bool v)
{
    PCP_ASSERT(g.has_knn_graph());

    const int point_count = g.size();

    info().iff(v) << "0/5 Computing normals";
    internal::compute_unoriented_normals(g, v);

    info().iff(v) << "1/5 Extracting unique edges";
    std::set<std::pair<int,int>> edges;
    auto prog = Progress(point_count, v);
    for(int i=0; i<point_count; ++i)
    {
        for(int j : g.knn_graph().k_nearest_neighbors(i))
        {
            auto e = std::make_pair(i,j);
            if(e.first > e.second) std::swap(e.first, e.second);

            edges.insert(e);
        }
        ++prog;
    }
    info().iff(v) << "  " << edges.size() << " edges found";

    info().iff(v) << "2/5 Building graph for mst";
    mst::Graph graph(point_count, edges.size());
    for(const auto& e : edges)
    {
        const int i = e.first;
        const int j = e.second;
        const Vector3& n_i = g.normal(i);
        const Vector3& n_j = g.normal(j);
        const Scalar w = 1 - std::abs(n_i.dot(n_j));
        graph.addEdge(i, j, w);
    }

    info().iff(v) << "3/5 Computing mst";
    std::vector<mst::iPair> result;
    graph.kruskalMST(result);
    info().iff(v) << "  " << result.size() << " edges found in mst";

    info().iff(v) << "4. Building mst";
    std::vector<std::vector<int>> mst(point_count);
    for(const auto& e : result)
    {
        mst[e.first].push_back(e.second);
        mst[e.second].push_back(e.first);
    }

    info().iff(v) << "5/5 Propagate orientention";
    int topmost = 0;
    for(int i=0; i<point_count; ++i)
    {
        if(g[i].z() > g[topmost].z()) topmost = i;
    }
    std::stack<mst::iPair> stack;
    std::vector<bool> traversed(point_count, false);
    traversed[topmost] = true;
    for(int i : mst[topmost]) stack.push(std::make_pair(topmost, i));

    while(!stack.empty())
    {
        const auto e = stack.top();
        stack.pop();

        const int source = e.first;
        const int target = e.second;

        const Vector3& n_source = g.normal(source);
        Vector3& n_target = g.normal(target);

        if(n_source.dot(n_target) < 0) n_target *= -1;

        traversed[target] = true;

        for(int i : mst[target]) if(!traversed[i]) stack.push(std::make_pair(target, i));
    }
}

void compute_normals_robust(Geometry& g, float sigma, int iter, bool v)
{
    PCP_ASSERT(g.has_knn_graph());

    const int point_count = g.size();

    info().iff(v) << "0/5 Computing normals";
    internal::compute_unoriented_normals_robust(g, sigma, iter, v);

    info().iff(v) << "1/5 Extracting unique edges";
    std::set<std::pair<int,int>> edges;
    auto prog = Progress(point_count, v);
    for(int i=0; i<point_count; ++i)
    {
        for(int j : g.knn_graph().k_nearest_neighbors(i))
        {
            auto e = std::make_pair(i,j);
            if(e.first > e.second) std::swap(e.first, e.second);

            edges.insert(e);
        }
        ++prog;
    }
    info().iff(v) << "  " << edges.size() << " edges found";

    info().iff(v) << "2/5 Building graph for mst";
    mst::Graph graph(point_count, edges.size());
    for(const auto& e : edges)
    {
        const int i = e.first;
        const int j = e.second;
        const Vector3& n_i = g.normal(i);
        const Vector3& n_j = g.normal(j);
        const Scalar w = 1 - std::abs(n_i.dot(n_j));
        graph.addEdge(i, j, w);
    }

    info().iff(v) << "3/5 Computing mst";
    std::vector<mst::iPair> result;
    graph.kruskalMST(result);
    info().iff(v) << "  " << result.size() << " edges found in mst";

    info().iff(v) << "4. Building mst";
    std::vector<std::vector<int>> mst(point_count);
    for(const auto& e : result)
    {
        mst[e.first].push_back(e.second);
        mst[e.second].push_back(e.first);
    }

    info().iff(v) << "5/5 Propagate orientention";
    int topmost = 0;
    for(int i=0; i<point_count; ++i)
    {
        if(g[i].z() > g[topmost].z()) topmost = i;
    }
    std::stack<mst::iPair> stack;
    std::vector<bool> traversed(point_count, false);
    traversed[topmost] = true;
    for(int i : mst[topmost]) stack.push(std::make_pair(topmost, i));

    while(!stack.empty())
    {
        const auto e = stack.top();
        stack.pop();

        const int source = e.first;
        const int target = e.second;

        const Vector3& n_source = g.normal(source);
        Vector3& n_target = g.normal(target);

        if(n_source.dot(n_target) < 0) n_target *= -1;

        traversed[target] = true;

        for(int i : mst[target]) if(!traversed[i]) stack.push(std::make_pair(target, i));
    }
}

namespace internal {

void compute_unoriented_normals(Geometry& g, bool v)
{
    const int point_count = g.size();
    const int K = g.knn_graph().k();

    g.request_normals();

    auto prog = Progress(point_count, v);

    #pragma omp parallel for
    for(int i=0; i<point_count; ++i)
    {
        Matrix3 C = Matrix3::Zero();
        Vector3 m = Vector3::Zero();

        for(int j : g.knn_graph().k_nearest_neighbors(i))
        {
            const Vector3 p = g[j] - g[i];

            C += p * p.transpose();
            m += p;
        }
        m /= K;
        C = C/K - m * m.transpose();

        Eigen::SelfAdjointEigenSolver<Matrix3> eig(C);
        g.normal(i) = eig.eigenvectors().col(0);

        ++prog;
    }
}

namespace internal {
Scalar robust_weight(const Vector3& p, Scalar uc, const Vector3& ul, Scalar sigma)
{
    const Scalar f = uc + ul.dot(p);
    return std::exp(-f*f/(sigma*sigma));
}
} // namespace internal

void compute_unoriented_normals_robust(Geometry& g, float sigma, int iter, bool v)
{
    const int point_count = g.size();
//    const int K = g.knn_graph().k();

    g.request_normals();

    auto prog = Progress(point_count, v);

    #pragma omp parallel for
    for(int i=0; i<point_count; ++i)
    {
        Scalar  uc = 0;
        Vector3 ul = Vector3::Zero();

        for(int n=0; n<iter; ++n)
        {
            Matrix3 C = Matrix3::Zero();
            Vector3 m = Vector3::Zero();
            Scalar sum_w = 0;

            for(int j : g.knn_graph().k_nearest_neighbors(i))
            {
                const Vector3 p = g[j] - g[i];
                const Scalar w = n == 0 ? 1 : internal::robust_weight(p, uc, ul, sigma);

                C += w * p * p.transpose();
                m += w * p;
                sum_w += w;
            }

            if(sum_w <= 0)
            {
                warning().iff(v) << "sum_w=0 at i=" << i << " (n=" << n << ")";
//                PCP_ERROR;
                break;
            }

            m /= sum_w;
            C = C/sum_w - m * m.transpose();

            Eigen::SelfAdjointEigenSolver<Matrix3> eig(C);
            ul = eig.eigenvectors().col(0);
            uc = - ul.dot(m);
        }

        g.normal(i) = ul;

        ++prog;
    }
}


} // namespace internal

} // namespace pcp
