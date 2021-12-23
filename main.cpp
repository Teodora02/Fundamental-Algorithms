#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>

using namespace std;

class Algorithms
{
private:
    Algorithms()
    {}

public:
    static void DFS( const vector<vector<int>> &adjacencyList, int startNode, vector<bool> &visited )
    {
        stack<int> nodes;
        nodes.push( startNode );

        while( !nodes.empty() )
        {
            int crtNode = nodes.top();
            nodes.pop();

            if( visited[crtNode] )
                continue;

            visited[crtNode] = true;
            for( auto node : adjacencyList[crtNode] )
            {
                if( !visited[node] )
                    nodes.push( node );
            }
        }
    }

    static void BFS( const vector<vector<int>> &adjacencyList, int startNode, vector<int> &distance )
    {
        queue<pair<int, int>> nodes;
        nodes.push( { startNode, 0 } );
        distance[startNode] = 0;

        while( !nodes.empty() )
        {
            int crtNode = nodes.front().first, crtDistance = nodes.front().second;
            nodes.pop();

            for( auto node : adjacencyList[crtNode] )
            {
                if( distance[node] == -1 )
                {
                    distance[node] = crtDistance + 1;
                    nodes.push( { node, distance[node] } );
                }
            }
        }
    }

    static vector<vector<int>> getComponenteBiconexe( const vector<vector<int>> &adjacencyList, int n )
    {
        vector<bool>        visited( n + 1 );
        vector<int>         levels( n + 1 );
        vector<int>         minLevels( n + 1 );
        vector<vector<int>> componenteBiconexe;
        stack<int>          componenta;
        for( int i = 1; i < adjacencyList.size(); ++i )
        {
            componenteBiconexeHelper( adjacencyList, i, 0, 0, levels, minLevels, visited, componenta,
                                      componenteBiconexe );
        }

        return componenteBiconexe;
    }

    static vector<vector<int>> getComponenteTareConexe( const vector<vector<int>> &adjacencyList, int n,
                            const vector<vector<int>> &transposeList )
    {
        vector<bool> visited( n + 1 );
        stack<int>   traversal;
        for( int i = 1; i <= n; ++i )
            if( !visited[i] )
                componenteTareConexeHelper( adjacencyList, i, visited, traversal );

        vector<vector<int>> componente;
        for( ; !traversal.empty(); traversal.pop() )
        {
            int x = traversal.top();
            if( visited[x] )
            {
                componente.push_back( vector<int>() );
                componenteTareConexeHelperTranspose( transposeList, x, visited, componente.back() );
            }
        }

        return componente;
    }

    static vector<int> getDijkstra( const vector<vector<pair<int, int>>> &adjacencyList, int n, int m )
    {
        vector<int> d( n + 1, INT_MAX ), visited( n + 1 );
        d[1] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> q;
        q.push( { 0, 1 } );
        while( !q.empty() )
        {
            int node = q.top().second;
            q.pop();
            if( visited[node] )
                continue;

            visited[node] = true;
            for( auto neighbour : adjacencyList[node] )
            {
                if( d[node] + neighbour.second < d[neighbour.first] )
                {
                    d[neighbour.first] = d[node] + neighbour.second;
                    q.push( { d[neighbour.first], neighbour.first } );
                }
            }
        }

        return d;
    }

    static vector<int> getBellmanFord( const vector<vector<pair<int, int>>> &adjacencyList, int n, int m )
    {
        vector<int> d( n + 1, INT_MAX ), visited( n + 1 );
        d[1] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> q;
        q.push( { 0, 1 } );
        while( !q.empty() )
        {
            int node = q.top().second;
            q.pop();
            visited[node]++;
            if( visited[node] >= n )
            {
                d.resize( 0 );
                break;
            }

            for( auto neighbour : adjacencyList[node] )
                if( d[node] + neighbour.second < d[neighbour.first] )
                {
                    d[neighbour.first] = d[node] + neighbour.second;
                    q.push( make_pair( d[neighbour.first], neighbour.first ) );
                }
        }

        return d;
    }

    static void getRoyFloyd( int n, vector<vector<int>> &costMatrix )
    {
        for( int k = 1; k <= n; k++ )
            for( int i = 1; i <= n; i++ )
                for( int j = 1; j <= n; j++ )
                    if( i != j && costMatrix[i][k] != 0 && costMatrix[k][j] != 0 &&
                            ( costMatrix[i][j] > costMatrix[i][k] + costMatrix[k][j] || costMatrix[i][j] == 0 ) )
                        costMatrix[i][j] = costMatrix[i][k] + costMatrix[k][j];
    }

    static int findRoot( int node, vector<int> &t )
    {
        if( t[node] == node )
            return node;

        t[node] = findRoot( t[node], t );

        return t[node];
    }

    static void unite( int node1, int node2, vector<int> &t )
    {
        t[Algorithms::findRoot( node2, t )] = Algorithms::findRoot( node1, t );
    }

    static int getDiameter( const vector<vector<int>> &adjacencyList, int n )
    {
        int         maxDistance = -1, furthestNode, furthestNode2;
        vector<int> distance( n + 1, -1 );

        Algorithms::BFS( adjacencyList, 1, distance );

        for( int i = 1; i <= n; ++i )
        {
            if( distance[i] > maxDistance )
            {
                maxDistance  = distance[i];
                furthestNode = i;
            }
        }

        fill( distance.begin(), distance.end(), -1 );  // reinitializez distanta la -1

        Algorithms::BFS( adjacencyList, furthestNode, distance );

        maxDistance = -1;
        for( int i = 1; i <= n; ++i )
        {
            if( distance[i] > maxDistance )
            {
                maxDistance = distance[i];
            }
        }

        return maxDistance + 1;
    }

    static vector<int> sortTopological( const vector<vector<int>> &adjacencyList, int n )
    {
        vector<bool> visited( n + 1 );
        stack<int>   traversal;
        Algorithms::sortTopologicalHelper( adjacencyList, 1, visited, traversal );

        vector<int> sorted;
        while( !traversal.empty() )
        {
            sorted.push_back( traversal.top() );
            traversal.pop();
        }

        return sorted;
    }

    static vector<vector<int>> havelHakimi( vector<int> &v, int n )
    {
        vector<vector<int>> adjacencyList( n + 1 );
        int                 crtFirst = 1;  // primul element nenul
        while( true )
        {
            int i;
            for( i = crtFirst + 1; i < v.size(); ++i )
            {
                v[i] -= v[crtFirst];
                --v[crtFirst];

                adjacencyList[crtFirst].push_back( i );
            }

            if( v[crtFirst] > 0 )
            {
                return vector<vector<int>>();
            }

            sort( v.begin() + crtFirst, v.begin() + n + 1, less<int>() );

            while( v[crtFirst] == 0 )
                ++crtFirst;

            reverse( v.begin() + crtFirst, v.begin() + n + 1 );

            if( v[n] == 0 )
            {
                return adjacencyList;
            }
        }
    }

private:
    static void componenteBiconexeHelper( const vector<vector<int>> &adjacencyList, int crtNode, int parent,
                                          int crtLevel, vector<int> &levels, vector<int> &minLevels,
                                          vector<bool> &visited, stack<int> &componenta,
                                          vector<vector<int>> &componenteBiconexe )
    {
        visited[crtNode]   = true;
        levels[crtNode]    = crtLevel;
        minLevels[crtNode] = crtLevel;
        componenta.push( crtNode );

        for( auto node : adjacencyList[crtNode] )
        {
            if( !visited[node] )
            {
                int capNode = componenta.top();

                componenteBiconexeHelper( adjacencyList, node, crtNode, crtLevel + 1, levels, minLevels, visited,
                                          componenta, componenteBiconexe );

                if( minLevels[node] >= levels[crtNode] || ( parent == 0 && adjacencyList[crtNode].size() > 1 ) )
                {
                    componenteBiconexe.push_back( vector<int>( { crtNode } ) );
                    while( componenta.top() != capNode )
                    {
                        componenteBiconexe.back().push_back( componenta.top() );
                        componenta.pop();
                    }
                }

                minLevels[crtNode] = min( minLevels[crtNode], minLevels[node] );
            }
            else if( node != parent )
                minLevels[crtNode] = min( minLevels[crtNode], levels[node] );
        }
    }

    static void componenteTareConexeHelper( const vector<vector<int>> &adjacencyList, int crtNode,
                                            vector<bool> &visited, stack<int> &traversal )
    {
        visited[crtNode] = true;
        for( auto node : adjacencyList[crtNode] )
            if( !visited[node] )
                componenteTareConexeHelper( adjacencyList, node, visited, traversal );

        traversal.push( crtNode );
    }

    static void componenteTareConexeHelperTranspose( const vector<vector<int>> &adjacencyList, int crtNode,
            vector<bool> &visited, vector<int> &componenta )
    {
        visited[crtNode] = false;
        componenta.push_back( crtNode );
        for( auto node : adjacencyList[crtNode] )
            if( visited[node] )
                componenteTareConexeHelperTranspose( adjacencyList, node, visited, componenta );
    }

    static void sortTopologicalHelper( const vector<vector<int>> &adjacencyList, int crtNode, vector<bool> &visited,
                                       stack<int> &traversal )
    {
        visited[crtNode] = true;
        for( auto node : adjacencyList[crtNode] )
            if( !visited[node] )
                sortTopologicalHelper( adjacencyList, node, visited, traversal );

        traversal.push( crtNode );
    }
};

void solveDFS()
{
    int      n, m;
    ifstream in( "dfs.in" );
    in >> n >> m;
    vector<vector<int>> adjacencyList( n + 1 );

    int x, y;
    for( int i = 0; i < m; ++i )
    {
        in >> x >> y;
        adjacencyList[x].push_back( y );
        adjacencyList[y].push_back( x );
    }

    int          nrComponents = 0;
    vector<bool> visited( n + 1 );
    ofstream     out( "dfs.out" );
    for( int startNode = 1; startNode <= n; ++startNode )
        if( !visited[startNode] )
        {
            ++nrComponents;
            Algorithms::DFS( adjacencyList, startNode, visited );
        }

    out << nrComponents;
}

void solveBFS()
{
    int      n, m, S;
    ifstream in( "bfs.in" );
    in >> n >> m >> S;
    vector<vector<int>> adjacencyList( n + 1 );

    int x, y;
    for( int i = 0; i < m; ++i )
    {
        in >> x >> y;
        adjacencyList[x].push_back( y );
    }

    vector<int> distance( n + 1, -1 );
    ofstream    out( "bfs.out" );
    Algorithms::BFS( adjacencyList, S, distance );
    for( int i = 1; i <= n; ++i )
    {
        out << distance[i] << ' ';
    }
}

void solveBiconex()
{
    int      n, m;
    ifstream in( "biconex.in" );
    in >> n >> m;
    vector<vector<int>> adjacencyList( n + 1 );

    int x, y;
    for( int i = 0; i < m; ++i )
    {
        in >> x >> y;
        adjacencyList[x].push_back( y );
        adjacencyList[y].push_back( x );
    }

    // dintr-un descdendent cu un back-edge
    vector<vector<int>> componenteBiconexe = Algorithms::getComponenteBiconexe( adjacencyList, n );

    ofstream out( "biconex.out" );
    out << componenteBiconexe.size() << '\n';
    for( const auto &componenta : componenteBiconexe )
    {
        for( auto node : componenta )
            out << node << ' ';
        out << '\n';
    }
}

void solveTareConex()
{
    int      n, m;
    ifstream in( "ctc.in" );
    in >> n >> m;
    vector<vector<int>> adjacencyList( n + 1 );
    vector<vector<int>> transposeList( n + 1 );

    int x, y;
    for( int i = 0; i < m; ++i )
    {
        in >> x >> y;
        adjacencyList[x].push_back( y );
        transposeList[y].push_back( x );
    }

    vector<vector<int>> componenteTareConexe = Algorithms::getComponenteTareConexe( adjacencyList, n, transposeList );

    ofstream out( "ctc.out" );
    out << componenteTareConexe.size() << '\n';
    for( const auto &componenta : componenteTareConexe )
    {
        for( auto node : componenta )
            out << node << ' ';
        out << '\n';
    }
}

void solveDijkstra()
{
    int      n, m;
    ifstream in( "dijkstra.in" );
    ofstream out( "dijkstra.out" );
    in >> n >> m;
    vector<vector<pair<int, int>>> v( n + 1 );
    for( int i = 1; i <= m; i++ )
    {
        int x, y, cost;
        in >> x >> y >> cost;
        v[x].push_back( make_pair( y, cost ) );
    }
    vector<int> dijkstra = Algorithms::getDijkstra( v, n, m );
    for( int i = 2; i <= n; i++ )
    {
        if( dijkstra[i] == INT_MAX )
            out << "0 ";
        else
            out << dijkstra[i] << ' ';
    }
}

void solveBellmanFord()
{
    int      n, m;
    ifstream in( "bellmanford.in" );
    ofstream out( "bellmanford.out" );
    in >> n >> m;
    vector<vector<pair<int, int>>> v( n + 1 );
    for( int i = 1; i <= m; i++ )
    {
        int x, y, cost;
        in >> x >> y >> cost;
        v[x].push_back( make_pair( y, cost ) );
    }
    vector<int> bellmanFord = Algorithms::getBellmanFord( v, n, m );
    if( bellmanFord.size() != 0 )
    {
        for( int i = 2; i <= n; i++ )
        {
            if( bellmanFord[i] == INT_MAX )
                out << "0 ";
            else
                out << bellmanFord[i] << ' ';
        }
    }
    else
        out << "Ciclu negativ!";
}

void solveDisjoint()
{
    ifstream in( "disjoint.in" );
    ofstream out( "disjoint.out" );
    int      k, x, y, n, m;
    in >> n >> m;

    vector<int> t( n + 1 );
    for( int i = 1; i <= n; i++ )
        t[i] = i;

    for( int i = 1; i <= m; i++ )
    {
        in >> k >> x >> y;
        if( k == 1 )
            Algorithms::unite( x, y, t );
        else
        {
            if( Algorithms::findRoot( x, t ) == Algorithms::findRoot( y, t ) )
                out << "DA\n";
            else
                out << "NU\n";
        }
    }
}

void solveDarb()
{
    ifstream in( "darb.in" );
    int      n;
    in >> n;
    vector<vector<int>> adjacencyList( n + 1 );

    for( int i = 0; i < n - 1; ++i )
    {
        int x, y;
        in >> x >> y;
        adjacencyList[x].push_back( y );
        adjacencyList[y].push_back( x );
    }

    ofstream out( "darb.out" );
    out << Algorithms::getDiameter( adjacencyList, n );
}

void solveRoyFloyd()
{
    ifstream in( "royfloyd.in" );
    int      n;
    in >> n;
    vector<vector<int>> costMatrix( n + 1,
                                    vector<int>( n + 1 ) );

    for( int i = 1; i <= n; i++ )
        for( int j = 1; j <= n; j++ )
            in >> costMatrix[i][j];

    Algorithms::getRoyFloyd( n, costMatrix );

    ofstream out( "royfloyd.out" );
    for( int i = 1; i <= n; i++ )
    {
        for( int j = 1; j <= n; j++ )
            out << costMatrix[i][j] << " ";
        out << endl;
    }
}

void solveSortTopological()
{
    ifstream in( "sortaret.in" );
    int      n, m;
    in >> n >> m;
    vector<vector<int>> adjacencyList( n + 1 );

    int x, y;
    for( int i = 0; i < m; ++i )
    {
        in >> x >> y;
        adjacencyList[x].push_back( y );
    }

    vector<int> sorted = Algorithms::sortTopological( adjacencyList, n );

    ofstream out( "sortaret.out" );
    for( auto node : sorted )
    {
        out << node << ' ';
    }
}

void solveHavelHakimi()
{
    int      n;
    ifstream in( "hakimi.in" );
    in >> n;
    vector<int> v( n + 1 );

    for( int i = 1; i <= n; ++i )
    {
        in >> v[i];
    }

    auto adjacencyList = Algorithms::havelHakimi( v, n );

    ofstream out( "hakimi.out" );
    if( adjacencyList.size() == 0 )
    {
        out << "imposibil\n";
    }
    else
    {
        for( int i = 1; i <= n; ++i )
        {
            for( auto node : adjacencyList[i] )
            {
                out << i << ' ' << node << '\n';
            }
        }
    }
}

int main()
{
    solveDFS();
    // solveBFS();
    // solveBiconex();
    // solveTareConex();
    // solveDijkstra();
    // solveBellmanFord();
    // solveDisjoint();
    // solveDarb();
    // solveRoyFloyd();
    // solveSortTopological();
    // solveHavelHakimi();
}
