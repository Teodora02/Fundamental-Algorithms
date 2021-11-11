#include <algorithm>
#include <fstream>
#include <queue>
#include <stack>
#include <vector>

using namespace std;

class Algorithms
{
public:
    // Parcurge graful in adancime si marcheaza nodurile vizitate in vectorul
    // visited. adjacencyList - lista de adiacenta a grafului startNode - nodul
    // din care incepe parcurgerea visited - vectorul in care marcam nodurile
    // vizitate
    static void DFS( const vector<vector<int>> &adjacencyList, int startNode, vector<bool> &visited )
    {
        stack<int> nodes;
        nodes.push( startNode );  // adaugam nodul de start in stiva

        while( !nodes.empty() ) {
            // scoatem varful stivei
            int crtNode = nodes.top();
            nodes.pop();

            if( visited[crtNode] )  // se poate sa fi vizitat deja nodul (un nod
                                    // poate aparea de 2 ori in stiva)
                continue;

            visited[crtNode] = true;  // vizitam nodul
            // bagam vecinii nevizitati in stiva
            for( auto node : adjacencyList[crtNode] ) {
                if( !visited[node] )
                    nodes.push( node );
            }
        }
    }

    static void BFS( const vector<vector<int>> &adjacencyList, int startNode, vector<int> &distance )
    {
        queue<pair<int, int>> nodes;     // retinem si nodul si distanta fata de start
        nodes.push( { startNode, 0 } );  // adaugam nodul de start in coada
        distance[startNode] = 0;         // vizitam nodul de start

        while( !nodes.empty() ) {
            // scoatem primul nod din coada
            int crtNode = nodes.front().first, crtDistance = nodes.front().second;
            nodes.pop();

            // bagam vecinii nevizitati in coada
            for( auto node : adjacencyList[crtNode] ) {
                if( distance[node] == -1 ) {
                    distance[node] = crtDistance + 1;  // vizitam vecinul
                    nodes.push( { node, distance[node] } );
                }
            }
        }
    }

    // Returneaza componentele biconexe ale grafului sub forma de vector de
    // vector de noduri. adjacencyList - lista de adiacenta a grafului levels -
    // vectorul nivelurilor nodurilor in arborele DFS minLevels - contine
    // nivelurile minime din care se poate ajunge din fiecare nod coborand in
    // jos si urcand pe
    //             o muchie de intoarcere in arborele DFS
    // visited - vectorul in care marcam nodurile vizitate
    static vector<vector<int>> getComponenteBiconexe( const vector<vector<int>> &adjacencyList, vector<int> &levels,
                                                      vector<int> &minLevels, vector<bool> &visited )
    {
        vector<vector<int>> componenteBiconexe;
        stack<int>          componenta;
        for( int i = 1; i < adjacencyList.size(); ++i ) {
            componenteBiconexeHelper( adjacencyList, i, 0, 0, levels, minLevels, visited, componenta,
                                      componenteBiconexe );
        }

        return componenteBiconexe;
    }

    static vector<vector<int>> getComponenteTareConexe( const vector<vector<int>> &adjacencyList, const vector<vector<int>> &transposeList,
                                                        vector<bool> &             visited )
    {
        stack<int> traversal;
        for( int i = 1; i < adjacencyList.size(); ++i )
            if( !visited[i] )
                componenteTareConexeHelper( adjacencyList, i, visited, traversal );

        vector<vector<int>> componente;
        for( ; !traversal.empty(); traversal.pop() ) {
            int x = traversal.top();
            if( visited[x] ) {
                componente.push_back( vector<int>() );
                componenteTareConexeHelperTranspose( transposeList, x, visited, componente.back() );
            }
        }

        return componente;
    }

private:
    // Calculeaza componentele biconexe ale componentei conexe curente folosind
    // o parcurgere DFS si returneaza un vector ce contine nodurile pentru
    // fiecare componenta. adjacencyList - lista de adiacenta a grafului crtNode
    // - nodul din care pornim parcurgerea DFS parent - parintele nodului in
    // arborele DFS crtLevel - nivelul pe care se afla nodul curent in arborele
    // DFS levels - vectorul nivelurilor nodurilor in arborele DFS minLevels -
    // contine nivelurile minime din care se poate ajunge din fiecare nod
    // coborand in jos si urcand pe
    //             o muchie de intoarcere in arborele DFS
    // visited - vectorul in care marcam nodurile vizitate
    // componenta - stiva care retine nodurile acumulate ce fac parte din
    // aceeasi componenta biconexa
    static void componenteBiconexeHelper( const vector<vector<int>> &adjacencyList, int crtNode, int parent,
                                          int crtLevel, vector<int> &levels, vector<int> &minLevels,
                                          vector<bool> &visited, stack<int> &componenta,
                                          vector<vector<int>> &componenteBiconexe )
    {
        visited[crtNode]   = true;
        levels[crtNode]    = crtLevel;
        minLevels[crtNode] = crtLevel;
        componenta.push( crtNode );

        for( auto node : adjacencyList[crtNode] ) {
            if( !visited[node] ) {
                int capNode = componenta.top();  // varful stivei pentru iteratia copilului
                                                 // curent; aici ne oprim cu extragerea
                                                 // nodurilor pentru componenta biconexa
                componenteBiconexeHelper( adjacencyList, node, crtNode, crtLevel + 1, levels, minLevels, visited,
                                          componenta, componenteBiconexe );
                // daca nodul are un copil din care nu putem ajunge coborand in
                // jos si urcand pe o muchie de intoarcere in arborele DFS catre
                // un stramos al nodului curent, atunci acesta este punct de
                // articulatie; caz special: radacina arborelui DFS este punct
                // de articulatie iff are cel putin 2 copii
                if( minLevels[node] >= levels[crtNode] || ( parent == 0 && adjacencyList[crtNode].size() > 1 ) ) {
                    // scoatem nodurile din stiva pana la capNode pentru a le
                    // insera in componenta biconexa inseram de asemenea si
                    // nodul curent
                    componenteBiconexe.push_back( vector<int>( { crtNode } ) );
                    while( componenta.top() != capNode ) {
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

    // DFS
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
};

void solveDFS()
{
    int      N, M;
    ifstream in( "dfs.in" );
    in >> N >> M;
    vector<vector<int>> adjacencyList( N + 1 );

    int x, y;
    for( int i = 0; i < M; ++i ) {
        in >> x >> y;
        adjacencyList[x].push_back( y );
        adjacencyList[y].push_back( x );
    }

    int          nrComponents = 0;
    vector<bool> visited( N + 1 );
    ofstream     out( "dfs.out" );
    for( int startNode = 1; startNode <= N; ++startNode )
        if( !visited[startNode] ) {
            ++nrComponents;
            Algorithms::DFS( adjacencyList, startNode, visited );
        }

    out << nrComponents;
}

void solveBFS()
{
    int      N, M, S;
    ifstream in( "bfs.in" );
    in >> N >> M >> S;
    vector<vector<int>> adjacencyList( N + 1 );

    int x, y;
    for( int i = 0; i < M; ++i ) {
        in >> x >> y;
        adjacencyList[x].push_back( y );
    }

    vector<int> distance( N + 1, -1 );
    ofstream    out( "bfs.out" );
    Algorithms::BFS( adjacencyList, S, distance );
    for( int i = 1; i <= N; ++i ) {
        out << distance[i] << ' ';
    }
}

void solveBiconex()
{
    int      N, M;
    ifstream in( "biconex.in" );
    in >> N >> M;
    vector<vector<int>> adjacencyList( N + 1 );

    int x, y;
    for( int i = 0; i < M; ++i ) {
        in >> x >> y;
        adjacencyList[x].push_back( y );
        adjacencyList[y].push_back( x );
    }

    vector<bool> visited( N + 1 );
    vector<int>  levels( N + 1 );     // nivelul nodurilor in arborele DFS
    vector<int>  minLevels( N + 1 );  // nivelul minim pe care se poate ajunge
                                      // dintr-un descdendent cu un back-edge
    vector<vector<int>> componenteBiconexe =
        Algorithms::getComponenteBiconexe( adjacencyList, levels, minLevels, visited );

    ofstream out( "biconex.out" );
    out << componenteBiconexe.size() << '\n';
    for( const auto &componenta : componenteBiconexe ) {
        for( auto node : componenta )
            out << node << ' ';
        out << '\n';
    }
}

void solveTareConex()
{
    int      N, M;
    ifstream in( "ctc.in" );
    in >> N >> M;
    vector<vector<int>> adjacencyList( N + 1 );
    vector<vector<int>> transposeList( N + 1 );

    int x, y;
    for( int i = 0; i < M; ++i ) {
        in >> x >> y;
        adjacencyList[x].push_back( y );
        transposeList[y].push_back( x );
    }

    vector<bool>        visited( N + 1 );
    vector<vector<int>> componenteTareConexe = Algorithms::getComponenteTareConexe( adjacencyList, transposeList, visited );

    ofstream out( "ctc.out" );
    out << componenteTareConexe.size() << '\n';
    for( const auto &componenta : componenteTareConexe ) {
        for( auto node : componenta )
            out << node << ' ';
        out << '\n';
    }
}

int main()
{
    // solveDFS();
    // solveBFS();
    // solveBiconex();
    solveTareConex();
}