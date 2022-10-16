#ifndef TOPOLOGICAL_SORTING_H
#define TOPOLOGICAL_SORTING_H
#include <list>
#include <stack>
#include <vector>
class TopologicalSorting
{
    int V;
    std::vector<std::list<int>> adj;
    void sortUtil(int v, std::vector<bool> &visited, std::stack<int>& Stack)
    {
        visited[v] = true;
        std::list<int>::iterator i;
        for (i = adj[v].begin(); i != adj[v].end(); ++i)
            if (!visited[*i])
                sortUtil(*i, visited, Stack);
        Stack.push(v);
    }

    bool isCyclicUtil(int v, std::vector<bool> &visited, std::vector<bool> &recStack)  // used by isCyclic()
    {
        if(visited[v] == false)
        {
            // Mark the current node as visited and part of recursion stack
            visited[v] = true;
            recStack[v] = true;

            // Recur for all the vertices adjacent to this vertex
            std::list<int>::iterator i;
            for(i = adj[v].begin(); i != adj[v].end(); ++i)
            {
                if ( !visited[*i] && isCyclicUtil(*i, visited, recStack) )
                    return true;
                else if (recStack[*i])
                    return true;
            }
        }
        recStack[v] = false;  // remove the vertex from recursion stack
        return false;
    }

public:
    TopologicalSorting(int V)
    {
        this->V = V;
        adj = std::vector<std::list<int>>(V);
    }


    void addEdge(int v, int w)
    {
        adj[v].push_back(w);
    }

    bool isCyclic()
    {
        std::vector<bool> visited(V, false), recStack(V, false);
        for(int i = 0; i < V; i++)
            if (isCyclicUtil(i, visited, recStack))
                return true;
        return false;
    }

    std::vector<int> sort()
    {
        std::stack<int> Stack;
        std::vector<bool> visited(V, false);
        for (int i = 0; i < V; i++)
            if (visited[i] == false)
                sortUtil(i, visited, Stack);
        std::vector<int> result(0);
        while (Stack.empty() == false)
        {
            result.push_back(Stack.top());
            Stack.pop();
        }
        return result;
    }
};
#endif // TOPOLOGICAL_SORTING_H
