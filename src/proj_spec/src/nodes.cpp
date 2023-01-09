#include "nodes.hh"

void nodes::addStart(float x, float y, int R)
{
    node tmpNode = node(x, y);
    if (nodeColissionCheck(tmpNode))
    {
        tmpNode.nodeID = 0;
        N.push_back(tmpNode);
        connectNode(N.back(), R);
    }
}

void nodes::addEnd(float x, float y, int R)
{
    node tmpNode = node(x, y);
    if (nodeColissionCheck(tmpNode))
    {
        tmpNode.nodeID = 1;
        N.push_back(tmpNode);
        connectNode(N.back(), R);
    }
}

void nodes::generateNodes(int amount, float R)
{
    long long unsigned int amo = amount;
    srand(time(NULL));
    node tmpNode;
    int randomNumber;
    int i = 2;
    while (N.size() < amo)
    {
        float x = 0.0;
        float y = 0.0;

        randomNumber = rand();
        x = ((float)(randomNumber % ((int)W->maxGridX * 10)) / 10.0);
        randomNumber = rand();
        y = ((float)(randomNumber % ((int)W->maxGridY * 10)) / 10.0);

        tmpNode = node(x, y);
        if (nodeColissionCheck(tmpNode))
        {
            tmpNode.nodeID = i;
            i++;
            N.push_back(tmpNode);
            connectNode(N.back(), R);
        }
    }
}

int nodes::connectNode(node &connectedNode, float R)
{
    for (long long unsigned int i = 0; i < (N.size() - 1); i++)
    {
        float tmpdist = connectedNode.calcdistance(N[i]);
        if (tmpdist < R)
        {
            connectedNode.connections++;
            connectedNode.connectedNodes.push_back(make_pair(N[i].nodeID, tmpdist));
            N[i].connections++;
            N[i].connectedNodes.push_back(make_pair(connectedNode.nodeID, tmpdist));
            connections++;
            allEdges.push_back(make_pair(make_pair(N[i].nodeID, connectedNode.nodeID), tmpdist));
        }
    }

    return 0;
    // std::cout<<"Connected to " <<N[N.size()-1].connections << " nodes.\n";
}

void nodes::findDistance()
{
    const int V = N.size();
    vector<iPair> adj[V];
    vector<int> path;
    pair<float, float> tmpPair;
    for (long unsigned int i = 0; i < allEdges.size(); i++)
    {
        addEdge(adj,allEdges[i].first.first, allEdges[i].first.second, allEdges[i].second);
    }
    path=shortestPath(adj,V,0,1);
    for(long unsigned int i=0;i<path.size();i++)
    {
        tmpPair=make_pair(N[path[i]].pos[0],N[path[i]].pos[1]);
        finalPath.push_back(tmpPair);
    }
}

// void nodes::printToTable()
// {
//     // vector<int> tmp;
//     for(long unsigned int i=0; i<N.size();i++)
//     {
//         for(long unsigned int j=0; j<N.size();j++)
//         {
//             table[i][j]=0;
//             // tmp.push_back(0);
//         }
//         // table.push_back(tmp);
//         for(long unsigned int k=0; k<N[i].connectedNodes.size();k++)
//         {
//             table[i][N[i].connectedNodes[k]->nodeID]=(N[i].distanceToNode[k]*100);
//         }
//     }
// }

bool nodes::nodeColissionCheck(node checkedNode)
{
    for (long long unsigned int i = 0; i < W->obstacles.size(); i++)
    {
        if (!(W->obstacles[i]->checkCollision(checkedNode.pos[0], checkedNode.pos[1])))
            return false;
    }
    return true;
}




void addEdge(vector<pair<int, int>> adj[], int u, int v, int wt)
{
    adj[u].push_back(make_pair(v, wt));
    adj[v].push_back(make_pair(u, wt));
}

vector<int> shortestPath(vector<pair<int, int>> adj[], int V, int src, int target)
{
    priority_queue<iPair, vector<iPair>, greater<iPair>> pq;
    vector<int> dist(V, INF);
    vector<bool> visited(V, false);
    vector<int> prev(V, -1);
    pq.push(make_pair(0, src));
    dist[src] = 0;
    while (!pq.empty() && !visited[target])
    {
        int u = pq.top().second;
        pq.pop();
        if (visited[u])
        {
            continue;
        }
        visited[u] = true;
        for (auto x : adj[u])
        {
            int v = x.first;
            int weight = x.second;

            if (dist[v] > dist[u] + weight)
            {
                // relax
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
                prev[v] = u;
            }
        }
    }

    vector<int> res;
    res.push_back(target);
    int temp = target;
    while (temp != 0)
    {
        temp = prev[temp];
        res.push_back(temp);
    }
    // for (long unsigned int i = 0; i < res.size(); i++)
    // {
    //     cout << i << ": " << res[i] << endl;
    // }

    return res;

}
