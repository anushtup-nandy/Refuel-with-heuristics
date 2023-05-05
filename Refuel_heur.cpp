#include <iostream>
#include <vector>
#include <chrono>
#include <queue>
#include <string>
#include <tuple>
#include <bits/stdc++.h>

/*
    ERRORS in:
    1. Dijkstra Code (gives [inf,inf,...])
    2. OPEN list implementation. (couldn't understand how to implement priority queue on the labels)
*/
using namespace std;

struct label
{
    int v;
    float g, q;

}; 


map<int,set<int>> compute_reachable_sets(vector<pair<int,pair<int,int>>> adj[], int n, int qmax){
    
    map<int,set<int>>mp;
    for(int i =  0 ; i  < n ; i++)
    {
        vector<int> d_star(n,INT_MAX);
        d_star[i] = 0 ;
        priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>>pq;

        pq.push({0,i});
        while(pq.size()>0)
        {
            int currDist = pq.top().first;
            int curr = pq.top().second;
            
            pq.pop();
            if(currDist>qmax)
            {
                continue;
            }
            else
            mp[i].insert(curr);
            
            for(auto it:adj[curr])
            {
                  
                  if(mp[i].find(it.first) != mp[i].end())
                  {
                      continue;
                  }
                  if(d_star[it.first]> d_star[curr]+it.second.second)
                  {
                      d_star[it.first] = d_star[curr]+it.second.second;
                      pq.push({d_star[it.first],it.first});
                  }
            }
        }
        
        
    }

    return mp;
}

map<int, set<float>> dijkstra(vector<pair<int,pair<int,int>>> adj[], int n, int s, int t) {

    map<int, set<float>> heur;
    vector<int> dist(n, INT_MAX); // initialize all distances to INF
    dist[t] = 0; // set the distance to the target node as 0
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;  //min heap
    pq.push({0, t});

    while (!pq.empty()) {
        int u = pq.top().first;
        int d = pq.top().second;
        pq.pop();
        if (d > dist[u]){
            continue; // skip if we have already found a shorter path to this node
        }
        for (auto edge : adj[u]) {
            int v = edge.first;
            int w = edge.second.second; //gives the energy the robot has.
           // cout<< w<<endl;
            if (dist[v] > dist[u] + w) { // if the current path is better than previous paths to v, update the distance and add it to the queue
                dist[v] = dist[u] + w;
                pq.push({v, dist[v]});
            }
        }
    }

    return heur; // return the heuristics.
}

bool CheckForPrune(label l, vector<label>& frontier) {
    for (const auto& l_prime : frontier) {
        if (l_prime.g <= l.g && l_prime.q >= l.q) {
            return true;
        }
    }
    return false;
}

vector<label> FilterAndAddFront(label l, vector<label>& frontier){
    
    vector<label> new_frontier;
    
    for (const auto& l_prime : frontier) {
        if (l_prime.g <= l.g && l_prime.q >= l.q) {
            continue;
        }
    }
    new_frontier.push_back(l);
    frontier = new_frontier;
    return frontier;
}

int Min_cost_graph(vector<pair<int,pair<int,int>>> adj[], int n)
{
    int min_cost = INT_MAX;
    for(int i = 0; i < n; i++) {
        for(auto& p : adj[i]) 
        {
            if(p.second.first < min_cost) 
            {
                min_cost = p.second.first;
            }
        }
    }
    return min_cost;
}

vector<int> Cost_vector(vector<pair<int,pair<int,int>>> adj[], int n)
{
    vector<int> cost;
    for(int i=0;i<n;i++){
    for(int j=0;j<adj[i].size();j++){
        int u=i;
        int v=adj[i][j].first;
        int w=adj[i][j].second.first;
        int wt=adj[i][j].second.second;
        cost.push_back(w);
        
    }
    }
    return cost;
}

int main()
{
    int n = 4;

/*
    The graph is an adjacency list containing:
    <"successor", <"cost of refuelling", "energy cost on road">>

    label will be a *struct*: will have a vertex id, g and q values as well as its own ID.
        vector-> to map the label with the struct.
*/
    vector<pair<int,pair<int,int>>> adj[n];
    label l;
    vector<label> lab;
    int qmax = 10;
    float dist[n];
    map<int,set<int>>mp;
    map<int,set<float>> heur;
    vector<label> frontier = {}; //initialized to NULL
    map<int, set<float>> computed_heur;
    vector<int> Cost_vec;
    
    priority_queue<label, vector<label>, greater<label>> OPEN;  //--> couldn't figure out how to implement this.
    
    adj[0] = {make_pair(1,  make_pair(1, 5)),make_pair(2, make_pair(1, 2))};
    adj[1] = {make_pair(2, make_pair(3, 6)),make_pair(3, make_pair(3, 2))};
    adj[2] = {make_pair(3,make_pair(4,7))};
    
    int s = 0; // source node
    int t = 3; // target node
    
    
    mp = compute_reachable_sets(adj, n, qmax);

    heur = dijkstra(adj, n, s, t);
    int min_cost = Min_cost_graph(adj, n);
    Cost_vec = Cost_vector(adj,n);

    for (int i = 0; i<3; i++)
    {   
        
        float dvg = heur(adj, n, i, t);  //ERROR--> WHY?
        float heur = max((dvg - Cost_vec[i])*min_cost , 0);
        computed_heur[i].insert(heur);
    }



/*
    THE CODE FOR REFUEL A*
*/
    lab.push_back({0, 0, 0});
    OPEN.push(lab[0]);
    
    while(!OPEN.empty())
    {
        int node = OPEN.top().v;
        int cost = OPEN.top().g;
        int ener = OPEN.top().q;
        l = OPEN.top();
        OPEN.pop();

        if (CheckForPrune(l, frontier)){
            continue;
        }

        FilterAndAddFront(l, frontier);

        if (node == t)
        {
            break;
        }

        for (auto it:mp) //mp is the reachable sets
        {
            int cv_prime;
            int node_prime;
            for {auto j: it.second}
            {

            }
        }
        

    }



    

    //----------_TESTING------------------------------------------------------------------------

/*
    TESTING BACKWARDS DIJKSTRA:-----> GIVING [INF, INF....]
*/ 

    // vector<int> dist(n, INT_MAX);
    // dijkstra(3, adj, dist,n);

    // // print shortest distances from target to all other nodes
    // for (int i = 0; i < n; i++) {
    //     cout << "Shortest distance from node " << i << " to target " << 3 << " is " << dist[i] << endl;
    // }

/*
    TESTING THE GRAPH:--->CORRECT OUTPUT.
*/ 

    // for(int i=0;i<n;i++){
    // for(int j=0;j<adj[i].size();j++){
    //     int u=i;
    //     int v=adj[i][j].first;
    //     int w=adj[i][j].second.first;
    //     int wt=adj[i][j].second.second;
    //     cout<<"Edge from "<<u<<" to "<<v<<" with weight "<<wt<<" and cost "<< w <<endl;
    // }
    // }

/*
    TESTING THE REACHABLE SETS:---> CORRECT OUTPUT.
    prints for qmax of 10:
        0->0 1 2 3 
        1->1 2 3 
        2->2 3 
        3->3 
        
*/
    for(auto it:mp)
    {
        cout<<it.first<<"->";
        for(auto i:it.second)
        {
            cout<<i<<" ";
        }
        cout<<endl;
    }
    return 0;
}

