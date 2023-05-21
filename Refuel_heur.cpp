#include <iostream>
#include <vector>
#include <chrono>
#include <queue>
#include <string>
#include <tuple>
#include <climits>
#include <limits>
#include <bits/stdc++.h>


struct label
{
    int v, id;
    float f, g, q;

}; 

std::map<int,std::set<int>> compute_reachable_sets(std::vector<std::pair<int,std::pair<int,int>>> adj[], int n, int qmax){
    
    std::map<int,std::set<int>>mp;
    for(int i =  0 ; i  < n ; i++)
    {
        std::vector<int> d_star(n,INT_MAX);
        d_star[i] = 0 ;
        std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,std::greater<std::pair<int,int>>>pq;

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

std::vector<int> dijkstra(std::vector<std::pair<int,std::pair<int,int>>> adj[], int n, int s, int t) {

    std::map<int, std::set<float>> heur;
    std::vector<int> dist(n, std::numeric_limits<int>::max()); // initialize all energies to INF
    dist[s] = 0; // set the energy to the target node as 0
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;  //min heap
    pq.push({0, s});

    while (!pq.empty()) {
        int u = pq.top().second;    
        int d = pq.top().first;

        pq.pop();

        if (d > dist[u]){
            continue; // skip if we have already found a shorter path to this node
        }
        
        for (auto edge : adj[u]) {
            int v = edge.first;
            int w = edge.second.second; //gives the energy the robot has.
            
            if (dist[v] > dist[u] + w) { // if the current path is better than previous paths to v, update the distance and add it to the queue
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
                heur[v].insert(dist[v]); // insert the energy to the map
            }
        }
    }

    return dist;
}


bool CheckForPrune(label l, std::unordered_map<int, std::vector<label>>& frontier) {
    int v = l.v;

    for (const auto& l_prime : frontier[v]) {
        if (l_prime.g <= l.g && l_prime.q >= l.q) {
            return true;  // Skip if conditions are met
        }
    }
    return false;
}

void FilterAndAddFront(const label& l, std::unordered_map<int, std::vector<label>>& frontier) {
    int v = l.v;
    
    if (frontier.find(v) == frontier.end()) {
        frontier[v] = {l};  // Create a new frontier set for vertex v
        return;
    }
    
    int vertexSize = v + 1;  // The expected length of the frontier set at vertex v
    if (frontier[v].size() < vertexSize) {
        frontier[v].resize(vertexSize);  // Increase the length of the frontier set to vertexSize
    }
    
    for (const auto& l_prime : frontier[v]) {
        if (l_prime.g <= l.g && l_prime.q >= l.q) {
            return;  // Skip if conditions are met
        }
    }
    
    frontier[v].push_back(l);  // Add l to frontier set at vertex v
}

int Min_cost_graph(std::vector<std::pair<int,std::pair<int,int>>> adj[], int n)
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

std::vector<int> Cost_vector(std::vector<std::pair<int,std::pair<int,int>>> adj[], int n)
{
    std::vector<int> cost;
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


    std::vector<std::pair<int,std::pair<int,int>>> adj[n];
    label l;
    std::vector<label> lab;
    
    int qmax = 10;
    float dist[n];
    std::map<int,std::set<int>>mp;
        
          
    std::vector<int> heur_dist;
    //std::vector<label> frontier; 
    std::map<int, std::set<float>> computed_heur;
    std::unordered_map<int, std::vector<label>> frontier; //initialized to NULL
    std::vector<int> Cost_vec;
    
    //priority_queue<label, vector<label>, greater<label>> OPEN;  //--> couldn't figure out how to implement this.
    
    std::set<std::pair<float, int>> OPEN;
    
    adj[0] = {std::make_pair(1,  std::make_pair(1, 5)),std::make_pair(2, std::make_pair(1, 2))};
    adj[1] = {std::make_pair(2, std::make_pair(3, 6)),std::make_pair(3, std::make_pair(3, 2))};
    adj[2] = {std::make_pair(3,std::make_pair(4,7))};
    
    int s = 0; // source node
    int t = 3; // target node
    
    
    mp = compute_reachable_sets(adj, n, qmax);

    // heur_dist = dijkstra(adj, n, s, t);
    int min_cost = Min_cost_graph(adj, n);
    Cost_vec = Cost_vector(adj,n);

    // dvg will be the last element of the dijkstra vector--> basically the energy consumed for the last node.

    for (int i = 0; i<n; i++)
    {   
        std::vector<int> dijkstra_back = dijkstra(adj, n, i, t);
        heur_dist.push_back(dijkstra_back.back());
        float dvg;
        dvg = heur_dist[i];
        float heur = std::max((dvg - Cost_vec[i])*min_cost , 0.0f);
        computed_heur[i].insert(heur);
    }


/*
    THE CODE FOR REFUEL A*
*/
    lab.push_back({0, 0, 0});
    OPEN.insert(std::make_pair(lab[0].f, 0));

    label target_label;
    target_label.v = t;
    target_label.id = 0;
    target_label.f = std::numeric_limits<float>::infinity();
    target_label.g = std::numeric_limits<float>::infinity();
    target_label.q = std::numeric_limits<float>::infinity();
    
    // starting the A* search:
    while(!OPEN.empty())
    {
        auto it = OPEN.begin();
        int v = it->second;
        OPEN.erase(it);

        if (v == t && target_label.q <= qmax)
        {
            break;
        }

        const std::vector<label>& labels= frontier[v];

        for (const auto &l_prime : labels)
        {
            if (l_prime.g <= l.g && l_prime.q >= l.q)
            {
                // Skip if conditions are met
                continue;
            }
            if (l_prime.q <= qmax && !CheckForPrune(l_prime, frontier))
            {
                // Add the current label to the result
                FilterAndAddFront(l_prime, frontier);

                // Generate successor labels
                for (auto it : adj[v])
                {
                    int u = it.first;
                    int w = it.second.second; // energy cost on road
                    int e = it.second.first;  // cost of refueling

                    // Compute the new values for the successor label
                    label l_succ;
                    l_succ.v = u;
                    l_succ.id = l_prime.id + 1;
                    l_succ.g = l_prime.g + w;
                    l_succ.q = l_prime.q + e;
                    l_succ.f = l_succ.g + *computed_heur[u].begin(); 

                    // Add the successor label to the OPEN set
                    OPEN.insert(std::make_pair(l_succ.f, l_succ.v));
                    FilterAndAddFront(l_succ, frontier);
                }
            }
        }
    }

    // Find the optimal path
    std::vector<int> optimal_path;
    label curr_label = target_label;
    while (curr_label.v != s)
    {
        optimal_path.push_back(curr_label.v);
        int v = curr_label.v;
        const std::vector<label> &labels = frontier[v];
        for (const auto &l : labels)
        {
            if (l.id == curr_label.id - 1 && l.g == curr_label.g - Cost_vec[v] && l.q == curr_label.q - 1)
            {
                curr_label = l;
                break;
            }
        }
    }
    optimal_path.push_back(s);
    std::reverse(optimal_path.begin(), optimal_path.end());

    // Print the optimal path
    std::cout << "Optimal Path: ";
    for (int i : optimal_path)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    return 0;

}


    //----------_TESTING------------------------------------------------------------------------

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
    
    for(auto it:mp)
    {
        std::cout<<it.first<<"->";
        for(auto i:it.second)
        {
            std::cout<<i<<" ";
        }
        std::cout<<std::endl;
    }
    return 0;    
        
*/
