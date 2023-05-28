#include <iostream>
#include <vector>
#include <chrono>
#include <queue>
#include <string>
#include <tuple>
#include <climits>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <algorithm>

/*
    --------COMMENTS---------------------------
    The graph is an adjacency list containing:
    <"successor", <"cost of refuelling", "energy cost on road">>

    label will be a *struct*: will have a vertex id, g and q values as well as its own ID.
        vector-> to map the label with the struct.

*/

struct label
{
    int v, id;
    float f, g, q;

}; 

/*  
    ComputeReachable sets:
    prints for qmax of 10:
        0->0 1 2 3 
        1->1 2 3 
        2->2 3 
        3->3 
*/
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

/*
    Standard Dijkstra algorithm.
*/

std::vector<int> dijkstra(std::vector<std::pair<int,std::pair<int,int>>> adj[], int n, int s, int t) {

    //std::map<int, std::set<float>> heur;
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
                //heur[v].insert(dist[v]); // insert the energy to the map
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
    
    frontier[v].push_back(l);  // Add l to frontier set at vertex v
}


/*
    Finds the minimum cost in the entire graph.
*/
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

/*
    Gives a vector: vec[vertex] = cost of refuelling at that vertex
*/
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

/*
    Gives a vector: vec[vertex] = energy of travelling through that vertex.
*/
std::vector<int> Energy_vector(std::vector<std::pair<int,std::pair<int,int>>> adj[], int n)
{
    std::vector<int> energy;
    for(int i=0;i<n;i++){
    for(int j=0;j<adj[i].size();j++){
        int u=i;
        int v=adj[i][j].first;
        int w=adj[i][j].second.first;
        int wt=adj[i][j].second.second;
        energy.push_back(wt);
        
    }
    }
    return energy;
}

/*
    Gets all the neighbors of a particular vertex.
*/
std::vector<int> getNeighbors(std::vector<std::pair<int,std::pair<int,int>>> adj[], int index, int n)
{
    std::vector<int> neigh;

    for(int i = 0; i<n; i++)
    {
        for (int j=0 ; j<adj[i].size(); j++)
        {
            if (index==i){
                neigh.push_back(adj[i][j].first);
            }
        }
    }
    return neigh;
}

/*
    gives the integer distance it needs to travel from vertex1 to vertex2 (part of the graph)
*/
int getdist(const std::vector<std::pair<int, std::pair<int, int>>> adj[], int elem1, int elem2, int n) {
    
    int dist;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<adj[i].size(); j++)
        {
            if (i == elem1 && adj[i][j].first == elem2)
            {
                dist = adj[i][j].second.second;
            }
        }
    }
    return dist;

}

/*
    Function to give the heuristic of a particular vertex computed inside  "COMPUTED_HEUR"
*/
float getFloatHeur(const std::map<int, std::set<float>>& computed_heur, int key) {
    auto it = computed_heur.find(key);
    if (it != computed_heur.end() && !it->second.empty()) {
        return *(it->second.begin());
    } else {
        return 0.0f;
    }
}

int main()
{
    int n = 4;
    int qmax = 10;
    int s = 0; // source node
    int t = 3; // target node

    //-------INITIALIZATIONS-------------------
    std::vector<std::pair<int,std::pair<int,int>>> adj[n];
    std::vector<label> lab;
    std::map<int,std::set<int>>mp;
    std::vector<int> heur_dist; 
    std::map<int, std::set<float>> computed_heur;
    std::unordered_map<int, std::vector<label>> frontier; //initialized to NULL
    std::vector<int> Cost_vec;
    std::vector<int> Ener_vec;
    std::set<std::pair<float, int>> OPEN;
    std::vector<int> Neighbours;
    
/*
    GRAPH:
*/
    adj[0] = {std::make_pair(1, std::make_pair(1, 5)),std::make_pair(2, std::make_pair(1, 2))};
    adj[1] = {std::make_pair(2, std::make_pair(3, 6)),std::make_pair(3, std::make_pair(3, 2))};
    adj[2] = {std::make_pair(3, std::make_pair(4,7))};
    
/*
    REACHABLE SETS
*/
    mp = compute_reachable_sets(adj, n, qmax);
    int min_cost = Min_cost_graph(adj, n);
    Cost_vec = Cost_vector(adj,n);

/*
    BACKWARDS DIJKSTRA CODE to generate the heuristics:
*/
    for (int i = 0; i<n; i++)
    {   
        std::vector<int> dijkstra_back = dijkstra(adj, n, i, t);
        heur_dist.push_back(dijkstra_back.back());
        float dvg;
        dvg = heur_dist[i]; // dvg will be the last element of the dijkstra vector--> basically the energy consumed for the last node.
        float heur = std::max((dvg - Cost_vec[i])*min_cost , 0.0f);
        //std::cout<<heur<<std::endl;
        computed_heur[i].insert(heur);
    }
/*
    THE CODE FOR REFUEL A*
*/
    lab.push_back({0, 0, 0.0F, 0.0F, 0.0F}); 
    OPEN.insert(std::make_pair(lab[0].f, 0)); //(f, index)

    label target_label;
    target_label.v = t;
    target_label.id = 0;

    // starting the A* search:
    while(!OPEN.empty())
    {
        auto it = OPEN.begin();
        auto l = lab[it->second]; //giving the ID
        OPEN.erase(it);
        Neighbours = getNeighbors(adj, l.v, n);

        if (CheckForPrune(l, frontier))
        {
            continue;
        }
        FilterAndAddFront(l,frontier);

        if(l.v==t)
        {
            break;
        }

        const std::vector<label>& labels= frontier[l.v]; //retrieves the labels associated with vertex id v from the frontier.
        
        for(auto v_prime: Neighbours)
        {
            int distance = getdist(adj, l.v, v_prime, n);
            label l_prime;
            l_prime.id++;
            //std::cout<<distance<<std::endl;
            l_prime.v = v_prime;
            if(Cost_vec[v_prime] > Cost_vec[l.v])
            {
                l_prime.g = l.g + (qmax - l.q)*Cost_vec[l.v];
                l_prime.q = qmax - distance;
                // std::cout<<distance<<std::endl;
                //std::cout<<"The q value for neighbour "<<v_prime<<" of vertex "<< l.v <<" is "<<l_prime.q<<std::endl;
            }
            else
            {
                if (distance > l.q)
                {
                    l_prime.g = l.g + (distance - l.q)*Cost_vec[l.v];
                    l_prime.q = 0;
                    //std::cout<<"The q value for neighbour "<<v_prime<<" of vertex "<< l.v <<" is "<<l_prime.q<<std::endl;
                }
                else
                {
                    l_prime.g = l.g;
                    l_prime.q = l.q - distance;
                    //std::cout<<"The q value for neighbour "<<v_prime<<" of vertex "<< l.v <<" is "<<l_prime.q<<std::endl;
                }   
            }
            
            l_prime.f = l_prime.g + getFloatHeur(computed_heur, v_prime);
            lab.push_back({l_prime.v, l_prime.id, l_prime.f, l_prime.g, l_prime.q});
            //std::cout<<l_prime.f<<std::endl;
            if (CheckForPrune(l_prime, frontier))
            {
                continue;
            }
            OPEN.insert(std::make_pair(l_prime.f, l_prime.id)); //--> SOME ERROR HERE?
        }
        
    }

    //Find the optimal path
    // std::vector<int> optimalPath;
    // label currentLabel = lab[target_label.id];
    // optimalPath.push_back(currentLabel.v);

    // while (currentLabel.v != s) {
    //     for (const auto& l : frontier[currentLabel.v]) {
    //         if (l.g == currentLabel.g - getdist(adj, currentLabel.v, l.v, n)) {
    //             currentLabel = l;
    //             optimalPath.push_back(currentLabel.v);
    //             break;
    //         }
    //     }
    // }

    // //std::reverse(optimalPath.begin(), optimalPath.end());

    // std::cout << "Optimal Path: ";
    // for (const auto& vertex : optimalPath) {
    //     std::cout << vertex << " ";
    // }    
    // std::cout << std::endl;

    for(auto itr: lab)
    {
        std::cout<<itr.v<<std::endl;
    }

    return 0;

}
