#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <queue>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <climits>
#include "rapidxml.hpp"

using namespace std;
using namespace rapidxml;

// Structure to store route results
struct RouteResult {
    int distance;
    vector<int> path;
};

// --- LRU CACHE IMPLEMENTATION ---
class LRUCache {
    int capacity;
    list<pair<pair<int, int>, RouteResult>> dq; // stores {{src, des}, result}
    unordered_map<string, list<pair<pair<int, int>, RouteResult>>::iterator> ma;

public:
    LRUCache(int cap) : capacity(cap) {}

    RouteResult* get(int src, int des) {
        string key = to_string(src) + "_" + to_string(des);
        if (ma.find(key) == ma.end()) return nullptr;
        
        // Move to front (Most Recently Used)
        dq.splice(dq.begin(), dq, ma[key]);
        return &ma[key]->second;
    }

    void put(int src, int des, RouteResult res) {
        string key = to_string(src) + "_" + to_string(des);
        if (ma.find(key) != ma.end()) {
            dq.erase(ma[key]);
        } else if (dq.size() == capacity) {
            string lastKey = to_string(dq.back().first.first) + "_" + to_string(dq.back().first.second);
            ma.erase(lastKey);
            dq.pop_back();
        }
        dq.push_front({{src, des}, res});
        ma[key] = dq.begin();
    }
};

// --- TRIE FOR AUTOCOMPLETE ---
class TrieNode {
public:
    unordered_map<char, TrieNode*> children;
    int nodeId; // Stores node ID if it's a valid end of a name
    TrieNode() : nodeId(-1) {}
};

class Trie {
    TrieNode* root;
public:
    Trie() { root = new TrieNode(); }

    void insert(string name, int id) {
        TrieNode* curr = root;
        for (char ch : name) {
            if (curr->children.find(ch) == curr->children.end())
                curr->children[ch] = new TrieNode();
            curr = curr->children[ch];
        }
        curr->nodeId = id;
    }

    int search(string name) {
        TrieNode* curr = root;
        for (char ch : name) {
            if (curr->children.find(ch) == curr->children.end()) return -1;
            curr = curr->children[ch];
        }
        return curr->nodeId;
    }
};

// --- GLOBAL ENGINE STATE ---
LRUCache cache(10); // Cache last 10 queries
Trie locationTrie;
unordered_map<int, string> idToName;

void dijkstra(int src, vector<vector<pair<int,int>>> &adj, vector<int> &dist, vector<int> &path){
    int n = adj.size();
    dist.assign(n, INT_MAX);
    path.assign(n, -1);
    
    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    dist[src] = 0;
    pq.push({0, src});
    while(!pq.empty()){
        auto i = pq.top();
        pq.pop();
        int d = i.first;
        int u = i.second;
        if(d > dist[u]) continue;
        for(auto p : adj[u]){
            int v = p.first;
            int w = p.second;
            if(dist[v] > d + w){
                dist[v] = d + w;
                path[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
}

bool readNodes(string filename, vector<vector<pair<int,int>>> &adj) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return false;
    }
    int count;
    if (!(file >> count)) {
        cerr << "Error reading node count from file." << endl;
        return false;
    }
    adj.assign(count + 1, vector<pair<int,int>>());
    int x, y, w;
    while(file >> x >> y >> w){
        if(w < 0) continue;
        if(x <= 0 || y <= 0 || x > count || y > count || x == y) continue;
        adj[x].push_back({y, w});
    }
    file.close();
    return true;
}

bool readXML(string filename, vector<vector<pair<int,int>>> &adj) {
    ifstream file(filename);
    if (!file.is_open()) return false;
    vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    buffer.push_back('\0');
    file.close();
    xml_document<> doc;
    try {
        doc.parse<0>(&buffer[0]);
    } catch (parse_error &e) {
        cerr << "XML Parse Error: " << e.what() << endl;
        return false;
    }
    xml_node<> *root = doc.first_node("osm");
    if (!root) return false;
    
    int maxId = 0;
    for (xml_node<> *node = root->first_node("node"); node; node = node->next_sibling("node")) {
        int id = stoi(node->first_attribute("id")->value());
        maxId = max(maxId, id);
        for (xml_node<> *tag = node->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
            if (string(tag->first_attribute("k")->value()) == "name") {
                string name = tag->first_attribute("v")->value();
                locationTrie.insert(name, id);
                idToName[id] = name;
            }
        }
    }
    adj.assign(maxId + 1, vector<pair<int,int>>());

    for (xml_node<> *way = root->first_node("way"); way; way = way->next_sibling("way")) {
        vector<int> nodesInWay;
        for (xml_node<> *nd = way->first_node("nd"); nd; nd = nd->next_sibling("nd"))
            nodesInWay.push_back(stoi(nd->first_attribute("ref")->value()));

        bool isOneWay = false;
        int weight = 1;
        for (xml_node<> *tag = way->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
            string k = tag->first_attribute("k")->value();
            string v = tag->first_attribute("v")->value();
            if (k == "oneway" && v == "yes") isOneWay = true;
            if (k == "weight") weight = stoi(v);
        }

        for (size_t i = 0; i + 1 < nodesInWay.size(); ++i) {
            adj[nodesInWay[i]].push_back({nodesInWay[i+1], weight});
            if (!isOneWay) adj[nodesInWay[i+1]].push_back({nodesInWay[i], weight});
        }
    }
    return true;
}

// --- TOP K CONNECTIVITY CHECK ---
struct NodeConnectivity {
    int nodeId;
    int degree;
    bool operator>(const NodeConnectivity& other) const {
        return degree > other.degree;
    }
};

vector<int> getTopKHubs(const vector<vector<pair<int,int>>>& adj, int k) {
    priority_queue<NodeConnectivity, vector<NodeConnectivity>, greater<NodeConnectivity>> minHeap;
    
    for (int i = 1; i < adj.size(); ++i) {
        int degree = adj[i].size();
        if (minHeap.size() < k) {
            minHeap.push({i, degree});
        } else if (degree > minHeap.top().degree) {
            minHeap.pop();
            minHeap.push({i, degree});
        }
    }
    
    vector<int> hubs;
    while (!minHeap.empty()) {
        hubs.push_back(minHeap.top().nodeId);
        minHeap.pop();
    }
    return hubs;
}

void solve(string filename, int srcId, int desId) {
    static vector<vector<pair<int,int>>> adj;
    if (adj.empty()) {
        if (filename.find(".xml") != string::npos) readXML(filename, adj);
        else readNodes(filename, adj);
        
        // Connectivity Check (Top K Ranking)
        cout << "[System Info] Identifying top 3 transit hubs..." << endl;
        vector<int> hubs = getTopKHubs(adj, 3);
        for (int hubId : hubs) {
            cout << " - Hub Node " << hubId << " (Connectivity: " << adj[hubId].size() << ")" << endl;
        }
    }

    if (srcId >= adj.size() || desId >= adj.size() || srcId < 1 || desId < 1) {
        cerr << "Error! Node ID out of range. Route attempt eliminated." << endl;
        return;
    }
    
    // Quick connectivity check: Is the node even in the graph?
    if (adj[srcId].empty() && adj[desId].empty()) {
        cerr << "Error! Source and Destination are isolated. Route attempt eliminated." << endl;
        return;
    }

    // Check LRU Cache
    RouteResult* cached = cache.get(srcId, desId);
    if (cached) {
        cout << "[Cache Hit] Returning stored route." << endl;
        cout << "Distance: " << cached->distance << endl;
        cout << "Path: ";
        for (int i = 0; i < cached->path.size(); i++) 
            cout << cached->path[i] << (i == cached->path.size() - 1 ? "" : " -> ");
        cout << endl;
        return;
    }

    vector<int> dist, path;
    dijkstra(srcId, adj, dist, path);

    if (dist[desId] == INT_MAX) {
        cout << "No path found." << endl;
    } else {
        RouteResult res;
        res.distance = dist[desId];
        int curr = desId;
        while (curr != -1) {
            res.path.push_back(curr);
            curr = path[curr];
        }
        reverse(res.path.begin(), res.path.end());
        
        cache.put(srcId, desId, res);
        cout << "[Cache Miss] Path calculated and stored." << endl;
        cout << "Distance: " << res.distance << endl;
        cout << "Path: ";
        for (int i = 0; i < res.path.size(); i++) 
            cout << res.path[i] << (i == res.path.size() - 1 ? "" : " -> ");
        cout << endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <filename> <source_node> <destination_node>" << endl;
        return 1;
    }
    
    string filename = argv[1];
    int src = stoi(argv[2]);
    int des = stoi(argv[3]);

    cout << "--- First Query ---" << endl;
    solve(filename, src, des);
    
    // Demonstrate LRU Cache
    cout << "\n--- Repeating same query (LRU Cache Demonstration) ---" << endl;
    solve(filename, src, des);
    
    return 0;
}
