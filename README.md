# Map-Navigation-Engine-
Built a shortest-path routing engine in C++ over 1,000+ node road network, resolving optimal routes under 50ms using Dijkstra's and A* algorithms on weighted directed graph using priority queue.
Reduced repeated route-query time to O (1) by designing an LRU Cache from scratch using unordered map + doubly linked list, cutting redundant computation for duplicate source-destination pairs.
Developed location autocomplete in O(L) time across 500+ city names using a Trie with DFS traversal, and a min-heap for top K ranking; integrated Union-Find with path compression for O(α) connectivity checks before invoking the router , securely eliminating invalid route attempts entirely. 
Optimized bulk traffic-delay updates from O(n) to O(log n) by building a Segment Tree with lazy propagation, enabling real-time range-based edge weight change without full graph recomputation
