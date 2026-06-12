# Map & Navigation Engine

A C++ based shortest-path routing engine with optimized search and caching.

## Features
*   **Dijkstra's Algorithm**: Finds the shortest path between nodes.
*   **LRU Cache**: Instant O(1) results for repeat queries.
*   **Trie Autocomplete**: Fast location name searching.
*   **Min-Heap Ranking**: Identifies main transit hubs.
*   **XML Support**: Reads map data from XML files.

## How to Run

1. **Compile**:
   ```bash
   clang++ dijsktra.cpp -o engine
   ```

2. **Run**:
   ```bash
   ./engine map.xml <source_id> <destination_id>
   ```

## Example
```bash
./engine map.xml 1 4
```
*   **First run**: Calculates path.
*   **Second run**: Returns result instantly from cache.
