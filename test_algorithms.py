import heapq
import math
import random
from collections import deque

edges = [
    ('A', 'B', 4), ('A', 'D', 2), ('B', 'C', 2), ('B', 'E', 6),
    ('C', 'F', 1), ('D', 'E', 5), ('D', 'G', 3), ('E', 'F', 7),
    ('F', 'H', 2), ('G', 'H', 4), ('H', 'I', 2), ('F', 'I', 10)
]
graph_dict = {}
for u, v, w in edges:
    graph_dict.setdefault(u, {})[v] = w
    graph_dict.setdefault(v, {})[u] = w

COORDS = {
    'A': (0, 0), 'B': (2, 1), 'D': (1, -1), 'C': (4, 1),
    'E': (3, -2), 'F': (5, -1), 'G': (2, -3), 'H': (4, -3), 'I': (6, -3),
}
HEURISTIC_SCALE = 0.4472135954999579  # verified admissible for this graph's weights

def heuristic(node, goal):
    (x1, y1), (x2, y2) = COORDS[node], COORDS[goal]
    return math.hypot(x2 - x1, y2 - y1) * HEURISTIC_SCALE


def dijkstra(graph, start, goal):
    pq = [(0, start, [])]
    visited = set()
    nodes_expanded = 0
    while pq:
        cost, node, path = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        nodes_expanded += 1
        path = path + [node]
        if node == goal:
            return cost, path, nodes_expanded
        for neighbor, weight in graph.get(node, {}).items():
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path))
    return float("inf"), [], nodes_expanded


def uniform_cost_search(graph, start, goal):
    # Historically the same algorithm as Dijkstra's, applied to search
    # problems rather than general shortest-path — kept as a separate
    # function since that's the standard framing in AI search courses,
    # but expect identical results to dijkstra() on this graph. That's
    # correct, not a bug.
    return dijkstra(graph, start, goal)


def astar(graph, start, goal):
    # Uses a real admissible/consistent heuristic (scaled straight-line
    # distance — see COORDS/HEURISTIC_SCALE above), not a hardcoded 0.
    # An admissible heuristic guarantees A* still finds the optimal path,
    # same as Dijkstra, typically while expanding fewer nodes.
    pq = [(heuristic(start, goal), 0, start, [])]  # (f_cost, g_cost, node, path)
    visited = set()
    nodes_expanded = 0
    while pq:
        f_cost, g_cost, node, path = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        nodes_expanded += 1
        path = path + [node]
        if node == goal:
            return g_cost, path, nodes_expanded
        for neighbor, weight in graph.get(node, {}).items():
            if neighbor not in visited:
                new_g = g_cost + weight
                new_f = new_g + heuristic(neighbor, goal)
                heapq.heappush(pq, (new_f, new_g, neighbor, path))
    return float("inf"), [], nodes_expanded


def bfs(graph, start, goal):
    # Real breadth-first search: FIFO queue, explores level by level by
    # hop count. Does NOT consider edge weight while choosing what to
    # explore next -- on a weighted graph this means the path found is
    # the one with the fewest edges, not necessarily the lowest total
    # weight. That's expected: BFS's shortest-path guarantee only holds
    # on unweighted graphs.
    queue = deque([(start, [start], 0)])
    visited = {start}
    nodes_expanded = 0
    while queue:
        node, path, cost = queue.popleft()
        nodes_expanded += 1
        if node == goal:
            return cost, path, nodes_expanded
        for neighbor, weight in graph.get(node, {}).items():
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor], cost + weight))
    return float("inf"), [], nodes_expanded


def dfs(graph, start, goal):
    # Real depth-first search: LIFO stack, explores as far as possible
    # down one branch before backtracking. Like BFS, gives no optimality
    # guarantee on a weighted graph -- it returns *a* path, not
    # necessarily a cheap one.
    stack = [(start, [start], 0)]
    visited = set()
    nodes_expanded = 0
    while stack:
        node, path, cost = stack.pop()
        if node in visited:
            continue
        visited.add(node)
        nodes_expanded += 1
        if node == goal:
            return cost, path, nodes_expanded
        for neighbor, weight in graph.get(node, {}).items():
            if neighbor not in visited:
                stack.append((neighbor, path + [neighbor], cost + weight))
    return float("inf"), [], nodes_expanded


def ant_colony_optimization(graph, start, goal,
                            num_ants=20, num_iterations=100,
                            decay=0.5, alpha=1, beta=2, Q=100):
    pheromones = {(u, v): 1.0 for u in graph for v in graph[u]}
    best_cost = float('inf')
    best_path = []

    for _ in range(num_iterations):
        iteration_paths = []
        for _ in range(num_ants):
            path = [start]
            visited = {start}
            cost = 0
            stuck = False
            while path[-1] != goal:
                current = path[-1]
                neighbors = [n for n in graph[current] if n not in visited]
                if not neighbors:
                    stuck = True
                    break
                scores = [
                    (pheromones.get((current, n), 1.0) ** alpha) * ((1.0 / graph[current][n]) ** beta)
                    for n in neighbors
                ]
                total = sum(scores)
                probs = [s / total for s in scores]
                next_node = random.choices(neighbors, weights=probs)[0]
                cost += graph[current][next_node]
                path.append(next_node)
                visited.add(next_node)
            if not stuck and path[-1] == goal:
                iteration_paths.append((cost, path))
                if cost < best_cost:
                    best_cost = cost
                    best_path = path[:]
        for key in pheromones:
            pheromones[key] *= (1 - decay)
        for cost, path in iteration_paths:
            for i in range(len(path) - 1):
                pheromones[(path[i], path[i+1])] += Q / cost
                pheromones[(path[i+1], path[i])] += Q / cost

    return best_cost, best_path


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------

start_node, goal_node = 'A', 'I'

results = {}
for name, fn in [('Dijkstra', dijkstra), ('UCS', uniform_cost_search),
                  ('A*', astar), ('BFS', bfs), ('DFS', dfs)]:
    cost, path, expanded = fn(graph_dict, start_node, goal_node)
    results[name] = (cost, path, expanded)
    print(f"{name:10s} cost={cost:6.1f}  hops={len(path)-1}  expanded={expanded:2d}  path={' -> '.join(path)}")

# ACO is stochastic — run with a fixed seed for the display, then verify over multiple seeds
random.seed(42)
aco_cost, aco_path = ant_colony_optimization(graph_dict, start_node, goal_node)
results['ACO'] = (aco_cost, aco_path, None)
print(f"{'ACO':10s} cost={aco_cost:6.1f}  hops={len(aco_path)-1}  expanded=N/A  path={' -> '.join(aco_path)}")

print()
print("=== Checks ===")

# 1. Dijkstra and UCS should be identical (same algorithm)
assert results['Dijkstra'][0] == results['UCS'][0], "Dijkstra and UCS should match"
print("PASS: Dijkstra and UCS produce identical cost (expected -- same algorithm)")

# 2. A* should find the SAME optimal cost as Dijkstra (admissible heuristic guarantee)
assert results['A*'][0] == results['Dijkstra'][0], "A* should still find optimal cost"
print("PASS: A* finds the same optimal cost as Dijkstra (heuristic is admissible)")

# 3. A* should expand <= nodes as Dijkstra (that's the whole point of a heuristic)
assert results['A*'][2] <= results['Dijkstra'][2], "A* should be at least as efficient"
print(f"PASS: A* expanded {results['A*'][2]} nodes vs Dijkstra's {results['Dijkstra'][2]} (heuristic guided search)")

# 4. BFS should find the path with fewest hops among all found paths
min_hops = min(len(r[1]) - 1 for r in results.values())
assert len(results['BFS'][1]) - 1 == min_hops, "BFS should find minimum-hop path"
print(f"PASS: BFS found the minimum-hop path ({min_hops} hops)")

# 5. BFS and DFS are no longer identical to each other or to Dijkstra/UCS
bfs_cost = results['BFS'][0]
dfs_cost = results['DFS'][0]
dijkstra_cost = results['Dijkstra'][0]
print(f"BFS cost={bfs_cost}, DFS cost={dfs_cost}, Dijkstra cost={dijkstra_cost}")
print(f"BFS path != DFS path: {results['BFS'][1] != results['DFS'][1]}")

# 6. Verify BFS/DFS are structurally different implementations (not the same function)
import inspect
bfs_uses_deque = 'deque' in inspect.getsource(bfs)
dfs_uses_stack_pop = '.pop()' in inspect.getsource(dfs) and 'popleft' not in inspect.getsource(dfs)
assert bfs_uses_deque, "BFS should use a FIFO deque"
assert dfs_uses_stack_pop, "DFS should use LIFO stack (.pop(), not .popleft())"
print("PASS: BFS uses FIFO queue (deque), DFS uses LIFO stack -- genuinely different traversal orders")

# 7. ACO: path must be valid (starts at start, ends at goal, all edges exist)
assert aco_path[0] == start_node and aco_path[-1] == goal_node, "ACO path must go from start to goal"
for i in range(len(aco_path) - 1):
    u, v = aco_path[i], aco_path[i+1]
    assert v in graph_dict.get(u, {}), f"ACO path contains non-existent edge {u}->{v}"
print(f"PASS: ACO returned a valid path from {start_node} to {goal_node}")

# 8. ACO: converges to optimal cost across multiple seeds (stochastic, but reliable on this small graph)
aco_costs = []
for seed in range(10):
    random.seed(seed)
    c, _ = ant_colony_optimization(graph_dict, start_node, goal_node)
    aco_costs.append(c)
optimal_cost = results['Dijkstra'][0]
assert all(c == optimal_cost for c in aco_costs), \
    f"ACO should converge to optimal cost {optimal_cost} across seeds, got: {aco_costs}"
print(f"PASS: ACO converges to optimal cost {optimal_cost} across 10 different random seeds")

print()
print("ALL CHECKS PASSED")
