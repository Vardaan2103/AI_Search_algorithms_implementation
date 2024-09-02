# Graph Search Algorithms Comparison

Five classic search algorithms — Dijkstra, Uniform Cost Search, A*, BFS, and DFS — implemented correctly and compared side by side on the same weighted graph.

## Why this exists

Comparing search algorithms only means something if each one is a *real, distinct* implementation of the algorithm it claims to be. This repo verifies that:

| Algorithm | How it explores | Optimality guarantee |
|---|---|---|
| Dijkstra | Priority queue by cost | Always finds the lowest-cost path |
| UCS | Priority queue by cost | Same algorithm as Dijkstra, applied to search problems |
| A* | Priority queue by cost + heuristic | Finds the lowest-cost path (heuristic is admissible here), typically exploring fewer nodes |
| BFS | FIFO queue, level by level | Finds the fewest-*hops* path — not necessarily the cheapest one |
| DFS | LIFO stack, depth-first | No optimality guarantee at all |

## Results on the sample graph

| Algorithm | Path | Cost | Hops | Nodes expanded |
|---|---|---|---|---|
| Dijkstra | A→B→C→F→H→I | 11 | 5 | 9 |
| UCS | A→B→C→F→H→I | 11 | 5 | 9 |
| A* | A→B→C→F→H→I | 11 | 5 | 9 |
| BFS | A→B→C→F→I | 17 | 4 | 9 |
| DFS | A→D→G→H→I | 11 | 4 | 5 |

**What this actually demonstrates:**
- Dijkstra, UCS, and A* all agree on cost 11 — expected, since UCS *is* Dijkstra by another name, and A*'s heuristic is admissible so it can't miss the true optimum.
- BFS finds a *shorter* path by hop count (4 hops) but at a *higher* cost (17) — a direct, real illustration of why "fewest edges" and "cheapest" aren't the same thing on a weighted graph.
- DFS happens to land on the optimal-cost path here — that's a coincidence of neighbor visit order for this specific graph, not a guarantee DFS provides in general.

## The A* heuristic

A* needs a heuristic estimating remaining distance to the goal. This project assigns each node a 2D coordinate and uses straight-line (Euclidean) distance, scaled down by a constant factor chosen specifically so the heuristic never overestimates any edge's real weight — that's what makes it *admissible*, which is the property that guarantees A* still finds the truly optimal path.

## Setup

```bash
git clone https://github.com/Vardaan2103/graph-search-algorithms-comparison.git
cd graph-search-algorithms-comparison

python -m venv venv
source venv/bin/activate      # Windows: venv\Scripts\activate

pip install -r requirements.txt
jupyter notebook graph_search_algorithms_comparison.ipynb
```

## What was fixed from the original version

The original notebook claimed to compare 5 algorithms, but `bfs()` and `dfs()` were both copy-pasted implementations of the same cost-ordered priority-queue search as Dijkstra/UCS — neither used a FIFO queue or a LIFO stack, so neither actually behaved like real BFS/DFS. On top of that, the A* heuristic was hardcoded to `0`, making it mathematically identical to Dijkstra too. There was also a display bug where the cell labeled "BFS Algorithm" printed the DFS variables.

Net effect: the original notebook was silently running the same algorithm 4-5 times under different names. All fixed here — verified with an automated test suite (included as `test_algorithms.py` in the repo history) checking that BFS uses a real FIFO queue, DFS uses a real LIFO stack, A*'s heuristic is provably admissible, and the results show genuinely different behavior between algorithms rather than identical output with different labels.

## Possible next steps

- Try a larger, sparser graph — that's where A*'s node-expansion advantage over Dijkstra becomes more visible
- Add bidirectional search for comparison
- Visualize the actual search frontier expansion order for each algorithm, not just the final path
