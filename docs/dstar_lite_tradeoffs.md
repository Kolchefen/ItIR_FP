# Why D* Lite is not cost-effective for this TSP

This package uses D* Lite to compute the pairwise shortest-path costs that feed
the 4-waypoint TSP in `tsp_executor.py`. This document explains why that
particular combination is not the algorithm's sweet spot — it works correctly,
but it does more work than the problem demands, and a simpler algorithm would
give identical answers.

## What D* Lite is actually good at

D* Lite (Koenig & Likhachev, 2002) is an **incremental** replanner. Its value
comes from a specific setting:

- You have a **single** goal and a single current start.
- The **grid is known to change** while the robot is moving (new obstacles
  detected, edge costs revised).
- After each change, you need a fresh shortest path **without re-expanding the
  whole graph** — D* Lite reuses its previous g/rhs values and only revisits
  cells whose costs are actually affected.

Under those conditions, D* Lite is typically one or two orders of magnitude
cheaper than re-running Dijkstra or A* from scratch.

## What our TSP actually asks for

The tour planner needs an **all-pairs cost matrix** over N = 5 vertices (robot
start + four waypoints) on a **static** inflated occupancy grid. For that
matrix it runs `DStarLite.compute_full()` once per goal and reads the resulting
g-values at the other N−1 vertices.

Two things about that workload break D* Lite's advantages:

1. **Static grid.** No edge costs change between runs, so D* Lite's
   incremental machinery — `k_m`, rhs bookkeeping, vertex re-keying on change
   — never fires. We pay for the complexity but never use it.

2. **Full-field queries.** We do not want a path to one start; we want g(s)
   for every other waypoint. `compute_full()` drains the entire queue,
   which turns the search into a **Dijkstra-equivalent expansion order**. The
   heuristic that makes focused D* Lite fast becomes dead weight in this mode,
   because we refuse to stop early. We end up paying A*'s priority-queue
   overhead to produce Dijkstra's output.

## What would be cheaper and produce the same result

For a static grid and an all-pairs cost matrix between N sources:

| Algorithm                 | Expansions per source | Extra machinery |
|---------------------------|-----------------------|-----------------|
| **Dijkstra (Nx)**         | O(V log V) per source | None            |
| **A\* (Nx, one per pair)**| Focused on each pair  | Heuristic       |
| **D\* Lite via compute_full** | O(V log V) per goal | Full rhs/key machinery, unused |
| Floyd–Warshall (dense)    | O(V³) total           | Overkill for sparse grids |

For our N = 5, V ≈ (map cells after inflation ~ few × 10⁵), N runs of Dijkstra
would give the same cost matrix with strictly less per-call overhead than
D* Lite. A\* between each of the 20 ordered pairs would be even cheaper
because each query can terminate as soon as the goal is popped.

## Why D* Lite is still in the package

Three reasons, in decreasing order of honesty:

1. **The assignment is about D\* Lite.** Implementing it on a real TurtleBot
   demonstrates the algorithm even if the *particular* experiment around it
   doesn't stretch it.
2. **It is a strict generalization.** Every run of D* Lite in this repo could
   be replaced by Dijkstra and the tour would be identical. Keeping the
   D* Lite class open-ended (it already supports focused search and
   incremental update via `k_m`) means future experiments — dynamic obstacles,
   discovered-map exploration, cost inflation during execution — can reuse it
   without re-implementing.
3. **Future work.** If the executor is extended to replan mid-leg when the
   local costmap reports a new obstacle, `compute_shortest_path()` becomes
   genuinely useful and the incremental updates would pay off. The scaffolding
   is there for that day.

## TL;DR

- For **this static 4-waypoint TSP**, N runs of Dijkstra (or A\* per pair)
  would produce the identical tour with less constant-factor overhead.
- D* Lite's win case — **incremental replanning on a changing grid** — is not
  exercised by a one-shot TSP over a known map.
- The implementation is kept because the algorithm itself is the subject of
  the project, and the class is ready to demonstrate its strengths once a
  dynamic-obstacle scenario is layered on top.
