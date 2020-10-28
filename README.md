# A*-Heuristics-to-Solve-TSP

> The [travelling salesman problem (**TSP**)](https://en.wikipedia.org/wiki/Travelling_salesman_problem) asks the following question: "Given a list of cities and the distances between each pair of cities, what is the shortest possible route that visits each city exactly once and returns to the origin city?"

There are multiple ways to solve the TSP and this project only focuses on the A* algorithm implementing different heuristics. The A* algorithm will always return the optimal solution to the TSP, however by changing the heuristics the performance can be improved. This project implements and exploares *Uniform Cost(UCS), Random Remaining Edge(RRE)*, and *Cheapest Remaining Edge(CRE)* heuristics.

For this exploration, we will use a symmetric ajecency matrix as our TSP cities/nodes.

| Heuristic | Description |
| --- | ----- |
| UCS | Only considers the best option so far excluding how far it is from the goal |
| RRE | It adds the current cost to a randomly picked remaining edge |
| CRE | It adds the current cost to the cheapest remaining edge |
| MST | It adds the current cost to the cost of the minimum spanning tree on the remaining edges |

## Setup

1. clone repo and cd into the dir
2. install numpy from pip

```zsh
pip3 install numpy
```

3. run the command where func can be ucs, rre, cre, and mst

```zsh
python3 ExploaringHeuristic.py [func] input_s5.txt
```


## Next Step

- [ ] Add networkx to visualize the adjacency matrix / graph and the optimal solution

- [ ] Add bigger adjacency matrix to run and visualize the difference
