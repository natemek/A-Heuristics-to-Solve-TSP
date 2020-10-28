from util import *
import heapq
import sys
import time
import random
import numpy as np

def main(argv):
    if argv:
        filename = argv[1]
        func = argv[0]
    else:  # default filename and func
        filename = 'input_s5.txt'
        func = 'ucs'

    matrix = get_matrix(filename)
    if matrix:
        start = 0
        # start = random.randint(0, len(matrix) - 1) 
        initial_time = time.time()
        initial_cpu = time.process_time()
        if func == 'ucs':
            print(A_uniformCost(matrix, start), time.time() - initial_time, time.process_time() - initial_cpu)
        elif func == 'rre':
            print(A_randomEdge(matrix, start), time.time() - initial_time, time.process_time() - initial_cpu)
        elif func == 'cre':
            print(A_cheapestEdge(matrix, start), time.time() - initial_time, time.process_time() - initial_cpu)
        elif func == 'mst':
            print(A_MST(matrix, start), time.time() - initial_time, time.process_time() - initial_cpu)
        

def A_uniformCost(matrix, start):
    """ Implements Uniform Cost Search on an adjesency matrix to solve TSP

      :param matrix: adjesency matrix
      :param start:  start node

      :return: returns (cost to goal, path, num of nodes visited)
    """
    num_of_nodes_expanded = 1
    queue = []  # Frontier

    """ The queue structure holds (cost, path) where

    :cost: cost of path from start to current node
    :path: path from start to current
  """
    heapq.heappush(queue, (0, [start]))

    while queue:
        prev_cost, path = heapq.heappop(queue)

        # goal state
        if len(path) >= len(matrix) + 1 and path[0] == path[-1]:
            return (prev_cost, path, num_of_nodes_expanded)

        # expand curr node
        curr_node = path[-1]
        for neighbour_node, neighbour_cost in enumerate(matrix[curr_node]):
            # if visited all nodes and attempting to go back to the initial
            if len(path) == len(matrix):
                total_cost = prev_cost + matrix[curr_node][start]
                final_path = path + [start]
                heapq.heappush(queue, (total_cost, final_path))
                num_of_nodes_expanded += 1
                break

            elif neighbour_node not in path:
                path_to_neighbour = path[:]
                path_to_neighbour.append(neighbour_node)
                total_cost = neighbour_cost + prev_cost
                heapq.heappush(queue, (total_cost, path_to_neighbour))
                num_of_nodes_expanded += 1

        # print(queue, "\n\n")

def A_randomEdge(matrix, start):
    """ Implements A* search with random remaining edge heuristic on an adjesency matrix to solve TSP

      :param matrix: adjesency matrix
      :param start:  start node

      :return: returns (cost to goal, path, num of nodes visited)
    """
    num_of_nodes_expanded = 1
    queue = []  # Frontier

    """ The queue structure holds (heuristic, path, path_cost) where

    :heuristic: path_cost + weight of random remaining edge
    :path: path to current from start
    :path_cost: cost of path from start to current node
  """
    heapq.heappush(queue, (0 + random.choice(remainingEdges(matrix, [start])), [start], 0))

    while queue:
        prev_cost, path, prev_total_cost = heapq.heappop(queue)

        # goal state
        if len(path) >= len(matrix) + 1 and path[0] == path[-1]:
            return (prev_total_cost, path, num_of_nodes_expanded)

        # expand curr node
        curr_node = path[-1]
        for neighbour_node, neighbour_cost in enumerate(matrix[curr_node]):
            # if visited all nodes and attempting to go back to the initial

            if len(path) >= len(matrix):
                return_cost = matrix[curr_node][start]
                total_cost = prev_total_cost + return_cost
                final_path = path + [start]
                heapq.heappush(queue, (total_cost + random.choice(remainingEdges(matrix, final_path)), final_path, total_cost))
                num_of_nodes_expanded += 1
                break

            elif neighbour_node not in path:
                path_to_neighbour = path[:]
                path_to_neighbour.append(neighbour_node)
                total_cost = neighbour_cost + prev_total_cost
                heapq.heappush(queue, (total_cost + random.choice(remainingEdges(matrix, path_to_neighbour)), path_to_neighbour, total_cost))
                num_of_nodes_expanded += 1  

        # print(queue, "\n\n")

def A_cheapestEdge(matrix, start):
    """ Implements A* search with cheapest remaining edge heuristic on an adjesency matrix to solve TSP

      :param matrix: adjesency matrix
      :param start:  start node

      :return: returns (cost to goal, path, num of nodes visited)
    """
    num_of_nodes_expanded = 1
    queue = []  # Frontier

    """ The queue structure holds (heuristic, path, path_cost) where

    :heuristic: path_cost + weight of cheapest remaining edge
    :path: path to current from start
    :path_cost: cost of path from start to current node
  """
    heapq.heappush(queue, (min(remainingEdges(matrix, [start])), [start], 0))

    while queue:
        prev_cost, path, prev_total_cost = heapq.heappop(queue)

        # goal state
        if len(path) >= len(matrix) + 1 and path[0] == path[-1]:
            return (prev_total_cost, path, num_of_nodes_expanded)

        # expand curr node
        curr_node = path[-1]
        for neighbour_node, neighbour_cost in enumerate(matrix[curr_node]):
            # if visited all nodes and attempting to go back to the initial
            if len(path) >= len(matrix):
                return_cost = matrix[curr_node][start]
                total_cost = prev_total_cost + return_cost
                final_path = path + [start]
                heapq.heappush(queue, (total_cost + min(remainingEdges(matrix, final_path)), final_path, total_cost))
                num_of_nodes_expanded += 1
                break

            elif neighbour_node not in path:
                path_to_neighbour = path[:]
                path_to_neighbour.append(neighbour_node)
                total_cost = neighbour_cost + prev_total_cost
                heapq.heappush(queue, (total_cost + min(remainingEdges(matrix, path_to_neighbour)), path_to_neighbour, total_cost))
                num_of_nodes_expanded += 1  

        # print(queue, "\n\n")

def A_MST(matrix, start):
    """ Implements A* search with minimum spanning tree heuristic on an adjesency matrix to solve TSP

      :param matrix: adjesency matrix
      :param start:  start node

      :return: returns (cost to goal, path, num of nodes visited)
    """
    num_of_nodes_expanded = 1
    queue = []  # Frontier

    """ The queue structure holds (heuristic, path, path_cost) where

    :heuristic: path_cost + weight of MST on remaining edge
    :path: path to current from start
    :path_cost: cost of path from start to current node
  """
    heapq.heappush(queue, (0, [start], 0))

    while queue:
        prev_cost, path, prev_total_cost = heapq.heappop(queue)

        # goal state
        if len(path) >= len(matrix) + 1 and path[0] == path[-1]:
            return (prev_total_cost, path, num_of_nodes_expanded)

        # expand curr node
        curr_node = path[-1]
        for neighbour_node, neighbour_cost in enumerate(matrix[curr_node]):
            # if visited all nodes and attempting to go back to the initial
            if len(path) >= len(matrix):
                return_cost = matrix[curr_node][start]
                total_cost = prev_total_cost + return_cost
                final_path = path + [start]
                m_ = np.array(matrix)
                idx_to_keep = np.array(remainingNodes(len(matrix), final_path))
                
                if len(idx_to_keep) == 0:
                    heuristic = 0
                else:
                    m_ = m_[idx_to_keep[:,None], idx_to_keep]
                    heuristic = kruskalMST(m_)

                heapq.heappush(queue, (total_cost + heuristic, final_path, total_cost))
                num_of_nodes_expanded += 1
                break

            elif neighbour_node not in path:
                path_to_neighbour = path[:]
                path_to_neighbour.append(neighbour_node)
                total_cost = neighbour_cost + prev_total_cost
                # create an np appay of the adj matrix to run mst on modified adj matrix
                m_ = np.array(matrix)
                idx_to_keep = np.array(remainingNodes(len(matrix), path_to_neighbour) + [neighbour_node])
                if len(idx_to_keep) == 0:
                    heuristic = 0
                else:
                    m_ = m_[idx_to_keep[:,None], idx_to_keep]
                    heuristic = kruskalMST(m_)
                heapq.heappush(queue, (total_cost + heuristic, path_to_neighbour, total_cost))
                num_of_nodes_expanded += 1  

        # print(queue, "\n\n")

# Entry point
if __name__ == '__main__':
    main(sys.argv[1:])