from util import *
import heapq
import sys
import time

def main(argv):
    if argv:
        filename = argv[1]
        func = argv[0]
    else:  # default filename
        filename = 'infile05_01.txt'
        func = 'ucs'
        
    matrix = get_matrix(filename)
    if matrix:
        start = 0
        # start = random.randint(0, len(matrix) - 1) 
        initial_time = time.time()
        initial_cpu = time.process_time()
        if func == 'ucs':
            print(A_uniformCost(matrix, start), time.time() - initial_time, time.process_time() - initial_cpu)
        

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
                # return_cost, return_path, nodes_expanded = shortest_path_uniformCost(matrix, neighbour_node, start)
                # final_path = path + return_path
                # total_cost = prev_cost + neighbour_cost + return_cost
                # num_of_nodes_expanded += nodes_expanded
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


# Entry point
if __name__ == '__main__':
    main(sys.argv[1:])