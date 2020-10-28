def print_matrix(m):
    for r in range(len(m)):
        for c in range(len(m[r])):
            print(m[r][c], end=' ')
        print()

def get_matrix(filename):
    try:
        txt = open(filename, 'r').readlines()[1:]
    except IOError:
        print('No such file or directory')
        return 0

    matrix = []
    for l in txt:
        temp = []
        for n in l.split():
            temp.append(int(n))
        matrix.append(temp)

    return matrix

def remainingEdges(matrix, path):
    """ Given a path and adj matrix it returns a list of nodes not visited in the path
    """
    res = []
    for i, j in enumerate(matrix[path[-1]]):
        if i not in path:
            res.append(j)
    # print(res)
    return res if res != [] else [0]

def remainingNodes(n, path):
    # return list(set((i for i in range(n)))-set(path))
    res = []
    for i in range(n):
        if i not in path:
            res.append(i)
    return res

# Sourced from Geeks for Geeks 
# Python implementation for Kruskal's Algorithm
# Finds MST using Kruskal's algorithm  
def kruskalMST(cost): 
    V = len(cost)
    parent = [i for i in range(V)] 
    mincost = 0 # Cost of min MST 
    
    def find(i): 
        while parent[i] != i: 
            i = parent[i] 
        return i   

    def union(i, j): 
        a = find(i) 
        b = find(j) 
        parent[a] = b 

    # Initialize sets of disjoint sets 
    for i in range(V): 
        parent[i] = i 
  
    # Include minimum weight edges one by one  
    edge_count = 0
    edge_list = []
    while edge_count < V - 1: 
        min = float('inf') 
        a = -1
        b = -1
        for i in range(V): 
            for j in range(V): 
                if find(i) != find(j) and cost[i][j] < min: 
                    min = cost[i][j] 
                    a = i 
                    b = j 
        union(a, b) 
        edge_list.append((a,b))
        # print('Edge {}:({}, {}) cost:{}'.format(edge_count, a, b, min)) 
        edge_count += 1
        mincost += min
  
    # print("Minimum cost= {}".format(mincost)) 
    # return (edge_list, mincost)
    return mincost
