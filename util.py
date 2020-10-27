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