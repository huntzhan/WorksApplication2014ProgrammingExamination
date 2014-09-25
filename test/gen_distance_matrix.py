import random


def print_matrix(width, height):
    matrix = []
    for _ in range(height):
        row = [0] * width
        matrix.append(row)
    # fill matrix.
    for x in range(width):
        for y in range(height):
            if x >= y:
                continue
            distance = random.randint(1, 100)
            matrix[x][y] = distance
            matrix[y][x] = distance
    # print it.
    for row in matrix:
        line = ", ".join(map(str, row))
        print "{" + line + "},"


if __name__ == '__main__':
    print_matrix(18, 18)
