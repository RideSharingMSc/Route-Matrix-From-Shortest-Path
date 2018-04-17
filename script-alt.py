from collections import defaultdict, deque

import threading
import copy
import math

timeIndex = 1
threadCount = 1

class myThread (threading.Thread):

   def __init__(self, tid, matrix, points, graph):
      threading.Thread.__init__(self)
      self.threadID = tid
      self.matrix = matrix
      self.points = points
      self.graph = graph

   def run(self):
      build_matrix(self.threadID, self.matrix, self.points, self.graph)


class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.distances[(from_node, to_node)] = distance


def dijkstra(graph, initial):
    visited = {initial: 0}
    path = {}

    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                weight = current_weight + graph.distances[(min_node, edge)]
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path


def shortest_path(graph, origin, destination):
    visited, paths = dijkstra(graph, origin)
    full_path = deque()
    _destination = paths[destination]

    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]

    full_path.appendleft(origin)
    full_path.append(destination)

    return visited[destination], list(full_path)


def build_matrix(tid, matrix, points, graph):

    range_value = int(math.ceil(len(points)/threadCount)) + 1

    for i in range(tid * range_value, (tid + 1) * range_value):
        print(i)
        if i >= len(points):
            break

        x = points[i]
        for y in points:
            if(tid == 0):
                print(y)
            if x == y:
                matrix[int(x) - 1][int(y) - 1] = 0
            else:
                matrix[int(x)-1][int(y)-1] = str(shortest_path(graph, x, y)).split(",")[0]


if __name__ == '__main__':

    graph = Graph()
    points = []

    print("Loading graph nodes")
    with open('points.csv', 'r') as f:
        for line in f:
            node = line.split(",")[0]
            graph.add_node(node)
            points.append(node)
    f.close()

    outerCount = 0;
    print("Loading graph edges")
    with open('edges.csv', 'r') as f:
        for line in f:
            startNode = line.split(",")[1]
            endNode = line.split(",")[2].split("\n")[0]

            with open('week.csv', 'r') as fp:
                innerCount = 0
                for costLine in fp:
                    if innerCount == outerCount:
                        edgeCost = costLine.split(",")[timeIndex]
                        graph.add_edge((startNode), (endNode), int(edgeCost))
                        break
                    innerCount = innerCount + 1
            fp.close()

            outerCount = outerCount + 1
    f.close()

    print("Building routing matrix")

    numColumns = len(points)
    colLength = len(points)

    matrix = [[0 for x in range(numColumns)] for y in range(colLength)]

    threads = []


    try:
        for tid in range(0, threadCount):
            thread = myThread(tid, matrix, points, graph)

            thread.start()

            threads.append(thread)

    except:
        print "Error: unable to start threads"

    for t in threads:
        t.join()

    print(matrix[0][3])
