from collections import defaultdict
from heapq import *

import threading
import math
import time
import csv

timeIndex = 1
threadCount = 1

class myThread (threading.Thread):

   def __init__(self, tid, matrix, points, edges):
      threading.Thread.__init__(self)
      self.threadID = tid
      self.matrix = matrix
      self.points = points
      self.edges = edges

   def run(self):
      build_matrix(self.threadID, self.matrix, self.points, self.edges)

def dijkstra(edges, f, t):
    g = defaultdict(list)
    for l,r,c in edges:
        g[l].append((c,r))

    q, seen = [(0,f,())], set()
    while q:
        (cost,v1,path) = heappop(q)
        if v1 not in seen:
            seen.add(v1)
            path = (v1, path)
            if v1 == t: return (cost, path)

            for c, v2 in g.get(v1, ()):
                if v2 not in seen:
                    heappush(q, (cost+c, v2, path))

    return float("inf")

def build_matrix(tid, matrix, points, edges):

    tot_count = len(points) * len(points)
    my_count = 0

    range_value = int(math.ceil(len(points)/threadCount)) + 1

    for i in range(tid * range_value, (tid + 1) * range_value):

        if i >= len(points):
            break

        x = points[i]
        for y in points:

            if x == y:
                matrix[int(x) - 1][int(y) - 1] = 0
            else:
                val = str(dijkstra(edges, x, y))
                if val == "inf" :
                    matrix[int(x) - 1][int(y) - 1] = -1
                else:
                    matrix[int(x)-1][int(y)-1] = int(((val.split(",")[0]).split("("))[1])
            my_count = my_count + 1
            print(my_count),
            print("/"),
            print (tot_count)

if __name__ == '__main__':

    points = []
    edges = []

    print("Loading graph nodes")
    with open('points.csv', 'r') as f:
        for line in f:
            node = line.split(",")[0]
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
                        edges.append((startNode, endNode, int(edgeCost)))
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
    start = time.time()
    try:
        for tid in range(0, threadCount):
            start1 = time.time()
            new_points = points[:]
            new_edges = edges[:]
            thread = myThread(tid, matrix, new_points, new_edges)
            thread.start()
            threads.append(thread)

    except:
        print "Error: unable to start threads"

    for t in threads:
        t.join()

    end = time.time()
    print("Time Taken : "),
    print(end - start)

    print("Writing matrix to a file")

    fileName = "week-" + str(timeIndex) + ".csv"

    ofile = open(fileName, "wb")
    writer = csv.writer(ofile, delimiter=',')

    for row in matrix:
        writer.writerow(row)

    ofile.close()
