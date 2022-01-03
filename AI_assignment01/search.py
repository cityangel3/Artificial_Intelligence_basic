###### Write Your Library Here ###########
from collections import defaultdict
from collections import deque
from heapq import *
import copy

#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """

    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################
    visited = [[False] * maze.cols for _ in range(maze.rows)]
    visited[start_point[0]][start_point[1]] = True
    neighbor = []
    queue = deque([start_point])
    route = Node((-1, -1), start_point)
    route.obj.append([route.parent, route.location])
    while queue:
        x, y = queue.popleft()
        if maze.isObjective(x, y):
            break
        neighbor = maze.neighborPoints(x, y)
        route.parent = (x, y)
        for i in range(len(neighbor)):
            nx, ny = neighbor.pop()
            route.location = (nx, ny)
            if not visited[nx][ny]:
                visited[nx][ny] = True
                queue.append((nx, ny))
                route.obj.append([route.parent, route.location])
    pt = None
    while route.obj:
        temp = route.obj.pop()
        cur = temp.pop()
        if maze.isObjective(cur[0], cur[1]):
            pt = temp.pop()
            path.append(cur)
        elif cur == pt:
            path.append(pt)
            pt = temp.pop()

    return path[::-1]
    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[]

        # F = G+H
        self.f=0
        self.g=0
        self.h=0

    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def astar(maze):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]

    path=[]

    ####################### Write Your Code Here ################################
    st_node = Node(None, start_point)
    frontier = []
    reached = []

    frontier.append(st_node)
    while frontier:
        cur_node = frontier[0]
        cur_index = 0
        ## frontier 중 가장 작은 f(=g+h)를 가진 노드 선별
        for i, temp in enumerate(frontier):
            if cur_node.__gt__(temp):
                cur_node = temp
                cur_index = i
        frontier.pop(cur_index)
        reached.append(cur_node)
        if cur_node.location == end_point:
            cursor = cur_node
            while cursor is not None:
                path.append(cursor.location)
                cursor = cursor.parent
            return path[::-1]
        neighbors = maze.neighborPoints(cur_node.location[0],cur_node.location[1])
        nb_nodes = []
        while neighbors:
            new_node = Node(cur_node, neighbors.pop())
            nb_nodes.append(new_node)
        for nb in nb_nodes:
            if nb in reached:
                continue
            nb.g = cur_node.g + 1
            nb.h = manhatten_dist(nb.location, end_point)
            nb.f = nb.g + nb.h

            test = [ft for ft in frontier if nb == ft and nb.g > ft.g]
            if len(test) == 0:
                frontier.append(nb)


    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #

def stage2_heuristic(location, remain_obj, row, col):
    dist, idx = 100000, (0,0)
    if not remain_obj:
        return 0
    for pt in remain_obj:
        temp = manhatten_dist(location, pt)
        if temp < dist:
            dist = temp
            idx = pt
    if len(remain_obj) == 4:
        dist += (row + col + min(row, col))
    elif len(remain_obj) == 3:
        dist += (row + col)
    elif len(remain_obj) == 2:
        if remain_obj[0][0] != remain_obj[1][0] and remain_obj[0][1] != remain_obj[1][1]:
            dist += (row + col)
        elif remain_obj[0][0] != remain_obj[1][0]:
            dist += col
        else:
            dist += row

    return dist

def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path = []
    ####################### Write Your Code Here ################################
    start_point = maze.startPoint()
    st_node = Node(None, start_point)
    st_node.obj.extend(end_points)
    frontier = []
    reached = defaultdict(int)
    heappush(frontier, [st_node.f, st_node])
    while frontier:
        info = heappop(frontier)
        cur_node = info[1]
        reached[(cur_node.location, tuple(cur_node.obj))] = 1

        if not cur_node.obj:
            cursor = cur_node
            while cursor is not None:
                path.append(cursor.location)
                cursor = cursor.parent
            return path[::-1]

        neighbors = maze.neighborPoints(cur_node.location[0], cur_node.location[1])
        nb_nodes = []
        while neighbors:
            new_node = Node(cur_node, neighbors.pop())
            new_node.obj.extend(cur_node.obj)
            nb_nodes.append(new_node)
        for nb in nb_nodes:
            if reached[(nb.location, tuple(nb.obj))]:
                continue
            if nb.location in end_points and nb.location in nb.obj:
                nb.obj.remove(nb.location)
            nb.g = cur_node.g + 1
            nb.h = stage2_heuristic(nb.location,nb.obj,maze.rows-2,maze.cols-2)
            nb.f = nb.g + nb.h
            heappush(frontier, [nb.f, nb])
    return path

    ############################################################################



# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #


def weight(p1,p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])


def mst(objectives, edges, start):

    cost_sum=0
    ####################### Write Your Code Here ################################
    graph = defaultdict(list)
    for u in objectives:
        for v in objectives:
            if u == v:
                continue
            w = edges[u][v]
            graph[u].append((w,u,v))
    connected = set()
    connected.add(start)
    candidate = copy.deepcopy(graph[start])
    heapify(candidate)
    while candidate:
        w, u ,v = heappop(candidate)
        if v not in connected:
            connected.add(v)
            cost_sum += w
        for edge in graph[v]:
            if edge[2] not in connected:
                heappush(candidate, edge)
    return cost_sum

    ############################################################################


def stage3_heuristic(edges, nb, sub_flag ,flag):
    dist = 100000
    prim_start = (0,0)
    if not nb.obj:
        return 0
    if nb.parent.location in sub_flag[1] or nb.parent.location == sub_flag[0]:
        flag = 2
    for ob in nb.obj:
        if flag == 1:
            temp = edges[nb.location][ob]
        elif flag == 2:
            temp = max(weight(nb.location,ob), (edges[nb.parent.location][ob] ))
        else:
            temp = weight(nb.location,ob)
        if temp < dist:
            dist = temp
            prim_start = ob
    dist += mst(nb.obj, edges, prim_start)
    return dist


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points = maze.circlePoints()
    end_points.sort()
    path = []
    ####################### Write Your Code Here ################################
    start_point = maze.startPoint()

    end_points.insert(0, start_point)
    edges = defaultdict(dict)
    for i in range(len(end_points)):
        for j in range(i+1, len(end_points)):
            frontier = []; reached = []
            st_node = Node(None, end_points[i])
            heappush(frontier,[st_node.f,st_node])
            val = -1
            while frontier:
                top = heappop(frontier)
                cur_node = top[1]
                reached.append(cur_node)
                if cur_node.location == end_points[j]:
                    cursor = cur_node
                    while cursor is not None:
                        val += 1
                        cursor = cursor.parent
                    edges[end_points[i]][end_points[j]] = val
                    edges[end_points[j]][end_points[i]] = val
                    break
                neighbors = maze.neighborPoints(cur_node.location[0], cur_node.location[1])
                nb_nodes = []
                while neighbors:
                    new_node = Node(cur_node, neighbors.pop())
                    nb_nodes.append(new_node)
                for nb in nb_nodes:
                    if nb in reached:
                        continue

                    nb.g = cur_node.g + 1
                    nb.h = weight(nb.location,end_points[j])
                    nb.f = nb.g + nb.h
                    heappush(frontier, [nb.f, nb])

    frontier = []
    check = defaultdict(int)
    end_points.remove(start_point)
    st_node = Node(None, start_point)
    st_node.obj = copy.deepcopy(end_points)
    heappush(frontier, [st_node.f, st_node])

    while frontier:
        info = heappop(frontier)
        cur_node = info[1]
        check[(cur_node.location, tuple(cur_node.obj))] = 1
        if not cur_node.obj:
            cursor = cur_node
            while cursor is not None:
                path.append(cursor.location)
                cursor = cursor.parent
            return path[::-1]

        candidate = maze.neighborPoints(cur_node.location[0], cur_node.location[1])
        nb_nodes = []

        while candidate:
            new_node = Node(cur_node, candidate.pop())
            new_node.obj.extend(cur_node.obj)
            nb_nodes.append(new_node)
        for nb in nb_nodes:
            flag = 0
            if nb.location in end_points:
                if nb.location in nb.obj:
                    flag = 1
                    nb.obj.remove(nb.location)
            if check[(nb.location, tuple(nb.obj))]:
                continue
            nb.g = cur_node.g + 1
            nb.h = stage3_heuristic(edges, nb, (start_point, end_points), flag)
            nb.f = nb.g + nb.h

            heappush(frontier, [nb.f, nb])
    return path
    ############################################################################
