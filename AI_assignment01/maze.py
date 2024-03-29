# --------- Get the Maze Information --------- ##

import re
import copy

class Maze:

    def __init__(self, filename):
        self.__filename = filename
        self.__wall = '#'
        self.__startchar = 'T'
        self.__objectivechar = '.'
        self.__start = None ## C언어에서의 NULL
        self.__objective = []
        self.__states_search = 0

        with open(filename) as f:
            lines = f.readlines()
        ## filename에서 모든 Line 받아옴, with as : 구문 빠져나가면 자동으로 close ##

        lines = list(filter(lambda x: not re.match(r'^\s*$', x), lines))
        lines = [list(line.strip('\n')) for line in lines]
        ## strip : 개행 제거, r'&\s*$' : matches an empty line?

        self.rows = len(lines)
        self.cols = len(lines[0])
        self.mazeRaw = lines

        if (len(self.mazeRaw) != self.rows) or (len(self.mazeRaw[0]) != self.cols):
            print("Maze dimensions incorrect")
            raise SystemExit
            return

        for row in range(len(self.mazeRaw)):
            for col in range(len(self.mazeRaw[0])):
                if self.mazeRaw[row][col] == self.__startchar:
                    self.__start = (row, col)   ## 시작점 'T' 좌표 저장
                elif self.mazeRaw[row][col] == self.__objectivechar:
                    self.__objective.append((row, col)) ## 목표점 '.' 좌표 저장

    def isWall(self, row, col):
        return self.mazeRaw[row][col] == self.__wall

    def isObjective(self, row, col):
        return (row, col) in self.__objective

    ## 시작 위치의 tuple(row, col)을 return
    def startPoint(self):
        return self.__start

    def setStart(self, start):
        self.__start = start

    def getDimensions(self):
        return (self.rows, self.cols)

    ## 원 위치에 해당하는 tuple list[(row1, col1), (row2, col2)]을 return
    def circlePoints(self):
        return copy.deepcopy(self.__objective)


    def setObjectives(self, objectives):
        self.__objective = objectives


    def getStatesSearch(self):
        return self.__states_search

    def choose_move(self, row, col):
        return row >= 0 and row < self.rows and col >= 0 and col < self.cols and not self.isWall(row, col)

    ## 현재 위치에서 인접 위치에 해당하는 tuple list를 return
    def neighborPoints(self, row, col):
        possibleNeighbors = [
            (row + 1, col),
            (row - 1, col),
            (row, col + 1),
            (row, col - 1)
        ]
        neighbors = []
        for r, c in possibleNeighbors:
            if self.choose_move(r, c):
                neighbors.append((r,c))
        self.__states_search += 1
        return neighbors

    def isValidPath(self, path):

        for i in range(1, len(path)):
            prev = path[i-1]
            cur = path[i]
            dist = abs((prev[1]-cur[1])+(prev[0]-cur[0]))
            if dist > 1:
                return "Not single hop"

        for pos in path:
            if not self.choose_move(pos[0], pos[1]):
                return "Not valid move"

        if not set(self.__objective).issubset(set(path)):
            return "Not all goals passed"


        if not path[-1] in self.__objective:
            return "Last position is not goal"

        return "Valid"
