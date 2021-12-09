import math
from Map import *


class generalNode:
    def __init__(self, heuristic, position, edge=None):
        self.total = math.inf
        self.edgeDistance = edge
        self.manDistance = heuristic
        self.parent = None
        self.neighbours = []
        self.position = position


class StartSearch:
    def __init__(self, task=1):
        self.map = Map_Obj(task=task)
        self.intMap, self.stringMap = self.map.get_maps()
        self.nodeArray = []
        self.goal = self.map.get_goal_pos()
        self.start = self.map.get_start_pos()
        self.initializeMaze()

    def initializeMaze(self):
        """
        Initializes a maze by using intMap from Map.py and replacing all non-negative numbers
        with generalNodes. Uses Manhattan-distance to measure heuristic values.
        Also initiates every node with a list of neighbours in order to make iteration
        easier in the main search-algorithm. Makes main A*-algorithm iterate through an array
        of nodes rather than array of numbers/strings.
        :return None:
        """
        intMap = self.intMap
        for i in range(len(intMap)):
            currentAxis = []
            for j in range(len(intMap[i])):
                edge = intMap[i][j]
                if edge != -1:
                    vertical_heuristic = self.goal[0] - i
                    horizontal_heuristic = self.goal[1] - j
                    currentNode = generalNode(abs(vertical_heuristic) + abs(horizontal_heuristic), [i, j], edge)
                    currentAxis.append(currentNode)
                    # Check if there are any neighbours above or to the left of current node
                    if i > 0 and self.nodeArray[i - 1][j] != 'X':
                        nodeY = self.nodeArray[i - 1][j]
                        if self.checkNeighbours(currentNode, nodeY.neighbours) \
                                and self.checkNeighbours(nodeY, currentNode.neighbours):
                            nodeY.neighbours.append(currentNode)
                            currentNode.neighbours.append(nodeY)
                    if j > 0 and currentAxis[j - 1] != 'X':
                        nodeX = currentAxis[j - 1]
                        if self.checkNeighbours(currentNode, nodeX.neighbours) \
                                and self.checkNeighbours(nodeX, currentNode.neighbours):
                            nodeX.neighbours.append(currentNode)
                            currentNode.neighbours.append(nodeX)
                else:
                    currentAxis.append('X')
            self.nodeArray.append(currentAxis)

    def checkNeighbours(self, inputNode, neighbourList):
        """
        Checks if inputNode is contained in neighbourList.
        :param inputNode: node to check
        :param neighbourList: list with nodes to check whether or not contains inputNode-object
        :return:False if inputNode is in neighbourList, True otherwise
        """
        for checkNode in neighbourList:
            if inputNode.position.__eq__(checkNode.position):
                return False
        return True

    def visualizeManhattanDistance(self):
        """
        Calculates the manhattan distance between all nodes and goal,
        and visualizes it by printing arrays.
        :return: None
        """
        visual = []
        for i in range(len(self.nodeArray)):
            tmp = []
            if self.nodeArray[i]:
                for j in range(len(self.nodeArray[i])):
                    if self.nodeArray[i][j] != 'X':
                        tmp.append(self.nodeArray[i][j].manDistance)
                    else:
                        tmp.append('X')
            print(tmp)
            visual.append(tmp)
        # print(visual)

    def Astar(self):
        """
        Calculates the shortest path from the start node to the finish node, using a heuristic and weighted value.
        :return:None
        """
        openNodes = []
        closedNodes = []
        # Defines the start node
        n_0 = self.nodeArray[self.start[0]][self.start[1]]
        var3 = n_0.position
        openNodes.append(n_0)
        while len(openNodes) >= 1:
            currentNode = openNodes.pop()
            # If the current node is a goal node, recursively create and return a path through parent nodes back to n_0
            if currentNode.position.__eq__(self.goal):
                finished = False
                finalPath = [currentNode]
                parentNode = currentNode.parent
                while not finished:
                    finalPath.append(parentNode)
                    parentNode = parentNode.parent
                    finished = (parentNode.position == self.start)
                return finalPath
            # Iterates through every neighbour of currentNode
            for neighbour in currentNode.neighbours:
                tempEdgeDistance = neighbour.edgeDistance + currentNode.edgeDistance
                tempTotal = tempEdgeDistance + neighbour.manDistance

                # Will check if there exists a node in either open or closed list that has lower cost than current path
                checkedOpenNode = next((x for x in openNodes if x.position == neighbour.position), None)
                checkedClosedNode = next((x for x in closedNodes if x.position == neighbour.position), None)
                if checkedOpenNode and checkedOpenNode.edgeDistance <= tempEdgeDistance:
                    continue
                if checkedClosedNode and checkedClosedNode.edgeDistance <= tempEdgeDistance:
                    continue

                neighbour.edgeDistance = tempEdgeDistance
                neighbour.total = tempTotal
                openNodes.append(neighbour)
                openNodes.sort(key=lambda node: -node.total)
                neighbour.parent = currentNode
            closedNodes.append(currentNode)


task_number = int(input("Type in task number:"))
exercise = StartSearch(task=task_number)  # WRITE TASK NR. HERE (default=1)
map_visualizer = Map_Obj(task=task_number)  # WRITE TASK NR. HERE (default=1)
path = exercise.Astar()
intMap, stringMap = map_visualizer.get_maps()
for i in range(len(intMap)):
    for j in range(len(intMap[i])):
        for node in path:
            if node.position[0] == i and node.position[1] == j:
                # Path will be visualized due to adding ' X ': (0, 0, 255) to
                # colors variable in Map_Obj.show_map
                stringMap[i][j] = ' X '

map_visualizer.show_map(stringMap)
