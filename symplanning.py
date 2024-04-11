import numpy as np
import heapq
import time
class blockPlanning:
    def __init__(self, initState:np.array, goalState:np.array, topPadding:int):
        self.initPose = np.concatenate((initState, np.zeros((topPadding, initState.shape[1]))), axis=0)
        self.goalPose = np.concatenate((goalState, np.zeros((topPadding, goalState.shape[1]))), axis=0)
        self.topPadding = topPadding    
        self.rows = len(initState)+topPadding
        self.cols = len(initState[0])
        colors_init = np.unique(initState)
        colors_goal = np.unique(goalState)
        self.colors = colors_goal if colors_goal.size > colors_init.size else colors_init
        self.colors = self.colors[self.colors != 0]

    def isNoGap(self, state:np.matrix):
        for i in range(self.cols):
            column = state[:, i]
            zero_indices = np.where(column == 0)[0]
            non_zero_indices = np.where(column != 0)[0]
            if zero_indices.size > 0 and non_zero_indices.size > 0:
                if max(zero_indices) > min(non_zero_indices):
                    return False
        return True
    
    def findTopIndex(self,state:np.array):
        # returns the top index (==0), (col,row)
        indexes = []
        for i in range(self.cols):
            column = state[:, i]
            zero_indices = np.where(column == 0)[0]
            indexes.append((i,zero_indices[0]))
        return indexes
        
    def isPhysicallyLegal(self, state:np.array):
        topIndexes = self.findTopIndex(state)
        # check if for each colomn, the difference between top index to adjacent index is less than 2
        for i,(col,row) in enumerate(topIndexes[:-1]):
            allowableMin = max(row - 1,0)
            allowableMax = min(row + 1,self.rows-1)
            if topIndexes[i+1][1] < allowableMin or topIndexes[i+1][1] > allowableMax:
                return False
        return True
            
        
    def getSuccessors(self, state:np.array):
        successors = []
        topIndexes = self.findTopIndex(state)
        # print("State: ", state)
        # for the top indexes, we can directly add blocks
        for (col,row) in topIndexes:
            if row < self.rows - 1: # not adding to the top row
                for color in self.colors:
                    newState = np.copy(state)
                    newState[row][col] = color
                    if self.isPhysicallyLegal(newState):
                        successors.append(newState)
        # print("Successors after add: ", len(successors))
        # for one row below the top indexes, we can move blocks to table (eliminate)
        for (col,row) in topIndexes:
            if row > 0:
                newState = np.copy(state)
                newState[row-1][col] = 0
                if self.isPhysicallyLegal(newState):
                    successors.append(newState)
        # print("Successors after eliminate: ", len(successors))
        # for one row below top indexes, we can move to other top indexes
        for (col,row) in topIndexes:
            if row > 0:
                for (newCol,newRow) in topIndexes:
                    if newCol == col or newRow == self.rows-1:
                        continue
                    newState = np.copy(state)
                    newState[newRow][newCol] = newState[row-1][col]
                    newState[row-1][col] = 0
                    if self.isPhysicallyLegal(newState):
                        successors.append(newState)
        # print("Successors after move: ", len(successors))
        return successors
    
    def CheckVisited(self,state,vertices):
        for i in range(len(vertices)):
            if np.array_equal(state,vertices[i]):
                return True
        return False
    
    def AstarSearch(self):
        pq = []
        vertices = []
        parent = []
        cost2come=[]
        heapq.heappush(pq, (0, 0)) # (cost, vertex_id)
        vertices.append(self.initPose) # (cost, vertex_id)
        parent.append(0)
        cost2come.append(0)
        FoundPath=False
        id = 0
        while pq:
            f, id = heapq.heappop(pq)
            if self.isGoal(vertices[id]):
                FoundPath = True
                break
            # print("Exploring: ")
            # print(vertices[id])
            successors = self.getSuccessors(vertices[id])  
            for s in successors:
                if not self.CheckVisited(s,vertices):
                    g = cost2come[id]+1
                    h = self.heuristic(s)
                    f = g+h
                    vertices.append(s)
                    if (len(vertices))%2000==0:
                        print("Explored: ", len(vertices)-1)
                    parent.append(id)
                    cost2come.append(g)
                    heapq.heappush(pq,(f,len(vertices)-1))
        print("Path Found: ", FoundPath)
        Plan=[]
        x = id
        if FoundPath:
            while not x==0:
                Plan.insert(0,vertices[x])
                x=parent[x]
        print("Plan:")
        for p in Plan:
            print(p)
                

    def isGoal(self, state:np.array):
        return np.array_equal(state, self.goalPose)
    
    def heuristic(self, state:np.array):
        score = 0
        # score = np.sum(state != self.goalPose)
        diff = self.goalPose - state
        for col in range(self.cols):
            for row in range(self.rows):
                if state[row][col] == 0:
                    score += 1
                elif diff[row][col] != 0:
                    score += self.rows - row -self.topPadding
                    break
        return score

if __name__ == '__main__':
    initState = np.array([[1, 2, 1, 1], [1, 2, 1, 2], [2, 2, 2, 1], [2, 1, 2, 1]])
    # initState = np.zeros((4,4))
    goalState = np.array([[1, 2, 1, 2], [1, 2, 2, 1], [1, 2, 1, 2], [2, 1, 2, 1]])

    # initState = np.array([[1, 2, 1], [1, 2, 1], [2, 2, 2]])
    # goalState = np.array([[1, 2, 1], [2, 1, 2], [1, 2, 1]])
    
    # testState = np.array([[1, 1, 1], [0, 1, 1], [0, 1, 1], [0, 0, 0], [0, 0, 0]])
    bp = blockPlanning(initState, goalState,1)
    # print(bp.heuristic(testState))
    # print(bp.isPhysicallyLegal(testState))

    start_time = time.time()
    bp.AstarSearch()
    end_time = time.time()

    execution_time = end_time - start_time
    print(f"Execution time: {execution_time} seconds")