import rospy
from Queue import Queue
from explorer_node_base import ExplorerNodeBase
import math
from collections import deque
import random

# This class implements a super super super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

def get_median(my_list):
    # gets the median of the list, if length is even it gets the middle element, 
    # if odd it floors the value instead of getting middle value, since this
    # may not be a reachable place
    my_list.sort()
    idx = 0
    if len(my_list) == 0:
        print('List is empty.')
        return None

    if len(my_list) % 2 == 0:
	idx = len(my_list)/2
        return my_list[idx]

    if len(my_list) % 2 != 0:
	idx = len(my_list)//2
        return my_list[idx]
    

class ExplorerNodeWFD(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)
        self.blackList = []
        self.visitedFrontiers = []
        print('Using WFD explorer')

    def updateFrontiers(self):
        print('Updating frontiers')
        print('Current number of frontiers: ', len(self.FrontierList))
        # Clear queue and put initial pose in qm and MapOpen
        queueM = deque()
        queueF = deque()
        MapOpen = deque()
        MapClose = deque()
        FrontierOpen = deque()
        FroniterClose = deque()
        self.FrontierList[:] = []
        pose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((self.pose.x,self.pose.y))
        queueM.append(pose)
        MapOpen.append(pose)

        # While qm is not empty loop over
        while len(queueM) != 0:
            
            #remove element from qm
            p = queueM.pop()

            # If the removed element is in MapClose go back to start of while loop
            if self.markedAs(p, MapOpen, MapClose, FrontierOpen, FroniterClose)["MapClose"] == True:
                continue

            # If p is a frontier cells then do this
            if self.isFrontierCell(p[0],p[1]) is True:

                # clear qf list, create new list for frontier and p in both
                queueF.clear()
                NewFrontier = deque()
                queueF.append(p)
                FrontierOpen.append(p)

                # while qf is not empty loop over
                while len(queueF) != 0:

                    # get element form qf
                    q = queueF.pop()

                    # if q is in MapClose or FrontierClose return to top of while loop
                    mark = self.markedAs(q, MapOpen, MapClose, FrontierOpen, FroniterClose)
                    if (mark["MapClose"] == True) or (mark["FrontierClose"] == True):
                        continue

                    # if q is a frontier add to new frontier
                    if self.isFrontierCell(q[0],q[1]) is True:
                        NewFrontier.append(q)

                        # for all cells adjacent to q add to qf if they are not marked as follows                    
                        for w in self.getAllAdjacentCells(q):
                            mark = self.markedAs(w, MapOpen, MapClose, FrontierOpen, FroniterClose) 
                            if (mark["FrontierOpen"] == False) and (mark["FrontierClose"] == False) and (mark["MapClose"] == False):
                                queueF.append(w)
                                FrontierOpen.append(w)
                
                    # add q to frontier close
                    FroniterClose.append(q)

                # save NewFrontier
                self.FrontierList.append(NewFrontier)

                # add every point in NewFrontier to MapClose
                for points in list(NewFrontier):
                    MapClose.append(points)

            # for all cells adjacent to p, add to qm and MapOpen if they meet the conditions
            for cells in self.getAllAdjacentCells(p):
                mark = self.markedAs(cells, MapOpen, MapClose, FrontierOpen, FroniterClose)
                if (mark["MapOpen"] == False) and (mark["MapClose"] == False):
                    if self.checkAdjacent(cells) == True:
                        queueM.append(cells)
                        MapOpen.append(cells)

            # add p to map close
            MapClose.append(p)
        print('Finished updating frontiers current number of frontiers: ', len(self.FrontierList))


    def chooseNewDestination(self):

        # get current pose
        pose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((self.pose.x,self.pose.y))
        median = []
        AddCandidate = True
        distance = float("inf")
        destination = None
 
        # update the frontiers 
        self.updateFrontiers()
 
        # if after updating it is still empty, we've visited everything and we're done
        if len(self.FrontierList) == 0:
            return False, None

        # get median of every frontier, if the median is not in a unreachable
        # place and has not been visited before then add to list median
        for Frontier in self.FrontierList:
            if len(Frontier) != 0:
                AddCandidate = True
                candidate = self.getMedianOfFrontier(Frontier)
                for blacklisted in self.blackList:
                    if blacklisted == candidate:
                        AddCandidate = False
                for visited in self.visitedFrontiers:
                    if visited == candidate:
                        AddCandidate = False
                if AddCandidate == True:
                    median.append((candidate, Frontier))

        # if median list is empty all frontiers have been explored
        if len(median) == 0:
            return False, None

        # get the median that is closest to us and send the destination
        for values in median:
            dX = pose[0] - values[0][0]
            dY = pose[1] - values[0][1]
            current_distance = math.sqrt(dX * dX + dY * dY)
            if current_distance < distance:
                distance = current_distance
                destination = values[1]

        destination = random.sample(destination,1)
        #print("DESTINATION: ", destination)
        print("Destination is frontier: ", self.isFrontierCell(destination[0][0], destination[0][1]))
        return True, destination[0]


    def destinationReached(self, goal, goalReached):
        # if the goal was not reached add the goal to blacklist
        if goalReached is True:
            self.visitedFrontiers.append(goal)
        if goalReached is False:
            self.blackList.append(goal)


    def markedAs(self, p, MapOpen, MapClose, FrontierOpen, FrontierClose):
        # check if p (the coordinate) is in any of the lists and return
        # dict indicating which it is in/not in
        marked_as = {"MapOpen": False, "MapClose": False, "FrontierOpen": False, "FrontierClose": False}
    
        for cell in list(MapOpen):
                if cell == p:
                    marked_as["MapOpen"] = True

        for cell in list(MapClose): 
                if cell == p:
                    marked_as["MapClose"] = True
        
        for cell in list(FrontierOpen):
                if cell == p:
                    marked_as["FrontierOpen"] = True
        
        for cell in list(FrontierClose):
                if cell == p:
                    marked_as["FrontierClose"] = True
        
        return marked_as
    
    def getAllAdjacentCells(self, p):
        # returns a list of adjacent cells if they are in grid
        AdjacentCells = []
        if ( p[0] - 1 >= 0) & ((p[0] - 1) < self.occupancyGrid.getWidthInCells()) & ((p[1] - 1) >= 0) & ((p[1] - 1) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] - 1, p[1] - 1))

        if ( p[0] - 0 >= 0) & ((p[0] - 0) < self.occupancyGrid.getWidthInCells()) & ((p[1] - 1) >= 0) & ((p[1] - 1) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] - 0, p[1] - 1))

        if ( p[0] + 1 >= 0) & ((p[0] + 1) < self.occupancyGrid.getWidthInCells()) & ((p[1] - 1) >= 0) & ((p[1] - 1) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] + 1, p[1] - 1))

        if ( p[0] + 1 >= 0) & ((p[0] + 1) < self.occupancyGrid.getWidthInCells()) & ((p[1] - 0) >= 0) & ((p[1] - 0) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] + 1, p[1] - 0))

        if ( p[0] + 1 >= 0) & ((p[0] + 1) < self.occupancyGrid.getWidthInCells()) & ((p[1] + 1) >= 0) & ((p[1] + 1) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] + 1, p[1] + 1))

        if ( p[0] - 0 >= 0) & ((p[0] - 0) < self.occupancyGrid.getWidthInCells()) & ((p[1] + 1) >= 0) & ((p[1] + 1) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] + 0, p[1] + 1))

        if ( p[0] - 1 >= 0) & ((p[0] - 1) < self.occupancyGrid.getWidthInCells()) & ((p[1] + 1) >= 0) & ((p[1] + 1) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] - 1, p[1] + 1))

        if ( p[0] - 1 >= 0) & ((p[0] - 1) < self.occupancyGrid.getWidthInCells()) & ((p[1] - 0) >= 0) & ((p[1] - 0) < self.occupancyGrid.getHeightInCells()):
            AdjacentCells.append((p[0] - 1, p[1] + 0))

        return AdjacentCells

    def checkAdjacent(self, cells):
        # checks if at least 1 cell adjacent is open and returns
        # true if thats the case, false if not
        adjCells = self.getAllAdjacentCells(cells)
        for cell in adjCells:
            if self.occupancyGrid.getCell(cell[0], cell[1]) == 0:
                return True
        return False
    
    def getMedianOfFrontier(self, Frontier):
        # returns the median of the frontier
        x = []
        y = []
        for cells in Frontier:
            x.append(cells[0])
            y.append(cells[1])
        x_median = get_median(x)
        y_median = get_median(y)
        
        return (x_median, y_median)
        




