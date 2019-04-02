import rospy
from Queue import Queue
from explorer_node_base import ExplorerNodeBase
import math
from collections import deque

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
                   

    def updateFrontiers(self,pose):

        # Clear queue and put initial pose in qm and MapOpen   
	self.queueM.clear() 
        self.queueM.append(pose)
        self.MapOpen.append(pose)

        # While qm is not empty loop over
        while len(self.queueM) != 0:

            #remove element from qm
            p = self.queueM.pop()

            # If the removed element is in MapClose go back to start of while loop
            if self.markedAs(p)["MapClose"] == True:
                continue

            # If p is a frontier cells then do this
            if self.isFrontierCell(p[0],p[1]) is True:

                # clear qf list, create new list for frontier and p in both
                self.queueF.clear()
                NewFrontier = deque()
                self.queueF.append(p)
                self.FrontierOpen.append(p)

                # while qf is not empty loop over
                while len(self.queueF) != 0:

                    # get element form qf
                    q = self.queueF.pop()

                    # if q is in MapClose or FrontierClose return to top of while loop
                    mark = self.markedAs(p)
                    if (mark["MapClose"] == True) or (mark["FrontierClose"] == True):
                        continue

                    # if q is a frontier add to new frontier
                    if self.isFrontierCell(q[0],q[1]) is True:
                        NewFrontier.append(q)

                        # for all cells adjacent to q add to qf if they are not marked as follows                    
                        for w in self.getAllAdjacentCells(q):
                            mark = self.markedAs(w) 
                            if (mark["FrontierOpen"] == False) or (mark["FrontierClose"] == False) or (mark["MapClose"] == False):
                                self.queueF.append(w)
				self.FrontierOpen.append(w)
                    
                    # add q to frontier close
                    self.FroniterClose.append(q)

                # save NewFrontier
                self.FrontierList.append(NewFrontier)

                # add every point in NewFrontier to MapClose
                for points in list(NewFrontier):
                    self.MapClose.append(points)

            # for all cells adjacent to p, add to qm and MapOpen if they meet the conditions
            for cells in self.getAllAdjacentCells(p):
                mark = self.markedAs(cells)
                if (mark["MapOpen"] == False) or (mark["MapClose"] == False):
                    if self.checkAdjacent(cells) == True:
                        self.queueM.append(cells)
                        self.MapOpen.append(cells)

            # add p to map close
            self.MapClose.append(p)


    def chooseNewDestination(self, pose):

        median = []
        AddCandidate = True
        distance = float("inf")
        destination = None
 
        # update the frontiers 
        self.updateFrontiers(pose)
 
        # if after updating it is still empty, we've visited everything and we're done
        if len(self.FrontierList) == 0:
            return False, None

        # get median of every frontier, if the median is not in a unreachable
        # place then add to list median
        for Frontier in self.FrontierList:
            candidate = self.getMedianOfFrontier(Frontier)
            for blacklisted in self.blackList:
                if blacklisted == candidate:
                    AddCandidate = False
            if AddCandidate == True:
                median.append(candidate)

        # if median list is empty all frontiers have been explored
        if len(median) == 0:
            return False, None

        # get the median that is closest to us and send the destination
        for values in median:
            dX = pose[0] - values[0]
            dY = pose[1] - values[1]
            current_distance = math.sqrt(dX * dX + dY * dY)
            if current_distance < distance:
                distance = current_distance
                destination = values

        return True, destination


    def destinationReached(self, goal, goalReached):
        # if the goal was not reached add the goal to blacklist
        if goalReached is False:
            self.blackList.append(goal)

    def markedAs(self, p):
        # check if p (the coordinate) is in any of the lists and return
        # dict indicating which it is in/not in
        marked_as = {"MapOpen": False, "MapClose": False, "FrontierOpen": False, "FrontierClose": False}
    
        for cell in list(self.MapOpen):
                if cell == p:
                    marked_as["MapOpen"] = True

        for cell in list(self.MapClose): 
                if cell == p:
                    marked_as["MapClose"] = True
        
        for cell in list(self.FrontierOpen):
                if cell == p:
                    marked_as["FrontierOpen"] = True
        
        for cell in list(self.FroniterClose):
                if cell == p:
                    marked_as["FrontierClose"] = True
        
        return marked_as
    
    def getAllAdjacentCells(self, p):
        # returns a list of adjacent cells 
        AdjacentCells = []
        AdjacentCells.append((p[0] - 1, p[1] - 1))
        AdjacentCells.append((p[0] - 0, p[1] - 1))
        AdjacentCells.append((p[0] + 1, p[1] - 1))
        AdjacentCells.append((p[0] + 1, p[1] - 0))
        AdjacentCells.append((p[0] + 1, p[1] + 1))
        AdjacentCells.append((p[0] + 0, p[1] + 1))
        AdjacentCells.append((p[0] - 1, p[1] + 1))
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
        
