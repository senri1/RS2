# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot mtion.
import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
import time
import math

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)

        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        # self.time_activate = threading.Timer(5.0,self.computeEntropy)
        # self.time_activate.start()

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()
        if time.time() - self.time_count > 5:
             print('5 seconds')
             self.computeEntropy()
             self.time_count = time.time()
        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()
        count = 0
        # Check to see if there any waypoints is an obstacle
        for i in range(len(self.currentPlannedPath.waypoints)):
            x = self.currentPlannedPath.waypoints[i].coords[0]
            y = self.currentPlannedPath.waypoints[i].coords[1]
            if self.occupancyGrid.getCell(x,y) == 1:
                count +=1
        # print('coords',self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(self.currentPlannedPath.waypoints[-1].coords))
        # print('coords',self.currentPlannedPath.waypoints[-1].coords[0],self.currentPlannedPath.waypoints[-1].coords[1])
        # print('coords',self.currentPlannedPath.waypoints[-1].label)
        if count > 0:
            self.controller.stopDrivingToCurrentGoal()
        pass
    
    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))
        # print('goal_coord',self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(goalCellCoords))
        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        planner_count = 0
        goalReached = False
        
        while (goalReached is False) & (rospy.is_shutdown() is False):

            # Set the start conditions to the current position of the robot
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)
            planner_count += 1
            self.gridUpdateLock.release()

            # If we can't reach the goal, give up and return
            if pathToGoalFound is False:
                rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                              goalCellCoords[0], goalCellCoords[1])
                print('Performance Rating = ', planner_count)
                return False
            
            # Extract the path
            # pastPlannedPath = self.currentPlannedPath
            self.currentPlannedPath = self.planner.extractPathToGoal()
            # if (pastPlannedPath == self.currentPlannedPath) or (self.goal_unreachable == True):
            #     print('Goal not Reachable')
            #     return(False)

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)
        print('Performance Rating = '+ str(planner_count))
        return goalReached


    def checkIfCellIsUnknown(self, x, y):
        newX = x 
        newY = y
        return (newX >= 0) & (newX < self.occupancyGrid.getWidthInCells()) \
            & (newY >= 0) & (newY < self.occupancyGrid.getHeightInCells()) \
            & (self.occupancyGrid.getCell(newX, newY) == 0.5)
                    
    def computeEntropy(self):
        entropy=0
        unknownCells = 0
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.checkIfCellIsUnknown(x, y) == True:
                    unknownCells += 1
        
        pCMap = math.log(2)* abs(unknownCells)

        #for every realisation map???
        entropy -= pCMap * math.log(pCMap)
        print('Saving entropy data')
        #save data
        dir = "/home/ros_user/catkin_ws_cw2/data/entropy_data"
        with open(dir,'a') as myfile:
            wr = csv.writer(myfile,quoting=csv.QUOTE_ALL)
            wr.writerow(entropy)
     

    
