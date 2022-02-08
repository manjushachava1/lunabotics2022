#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from priority_queue import PriorityQueue #importing PriorityQueue class to be used
from copy import deepcopy


class PathPlanner:
    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner", anonymous = False)
        
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.serv = rospy.Service('/plan_path', GetPlan, self.plan_path)
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.pubCSpace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size = 10)
        self.pubObstacle = rospy.Publisher('/path_planner/obs', GridCells, queue_size = 10)
        self.pubWaveFront = rospy.Publisher('/path_planner/wave', GridCells, queue_size = 10)
        self.pubPath = rospy.Publisher('/path_planner/path_home', Path, queue_size = 10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.pubAStar = rospy.Publisher('/path_planner/a_star_planning', GridCells, queue_size = 10)
        
        ## Initialize the request counter
        self.requestCounter = 0
        
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        index = y * mapdata.info.width + x
        return index
       


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        return math.sqrt((x2-x1)**2+(y2-y1)**2) #Using Pythagorian Thm to find the total travelled distance
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        #XyCoordinates to be returned
        xyCoord = Point()

        #Assign coord values from grid values
        xyCoord.x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        xyCoord.y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y

        return xyCoord
        



    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        try:
            mapPos = mapdata.info.origin.position       #location of pos data in mapdata variable
            mapRes = mapdata.info.resolution            #location of map resolution in variable 
            xPos = int((wp.x - mapPos.x) / mapRes)      #Eq taken from lab document
            yPos = int((wp.y - mapPos.y) / mapRes)
            output = (xPos, yPos)   #Create coordinate for function output
            return output
        except Exception as e:
            print(e)
            print('Failed on world_to_grid()')


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """

        PoseStampedList = []

        for pathI in range(len(path)):
            yaw = 0

            if pathI < len(path) - 1:
                #Calculate distance to final pose
                currPoint = path[pathI]
                nxtPoint = path[pathI + 1]

                dX = nxtPoint[0] - currPoint[0]
                dY = nxtPoint[1] - currPoint[1]

                #Calculate initial turn angle 
                angToDest = math.atan2(dY,dX)
                #angleToTurnTo = self.

            
            #XYZ and QuatStuff
            xyPos = PathPlanner.grid_to_world(mapdata,path[pathI][0],path[pathI][1])
            quatArray = quaternion_from_euler(0,0,angToDest)
            quatObj = Quaternion(*quatArray)

            #generate and populate a pose
            aPose = PoseStamped()
            aPose.pose.position = xyPos
            aPose.pose.orientation = quatObj
            aPose.header = mapdata.header
            PoseStampedList.append(aPose)

        return PoseStampedList


    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        xLim = mapdata.info.width -1
        yLim = mapdata.info.height -1
        
        xRange = range(0,xLim)
        yRange = range(0,yLim)
        
        freeThreshold = 10

        if(x in xRange and y in yRange):
            index = PathPlanner.grid_to_index(mapdata,x,y)
            if mapdata.data[index] < freeThreshold:
                return True
        
        return False

               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        
        #if the input values are greater than the mapdata, or less than 0, then
        # an exception is thrown
        if (x<0 or x> mapdata.info.width-1 or y<0 or y>mapdata.info.height-1):
            raise ValueError("Out of Bounds!")

        availibleSpaces = []

        #If x is not the value next to the boarder
        if (x!=mapdata.info.width-1):
            #Check is cell is walkable
            if (PathPlanner.is_cell_walkable(mapdata, x+1, y)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x+1,y))

        #If the x val is not the 0 boundary
        if (x!=0):
            #Check is cell is walkable
            if(PathPlanner.is_cell_walkable(mapdata, x-1, y)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x-1,y))
        
        #If y is not the value next to the boarder
        if (y!=mapdata.info.height-1):
            #Check is cell is walkable
            if (PathPlanner.is_cell_walkable(mapdata, x, y+1)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x,y+1))

        #If the y val is not the 0 boundary
        if (y!=0):
            #Check is cell is walkable
            if(PathPlanner.is_cell_walkable(mapdata, x, y-1)):
                #If cell can be reached, add it to the list of avaible spaces
                availibleSpaces.append((x,y-1))

        return availibleSpaces
        
    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT

        availibleSpaces = PathPlanner.neighbors_of_4(mapdata, x, y)

        
        if(x!=0 and y!=0):
            if(PathPlanner.is_cell_walkable(mapdata, x-1,y-1)):
                availibleSpaces.append((x-1,y-1))

        if(x!=mapdata.info.width-1 and y!=mapdata.info.height-1):
            if(PathPlanner.is_cell_walkable(mapdata, x+1,y+1)):
                availibleSpaces.append((x+1,y+1))

        if(x!=mapdata.info.width-1 and y!=0):
            if(PathPlanner.is_cell_walkable(mapdata, x+1,y-1)):
                availibleSpaces.append((x+1,y-1))

        if(x!=0 and y!=mapdata.info.height-1):
            if(PathPlanner.is_cell_walkable(mapdata, x-1,y+1)):
                availibleSpaces.append((x-1,y+1))

        return availibleSpaces
 
    
    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")     #log info
        try:
            mapServer = rospy.ServiceProxy('static_map', GetMap)    #Request data from the map server
            return mapServer().map    #Get the map parameter from the mapServer object
        except Exception as e:
            print(e)
            print('Failed on world_to_grid()')
            return None
        



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        try:
            THRESH = 50                     #Threshold for shading of a cell
            newMap = list(mapdata.data)     #Create a copy of existing map to expand obstacles with. Make list so its changeable
            worldCoordinates = []           #Initialize world coordinate list of obstacles

            rospy.loginfo("Calculating C-Space")

            ## Determine cspace for each layer of padding
            for i in range(padding):
                #print(i)
                ## Go through each cell in the occupancy grid (range used to start on row/col 0)
                for y in range(mapdata.info.height):
                    for x in range(mapdata.info.width):
                        ## Inflate the obstacles where necessary
                        if mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)] >= THRESH: 
                            newMap[PathPlanner.grid_to_index(mapdata, x, y)] = 100          #Set to 100 to make it 100% an obstacle
                            neighbors = PathPlanner.neighbors_of_8(mapdata, x, y)           #Get all walkable cells that neighbor main cell
                            for each in neighbors:
                                newMap[PathPlanner.grid_to_index(mapdata, each[0], each[1])] = 100  #Set cell to an obstacle in the map copy

                print('Found all the cspace for padding layer ' + str(i+1) + ' out of ' + str(padding))
                mapdata.data = deepcopy(newMap)   #Set the mapdata to the new map for use in recursion. 
            
            ## Convert cspace coordinates to world coordinates
            for y in range(mapdata.info.height):
                for x in range(mapdata.info.width):
                    if newMap[self.grid_to_index(mapdata, x, y)] >= THRESH:     #If an obstacle is detected
                        worldPoint = self.grid_to_world(mapdata, x, y)          #Make a worldpoint out of it
                        worldCoordinates.append(worldPoint)                     #append to list in order
            
            ## Create a GridCells message and publish it
            msg = GridCells()                               #Create GridCells Object
            msg.cell_height = mapdata.info.resolution       #dims are equal to map resolution
            msg.cell_width = mapdata.info.resolution
            msg.cells = worldCoordinates                    #Set cell data to the world coordinates of obstacles
            msg.header.frame_id = mapdata.header.frame_id   #Copy over frame id
            self.pubCSpace.publish(msg)                     #Publish to topic

            ## Return the C-space
            return mapdata
        except Exception as e:
            return None
            print('Failed on calc_cspace()')
            print(e)

    def a_star(self, mapdata, start, goal):
        """
        Using A* algorithm to calculate an path.
        :param mapdata  [OccupancyGrid] The map data.
        :param start    [(int, int)]    The initial point (a grid coordinate).
        :param goal     [(int, int)]    The goal point (a grid coordinate).
        :return         [[(int, int)]]  The path as a list of tuples (grid coordinates)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        
        #Raise an error if the goal is out bounds, i.e. the goal is an object or in c-space
        if PathPlanner.is_cell_walkable(mapdata, goal[0],goal[1]) is False:
            raise ValueError("Goal is Out of Bounds!")

        #Raise an error is the start is out bounds, i.e. the start is an object or in c-space
        if PathPlanner.is_cell_walkable(mapdata, goal[0],goal[1]) is False:
            raise ValueError("Start is Out of Bounds!")
        
        #creating a frontier to use the priority queue class to follow the sudo code
        mapFrontier = PriorityQueue()
        #place start position at the start of the point given
        mapFrontier.put(start,0)

        #Initialize a came_from dict
        came_from = {}
        came_from[start] = None
        #Initialize a cost
        cost= {}
        cost[start] =0 

        ## Create a GridCells message for visited cells
        neighbourViz = GridCells()                           #Create GridCells Object
        neighbourViz.cell_height = mapdata.info.resolution   #dims are equal to map resolution
        neighbourViz.cell_width = mapdata.info.resolution
        listofCells = list(neighbourViz.cells)
        
        ## Create a GridCells message for path cells
        pathCells = GridCells()                           #Create GridCells Object
        pathCells.cell_height = mapdata.info.resolution   #dims are equal to map resolution
        pathCells.cell_width = mapdata.info.resolution
        listOfPathCells = list(pathCells.cells)

        while (mapFrontier.empty() is False):
            #Get the top Priority from the frontier
            topPriority = mapFrontier.get()

            #if neighbor is goal, stop search 
            if (topPriority == goal):
                break

            #generate the 8 neighbors of topPriority
            #for each neighbor:
            for Neighbor in PathPlanner.neighbors_of_8(mapdata, topPriority[0], topPriority[1]):
                gVal = cost[topPriority] #add the topPriority to the cost of where you've been
                #calculate how much it would cost to get to neighbor
                hVal = PathPlanner.euclidean_distance(topPriority[0],topPriority[1],Neighbor[0],Neighbor[1])
                #calculate new total cost
                totalCost = gVal + hVal

                #turnthe neighbor values into a point format
                
                xyCoord = self.grid_to_world(mapdata,Neighbor[0],Neighbor[1])
                

                #listofCells.append(xyCoord) 
                neighbourViz.cells.append(xyCoord)
                
                #print(neighbourViz.cells)
                #print("listofcells" + str(listofCells))
                #Set cell data to the world coordinates of obstacles
                neighbourViz.header.frame_id = mapdata.header.frame_id               #Copy over header
                self.pubWaveFront.publish(neighbourViz)                 #Publish to topic


               
                
                #if the neighbor is not currently in the path travelled, or the total cost of this neighbor
                #less than the previous paths in cost list
                #expand like a spider web
                if Neighbor not in cost or totalCost < cost[Neighbor]:
                    #set the current neighbor to the totalCost
                    cost[Neighbor] = totalCost
                    #recalculate the hVal
                    hVal = PathPlanner.euclidean_distance(Neighbor[0],Neighbor[1],goal[0],goal[1])
                    #recalulate the total cost as a new variable to not override the previous
                    priority = totalCost + hVal
                    #put the neighbor into the priority list based on the new totalCost, aka it's priority
                    mapFrontier.put(Neighbor, priority)
                    #add the node ot the came_from
                    #came_from.update({Neighbour : topPriority})
                    came_from[Neighbor] = topPriority

        #starting at the goal, making that your current position
        currentPos = goal

        #make a list for the path
        path = []
        #add the start to your path
        path.append(goal)

        #print(came_from)


        while currentPos != start:
            currentPos = came_from[currentPos] 
            path.append(currentPos)

        path.reverse()
        deepcopyofpath = deepcopy(path)
        #print(path)
        '''
        for everyLoc in path:
            xyCoordPath = self.grid_to_world(mapdata,everyLoc[0],everyLoc[1])
            pathCells.cells.append(xyCoordPath)
            pathCells.header.frame_id = mapdata.header.frame_id               #Copy over header
            self.pubPath.publish(pathCells)
            rospy.sleep(0.05)      
        '''
        return path

    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        optimizedPath = deepcopy(path)
        for location in range(len(path)-2):

            #if the path is moving at a positive x& y diagonal
            if (path[location][0] == (path[location+1][0]-1) and path[location][0] == (path[location+2][0]-2)) and (path[location][1] == (path[location+1][1]-1) and path[location][1] == (path[location+2][1]-2)):
                optimizedPath.remove(path[location+1])
            #if the path is moving at a positive x & neg y diagonal
            if (path[location][0] == (path[location+1][0]-1) and path[location][0] == (path[location+2][0]-2)) and (path[location][1] == (path[location+1][1]+1) and path[location][1] == (path[location+2][1]+2)):
                optimizedPath.remove(path[location+1])
            #if the path is moving at a neg x & pos y diagonal
            if (path[location][0] == (path[location+1][0]+1) and path[location][0] == (path[location+2][0]+2)) and (path[location][1] == (path[location+1][1]-1) and path[location][1] == (path[location+2][1]-2)):
                optimizedPath.remove(path[location+1])
            #if the path is moving at a neg x & neg y diagonal
            if (path[location][0] == (path[location+1][0]+1) and path[location][0]== (path[location+2][0]+2)) and (path[location][1] == (path[location+1][1]+1) and path[location][1] == (path[location+2][1]+2)):
                optimizedPath.remove(path[location+1])

            #if the path is moving at a positive x direction
            if (path[location][0] == (path[location+1][0]-1) and path[location][0] == (path[location+2][0]-2)) and (path[location][1] == (path[location+1][1]) and path[location][1] == (path[location+2][1])):
                optimizedPath.remove(path[location+1])
            #if the path is moving at a neg x direction
            if (path[location][0] == (path[location+1][0]+1) and path[location][0] == (path[location+2][0]+2)) and (path[location][1] == (path[location+1][1]) and path[location][1]== (path[location+2][1])):
                optimizedPath.remove(path[location+1])

            #if the path is moving at a positive y direction
            if (path[location][0] == (path[location+1][0]) and path[location][0] == (path[location+2][0])) and (path[location][1] == (path[location+1][1]-1) and path[location][1] == (path[location+2][1]-2)):
                optimizedPath.remove(path[location+1])
            #if the path is moving at a neg y direction
            if (path[location][0] == (path[location+1][0]) and path[location][0] == (path[location+2][0])) and (path[location][1] == (path[location+1][1]+1) and path[location][1] == (path[location+2][1]+2)):
                optimizedPath.remove(path[location+1])
        print(path)
        print(optimizedPath)
        return optimizedPath

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        rospy.loginfo("Received path:" + str(path))
        pathMessage = Path()
        pathMessage.poses = self.path_to_poses(mapdata,path)
        pathMessage.header.frame_id = mapdata.header.frame_id
        rospy.loginfo("Returning a Path message")
        self.pubPath.publish(pathMessage)
        return pathMessage


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 3)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)

    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

        
if __name__ == '__main__':
    PathPlanner().run()
