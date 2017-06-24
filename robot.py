import numpy as np
from operator import add, sub
import heapq
#import collections
import random
import sys
import time

dir_sensors =  {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                     'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                     'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                     'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_reverse =  {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
                     'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
dir_move =     {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
dir_rotation = {'u': {'l':-1, 'u':0, 'r':1}, 'r': {'u':-1, 'r':0, 'd':1},
                    'd': {'r':-1, 'd':0, 'l':1}, 'l': {'d':-1, 'l':0, 'u':1},
                    'up': {'l':-1, 'u':0, 'r':1}, 'right': {'u':-1, 'r':0, 'd':1},
                    'down': {'r':-1, 'd':0, 'l':1}, 'left': {'d':-1, 'l':0, 'u':1}}
rotation_index = [-1, 0, 1]
        
class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.start = time.clock()
        self.location = self.start = (0, 0)
        self.prev_location = ()
        self.heading = 'u'
        self.maze_dim = maze_dim
        self.steps = 0
        self.count = 0
        self.index = 0
        self.exploring = True  #Flag to switch from exploring to exploiting
        self.planning = False  #Flag to allow a_star_search to run prior to second run
        self.goal = False      #Flag to indicate that the goal door was found
        self.mapped = False    #Flag to indicate that the maze has been fully explored
        self.rotate_cost = 2
        self.no_rotate_cost = 1
        self.goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.goal_door_location = None
        self.maze_map = {}
        self.init_maze_map() #Initialize the maze_map with the each cell as a key
                             # and the outer wall values.
        '''{(0, 0): {'d': 0, 'l': 0, 'r': 0, 'u': 11},
            (0, 1): {'d': 1, 'l': 0, 'r': 0, 'u': 10},
            (0, 2): {'d': 2, 'l': 0, 'r': 3, 'u': 9}}
           
           The self.maze_map dict will look like above after 3 'up' moves from (0,0),
           for test_maze_01.txt
        '''
        self.dead_end = False  #Flag to indicate whether to proceed with dead end back out mode.
        self.dead_end_set = set()   #Set of cells that lead into dead ends and dead end corridors.
        
        try:
            self.reset_threshold = int(sys.argv[2]) #Check if user entered a reset threshold
        except:
            self.reset_threshold = (self.maze_dim + 2)**2 #If reset threshold not entered, use this default
        
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.
        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.
        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        #Track total number of steps robot takes
        self.steps += 1
        self.sensors = np.array(sensors)
        
        if self.exploring:
            rotation, movement = self.explore()    #First run exploration mode to find goal and map maze
        else:
            rotation, movement = self.exploit()    #Second run exploitation mode to race to goal

        return rotation, movement

#********************************************************************************************************

### Function used during first run to find the goal and map the maze
###

    def explore(self):
        if self.goal and (self.mapped or self.steps >= self.reset_threshold):
            self.planning = True
            rotation,movement = ('Reset','Reset')
            self.exploring = False
            print '\nNumber of walls mapped per cell:'
            self.print_map1(self.maze_map)
            print '\nPercent of cell walls mapped is {:.2%}.'.format(self.is_mapped())
            print 'Total number of moves for 1st run is {}.\n'.format(self.steps)
            return rotation,movement
        else:
            #Update the maze map with the current position
            #self.update_maze_map1()
            self.update_maze_map2()
            self.is_mapped()
            #rotation,movement = self.random_explore()
            #rotation,movement = self.avoid_dead_ends_explore()
            rotation,movement = self.smart_map_explore()
            self.update_position(rotation,movement)
            if self.location == self.goal_door_location:
                self.print_map1(self.maze_map)
                print '\nFound Goal in {} moves.'.format(self.steps)
                print 'Percent of cell walls mapped is {:.0f}%.'.format(self.is_mapped())
            return rotation*90,movement

#********************************************************************************************************
#********************************************************************************************************

### Function used during the second run to exploit the mapped maze to find the fastest path to the goal
###

    def exploit(self):
        #To begin relocate robot to start, heading up and compute the best route to goal_door
        if self.planning:
            self.location = self.start
            self.heading = 'u'
            self.goal_route = []
            #self.goal_route = self.a_star_search()
            self.goal_route = self.d_star_lite_search()
            self.action_list = []
            self.action_list.append('S')
            self.planning = False
        
        #Look through each set of three steps in goal_route from current location to see how many of them are in
        #the same direction. This number will be the movement.
        movement = 0
        heading = [self.heading] #Initialize list with current heading to compare to the next three headings in goal_route
        if self.location != self.goal_door_location:
            for step in range(3):
                #Return the heading key in dir_move whose value is equal to the difference between each pair of successive
                #map locations in goal_route
                heading.append([k for k,v in dir_move.iteritems() \
                        if v == map(sub,self.goal_route[self.index+step+1],self.goal_route[self.index+step])][0])
                #heading now contains the next 3 turns. Increment through the list, for each one that is that same as
                #the current heading, add 1 to movement
                if heading[step] == heading[step+1]:
                    movement += 1
                else:
                    break
            if movement > 1: 
                rotation = 0
            else: 
                movement = 1
                rotation = dir_rotation[heading[0]][heading[1]]
            
            self.update_position(rotation,movement)   
            self.count += 1
            self.index += movement
            
        #Update route_map of rotations and movements for display
        self.route_map(rotation,movement)
        
        if self.location == self.goal_door_location:
            print 'Total length of route is {} grid cells.'.format(len(self.goal_route))
            print 'Total movements to goal = {}.'.format(self.count)
            end = time.clock()
            print 'Total compute time is {:.4} seconds.'.format(end - self.start[0])

        return rotation*90,movement
    
#********************************************************************************************************

### The following helper functions are used during initialization and as aids to explore()

### Function used during the second run to exploit the mapped maze to find the fastest path to the goal
###

    def init_maze_map(self):
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                self.maze_map[(i,j)] = {}
                if i == 0:
                    self.maze_map[(i,j)].update({'l':0})
                if i == self.maze_dim-1:
                    self.maze_map[(i,j)].update({'r':0})
                if j == 0:
                    self.maze_map[(i,j)].update({'d':0})
                if j == self.maze_dim-1:
                    self.maze_map[(i,j)].update({'u':0})

    def is_mapped(self):
        walls = []
        #Loop through maze_map and count number of walls that have been mapped
        for i in self.maze_map:
            walls.append(len(self.maze_map[i]))
        wallnum = float(sum(walls))
        #If they've all been mapped set mapped to True
        if wallnum == 4*(self.maze_dim**2):
            self.mapped = True
        #Track percentage of walls mapped to help in analysis
        #Want to determine minimum percentage of map necessary to achieve best scores
        percent = wallnum/(4*(self.maze_dim**2))
        return percent

    def print_map1(self,pmap):
        grid = []
        for i in reversed(range(self.maze_dim)):
            gridrow = []
            for j in range(self.maze_dim):
                if len(pmap[(j,i)]) > 0:
                    gridrow.append(len(pmap[(j,i)]))
                else:
                    gridrow.append('-')
            grid.append(gridrow)
        for i in range(len(grid)):
            print '[%s]'%'  '.join(map(str,grid[i]))
        
    def goal_door(self):
        if self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds:
            goal_door_location = self.location
            self.goal = True
        else:
            goal_door_location = None
        return goal_door_location
    
    def update_position(self, rot, mov):
        #print self.location, self.heading, rot, mov
        #Assign location to prev_location before updating
        self.prev_location = self.location
        # find the new heading after rotating from the current heading
        self.heading = [k for k,v in dir_rotation[self.heading].iteritems() if v == rot][0]
        # apply movment
        self.location = tuple(map(add,self.location,[i*mov for i in dir_move[self.heading]]))
        #if goal_door hasn't been found yet, check if this new position is the goal_door
        if not self.goal:
            self.goal_door_location = self.goal_door()
    
    #For random exploration mapping that doesn't update any map cell that hasn't been visited by robot
    def update_maze_map1(self):
        #update map with current location and wall sensor values
        self.maze_map[self.location].update(dict((h,i) for h,i in zip(dir_sensors[self.heading],self.sensors)))
        #add to this dictionary the distance to wall value for the previous cell that the robot just came from
        #the exception is cell (0,0) at the start - it has the boundary wall behind it.
        if len(self.maze_map[self.location]) < 4:
            if self.location == self.start:
                self.maze_map[tuple(self.location)].update({dir_reverse[self.heading]:0})
                #print self.start, self.maze_map[self.start], self.steps
            else:
                #Create a new dictionary entry for the reverse heading distance to wall, then add it to maze_map at current location
                back_sensor = {dir_reverse[self.heading]:self.maze_map[self.prev_location].get(dir_reverse[self.heading]) + 1}
                self.maze_map[self.location].update(back_sensor)
                #print self.maze_map[self.location]
    
    #For smart_map exploration where sensor values are used to update map cells that haven't been directly visited by robot
    def update_maze_map2(self):
        #Update the current location with the distances to the walls in each direction
        self.maze_map[self.location].update(dict((h,i) for h,i in zip(dir_sensors[self.heading],self.sensors)))
        #For each sensor value greater than zero, get the next cell in that direction, add the distance to the wall for that cell.
        #Repeat this for each subsequent cell, until the wall is reached.
        for h,s in zip(dir_sensors[self.heading],self.sensors):
            if s != 0:
                for i in range(1,s+1):
                    next_cell1 = tuple(map(add,self.location,[i*e for e in dir_move[h]]))
                    self.maze_map[next_cell1].setdefault(h,(s-i))
                    self.maze_map[next_cell1].setdefault(dir_reverse[h],i)
                    #When the cell with the wall is finally reached, update the adjacent cell in that direction with this wall
                    #location relative to its location.
                    if i == (s+1):
                        next_cell2 = tuple(map(add,self.location,[(i+1)*e for e in dir_move[h]]))
                        if next_cell2 in self.maze_map:
                            self.maze_map[next_cell2].setdefault(dir_reverse[h],0)
            #For every cell wall adjacent to the current cell, the next neighbor cell in that direction will also share that wall.
            #Update that cell with this wall location relative to its location.
            else:
                next_cell3 = tuple(map(add,self.location,dir_move[h]))
                if next_cell3 in self.maze_map:
                    self.maze_map[next_cell3].setdefault(dir_reverse[h],0)
    
    #Function to check for cells that don't lead to a dead end
    def dead_ends(self):
        open_dirs = [] #Directions available to move from current location
        neighbor_cells = [] #Neighbor cells in the open_dirs list of available headings
        no_dead_ends = [] #Sensor list indices that won't lead to a dead end
        open_cells = [] #List of open cells that have no dead ends
        #Check for available neighbor locations to current location in maze_map that are open
        open_dirs = [k for k,v in self.maze_map[self.location].iteritems() \
                     if k != dir_reverse[self.heading] and v != 0]
        for i in open_dirs:
            neighbor_cells = map(add,self.location,dir_move[i])
            if (tuple(neighbor_cells) in self.maze_map) and (tuple(neighbor_cells) not in self.dead_end_set):
                no_dead_ends.append(dir_sensors[self.heading].index(i))
                open_cells.append(tuple(neighbor_cells))
        return no_dead_ends,open_cells
    
    #Purely random mapping only to use as a baseline for the poorest possible performance
    def random_explore(self):
        #randomly choose the index of a direction in the the sensor array
        if np.count_nonzero(self.sensors) >= 1:
            rot_ind = random.choice(np.nonzero(self.sensors)[0])
            #Rotate and move
            rotation = rotation_index[rot_ind]
            movement = 1
        else:
            #If sensors all read 0, robot is in a dead end
            movement = 0
            rotation = 1
        
        return rotation, movement
    
    #Random selection of available openings while trying to avoid dead ends
    #Tries to improve on random_explore
    def avoid_dead_ends_explore(self):
        #if there are more than 1 open directions to choose, pick one at random
        if np.count_nonzero(self.sensors) > 1:
            #If robot was in a dead end corridor at last location, it is now out. Set self.dead_end to false.
            if self.dead_end:
                self.dead_end = False
            #Check if any available openings lead to dead end and return those that aren't
            no_dead_ends,open_cells = self.dead_ends()
            #Pick one of the sensor list indices at random
            rot_ind = random.choice(no_dead_ends)
            #Rotate and move
            rotation = rotation_index[rot_ind]
            movement = 1
        #If there's only 1 direction to move, just move there. This is also used to help indentify dead-end
        #corridors.
        elif np.count_nonzero(self.sensors) == 1:
            if self.dead_end:
                self.dead_end_set.add(self.location)
            rot_ind = np.flatnonzero(self.sensors).item()
            rotation = rotation_index[rot_ind]
            movement = 1
        else:
            movement = 0
            rotation = 1
            #If sensors all read 0 and not back at start, robot is in a dead end
            if self.location != self.start:
                self.dead_end = True
                self.dead_end_set.add(self.location)
                
        #This is to prevent the robot from continuing to explore the goal room once it's found.
        if self.goal and (self.location == self.goal_door_location):
            movement = -1
            rotation = 0
            self.dead_end = True
            self.dead_end_set.add(self.location)
        
        return rotation,movement
    
    #This exploring function is meant to employ the faster mapping function as well as improved
    #mapping strategy that will try to get to the goal as quickly as possible, then continue
    #mapping until the reset threshold is met.
    def smart_map_explore(self):
        #if there are more than 1 open directions to choose, pick one at random
        if np.count_nonzero(self.sensors) > 1:
            #If robot was in a dead end corridor at last location, it is now out. Set self.dead_end to false.
            if self.dead_end:
                self.dead_end = False
            #Check if any available openings lead to dead end and return those that aren't
            no_dead_ends,open_cells = self.dead_ends()
            #If there's more than one choice, choose the direction with the least mapped number of walls.
            #If they're equal, pick one at random.
            if len(no_dead_ends) > 1:
                oc = []
                [heapq.heappush(oc, (len(self.maze_map[o]), o)) for o in open_cells]
                if oc[0][0] == oc[1][0]:    #If the first two elements in the heapq list have equal priortities 
                    next_cell = random.choice(oc)[1]    #pick one at random.
                else:
                    next_cell = heapq.heappop(oc)[1]
                next_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,self.location)][0]
                rotation = dir_rotation[self.heading][next_heading]
                movement = 1
            else:
                rotation = rotation_index[no_dead_ends[0]]
                movement = 1
                
        #If there's only 1 direction to move, just move there. This is also used to help indentify dead-end
        #corridors.
        elif np.count_nonzero(self.sensors) == 1:
            if self.dead_end:
                self.dead_end_set.add(self.location)
            rot_ind = np.flatnonzero(self.sensors).item()
            rotation = rotation_index[rot_ind]
            movement = 1
        else:
            movement = 0
            rotation = 1
            #If sensors all read 0 and not back at start, robot is in a dead end
            if self.location != self.start:
                self.dead_end = True
                self.dead_end_set.add(self.location)
        
        #This is to prevent the robot from continuing to explore the goal room once it's found.
        if self.goal and (self.location == self.goal_door_location):
            movement = -1
            rotation = 0
            self.dead_end = True
            self.dead_end_set.add(self.location)
            
        return rotation,movement
    
    #Function to find goal door as quickly as possibly, then find pursue unexplored route back to start
    def goal_seek_explore(self):
        
        
        return rotation,movement

#********************************************************************************************************
#********************************************************************************************************

### These following helper functions are used as to aid the exploit() during the second run.
###

    #Function to print out a dictionary as an array with start (0,0) in the lower left corner
    def print_grid(self,grid):
        printgrid = []
        for i in reversed(range(self.maze_dim)):
            gridrow = []
            for j in range(self.maze_dim):
                gridrow.append(grid.get((j,i),'-'))
            printgrid.append(gridrow)
        for i in range(len(printgrid)):
            print '[%s]'%'  '.join(map(str,printgrid[i]))
        
    def neighbors(self, cell):
        open_dirs = []
        neighbors = []
        #Check for available neighbor locations that are open
        open_dirs = [k for k,v in self.maze_map[cell].iteritems() if v != 0]
        #print self.maze_map[cell]
        neighbors = [tuple(map(add,cell,dir_move[i])) for i in open_dirs]
        #print neighbors
        
        return neighbors

    def cost(self, prev_cell, cell, next_cell):
        #get current cell heading based on previous cell
        if cell != self.start:
            heading = [k for k,v in dir_move.iteritems() if v == map(sub,cell,prev_cell)][0]
        #get heading necessary to move into the next cell
        next_cell_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,cell)][0]
        #if these headings are equal, get cost for going straight
        if cell == self.start:
            cost = self.no_rotate_cost
        elif heading == next_cell_heading:
            cost = self.no_rotate_cost
        #if the headings are not the same, get cost for turning
        else:
            cost = self.rotate_cost
        return cost
        
    def heuristic(self, a, b):
        #Computes the Manhattan distance between a and b.
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)


### NOTE: This implementation of A* search was adapted from Amit Patel's Red Blob Games website: http://www.redblobgames.com/
###

    def a_star_search(self):
        open = []
        heapq.heappush(open, (0, self.start))
        came_from = {}
        cost_so_far = {}
        came_from[self.start] = None #Every key in came_from is a cell in the maze and 
                                     #has for its value the previous cell the robot was in.
        cost_so_far[self.start] = 0  #Every key in cost_so_far is a cell in the maze and
                                     #has for its value the number of steps to get to that cell
                                     #from self.start (0,0) plus the cost of moving to this same
                                     #cell from the previous cell.
                    
        goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
                
        found = False  # flag that is set when goal is found
        resign = False # flag set if robot can't find goal
        count = 0
        
        #This loop tracks the cost to get from any cell to the goal
        while not found and not resign:
            if len(open) == 0:
                resign = True
                print "Failed to find route to goal."
            else:
                count += 1
                current = heapq.heappop(open)[1]
                
                if current == self.goal_door_location:
                    found = True
                #else:
                elif current in self.maze_map:
                    #print 'test1'
                    for next_cell in self.neighbors(current):
                        # f = g + h: typical format for cost function for A* search
                        #Calculate the cost for moving from the current cell to any of the available neighhbors.
                        #The cost for moving to a neighbor in the same direction will be 1 and that for moving
                        #to a neighbor requiring a rotation will be 2.
                        #To calculate the current heading to determine whether a turn is needed we need the
                        #previous cell from which the robot moved to the current cell. This can be found in came_from.
                        move_cost = cost_so_far[current] + self.cost(came_from[current],current, next_cell)
                        #if next not in cost_so_far or new_cost < cost_so_far[next]:
                        if move_cost < cost_so_far.get(next_cell, float("inf")):
                            cost_so_far[next_cell] = move_cost
                            priority = move_cost + self.heuristic(self.goal_door_location, next_cell)
                            heapq.heappush(open,(priority, next_cell))
                            came_from[next_cell] = current
        
        #Once costs have all been calculated, we can now unpack the path to get from S to G
        current1 = self.goal_door_location
        path = [current1]
        while current1 != self.start:
            current1 = came_from[current1]
            path.append(current1)
        path.reverse()
        
        return path
    
    def d_star_lite_search(self):
        value = {}          #Every key in value is one the orientations - u, r, d, l - that have each cell as values.
                            #Each cell is also a key that contains the cost value for that cell in that orientation.
                
        policy = {}         #Every key in policy is one the orientations - u, r, d, l - that have each cell as values.
                            #Each cell is also a key that contains the policy action - L, #, R - for that cell in
                            #that orientation.
                
        path = []
        route = {}
        cost = [2, 1, 2]    #Cost for left turn, no turn, right turn
        action_name = ['L', '#', 'R']    #Symbols for left turn, no turn, right turn
        
        for i in range(self.maze_dim):      
            for j in range(self.maze_dim):
                value[(i,j)] = {'u':99,'r':99,'d':99,'l':99}
                policy[(i,j)] = {'u':99,'r':99,'d':99,'l':99}
        
        change = True
        while change:
            change = False
            
            for cell in self.maze_map:
                for orientation in ['u','r','d','l']:
                    if cell == self.goal_door_location:
                        if value[cell][orientation] > 0:
                            value[cell][orientation] = 0
                            policy[cell][orientation] = 'G'
                            change = True
                            
                    else:
                        for i in range(3):
                            o2 = dir_sensors[orientation][i]
                            c2 = tuple(map(add,cell,dir_move[o2]))
                            if o2 in self.maze_map[cell]:
                                if c2 in self.maze_map and self.maze_map[cell][o2] != 0:
                                    v2 = value[c2][o2] + cost[i]
                                    if v2 < value[cell][orientation]:
                                        change = True
                                        value[cell][orientation] = v2
                                        policy[cell][orientation] = action_name[i]
        
        #Print functions to visualize value and policy for debugging and understanding
        #how the algorithm works.
        #print_map2(value)
        #print_map2(policy)
            
        cell = self.start
        orientation = 'u'
        route[cell] = policy[cell][orientation]
        path = [cell]
        
        while policy[cell][orientation] != 'G':
            if policy[cell][orientation] == '#':
                o2 = orientation
            elif policy[cell][orientation] == 'R':
                o2 = dir_sensors[orientation][2]
            elif policy[cell][orientation] == 'L':
                o2 = dir_sensors[orientation][0]
            cell = tuple(map(add,cell,dir_move[o2]))
            orientation = o2
            route[cell] = policy[cell][orientation]
            path.append(cell)
        
        return path
    
    #Function to create a list of moves from start (S) to goal (G) using '#' for no turn, 'L' for left turn, 'R' for right turn
    def route_map(self, rot, mov):
        rmap = {}
        if rot == 0:
            for m in range(mov):
                self.action_list.append('#')
        elif rot == 1:
            self.action_list.pop()
            self.action_list.append('R')
            self.action_list.append('#')
        elif rot == -1:
            self.action_list.pop()
            self.action_list.append('L')
            self.action_list.append('#')
            
        if self.location == self.goal_door_location:
            self.action_list.pop()
            self.action_list.append('G')
            for i in range(len(self.goal_route)):
                rmap[self.goal_route[i]] = self.action_list[i]
            self.print_grid(rmap)
        #return rmap
    
    #Function visualize value and policy dictionaries in D* Lite
    def print_map2(self,vmap):
        grids = {'u':{},'r':{},'d':{},'l':{}}
        for orientation in grids:
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    grids[orientation][(i,j)] = vmap[(i,j)][orientation]
    
        for orientation in grids:
            print orientation
            gridmap = []
            for j in reversed(range(maze_dim)):
                gridrow = []
                for i in range(maze_dim):
                    gridrow.append(grids[orientation].get((i,j)))
                gridmap.append(gridrow)
            for i in range(len(gridmap)):
                print '[%s]'%' '.join(map(str,[format(el,'^2') for el in gridmap[i]]))