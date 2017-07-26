import numpy as np
from operator import add, sub
import heapq
import random
import sys
import time
import os

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

        self.start_time = time.clock()
        self.location = self.start = (0, 0)
        self.prev_location = ()
        self.heading = 'u'
        self.next_heading = 'r'
        self.maze_dim = maze_dim
        self.steps = 0
        self.mcount = 0
        self.rcount = 0
        self.index = 0
        
        self.exploring = True     #Flag to switch from exploring to exploiting
        self.planning = False     #Flag to allow a_star_search to run prior to second run
        self.goal = False         #Flag to indicate that the goal door was found
        self.mapped = False       #Flag to indicate that the maze has been fully explored
        self.quad_list = [True,False,False,False]    #List of quadrants visited: [1,2,3,4].
        self.loop = False         #Flag to indicate whether robot is in a loop
        self.testing = False
        
        self.pref_heading = []    #Heading preference list for smart mapping strategy
        self.rotate_cost = 3
        self.no_rotate_cost = 1
        self.rotate_count = 0
        self.loop_count = 0
        self.pref_head_count = 0
        self.goal_door_location = None
        self.steps_to_goal = 0
        self.goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.goals = []
        for i in range(maze_dim/2 - 1, maze_dim/2 + 1):
            for j in range(maze_dim/2 - 1, maze_dim/2 + 1):
                self.goals.append((i,j))
        
        self.cost_so_far = {}
        self.maze_map = {}
        self.count_map = {}
        self.rmap = {}
        self.init_maps()     #Initialize the maze_map with the each cell as a key
                             # and the outer wall values.
        '''{(0, 0): {'d': 0, 'l': 0, 'r': 0, 'u': 11},
            (0, 1): {'d': 1, 'l': 0, 'r': 0, 'u': 10},
            (0, 2): {'d': 2, 'l': 0, 'r': 3, 'u': 9}}
           
           The self.maze_map dict will look like above after 3 'up' moves from (0,0),
           for test_maze_01.txt
        '''
        self.dead_end = False  #Flag to indicate whether to proceed with dead end back out mode.
        self.dead_end_set = set()   #Set of cells that lead into dead ends and dead end corridors.
        
        default = (self.maze_dim + 2)**2
            
        try:
            test_report = os.environ['TEST_REPORT']
        except:
            test_report = ''
        if test_report == 'True':
            self.testing = True
        else:
            self.testing = False
        
        #try:
        #    #self.reset_threshold = int(sys.argv[2]) #Check if user entered a reset threshold
        #    movement = os.environ['MOVEMENT_RESET_THRESHOLD']
        #except:
        #    movement = '' #If reset threshold not entered, use this default
        #if isinstance(movement,(int,long)):
        #    if movement < 1000 and movement > 0:
        #        self.movement_reset_threshold = movement
        #        if self.testing: print 'Movement reset threshold set to {}.'.format(movement)
        #else:
        #    self.movement_reset_threshold = default
        #    if self.testing: print 'Movement reset threshold set to default = {}.'.format(default)
            
        try:
            coverage = os.environ['COVERAGE_RESET_THRESHOLD']
        except:
            coverage = ''
        if coverage == 'goal':
            self.coverage_reset_threshold = 'goal'
            if self.testing: print 'Coverage reset threshold set to goal.'
        elif int(coverage) <= 100 and int(coverage) > 0:
            self.coverage_reset_threshold = int(coverage)
            if self.testing: print 'Coverage reset threshold set to {}%.'.format(self.coverage_reset_threshold)
        else:
            self.coverage_reset_threshold = 100
            if self.testing: print 'Coverage reset threshold set to default = 100%.'
            
        try:
            controller_name = os.environ['MAP_STRATEGY']
        except:
            controller_name = ''
        if controller_name == 'random':
            self.controller = self.random_explore
            if self.testing: print 'Maze map strategy used is {}.\n'.format(controller_name)
        elif controller_name == 'avoid dead ends':
            self.controller = self.avoid_dead_ends_explore
            if self.testing: print 'Maze map strategy used is {}.\n'.format(controller_name)
        elif controller_name == 'smart map 1':
            self.controller = self.smart_map_explore1
            if self.testing: print 'Maze map strategy used is {}.\n'.format(controller_name)
        elif controller_name == 'smart map 2':
            self.controller = self.smart_map_explore2
            if self.testing: print 'Maze map strategy used is {}.\n'.format(controller_name)
        else:
            self.controller = self.smart_map_explore2
            if self.testing: print 'Maze map strategy used is smart map 2.\n'
                
        try:
            mapper_name = os.environ['MAPPER']
        except:
            mapper_name = ''
        if mapper_name == 'mapper1':
            self.mapper = self.update_maze_map1
            if self.testing: print 'Mapper method is mapper1.\n'
        else:
            self.mapper = self.update_maze_map2
            if self.testing: print 'Mapper method is mapper2.\n'
            
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


    def explore(self):
        ''' Function used during first run to find the goal and map the maze'''
        
        self.mapper()
        self.update_count_map()
        
        if self.goal and self.mapped:
        #if self.goal and (self.mapped or self.steps >= self.movement_reset_threshold):
            self.planning = True
            rotation,movement = ('Reset','Reset')
            self.exploring = False
            if self.testing:
                self.reporting('exploring')
            
            return rotation,movement
        else:
            rotation,movement = self.controller()
            self.update_position(rotation,movement)
            self.is_mapped(self.coverage_reset_threshold)
            
            #In case algorithm fails to find the goal, get some data back.
            if self.steps == 998 and self.testing:
                self.reporting('fail')
                
            return rotation*90,movement

#********************************************************************************************************

#********************************************************************************************************
### 
###

    def exploit(self):
        '''Function used during the second run to exploit the mapped maze to find the fastest path to the goal'''
        
        #To begin relocate robot to start, (0,0), with heading up and compute the best route to goal_door
        if self.planning:
            self.location = self.start
            self.heading = 'u'
            self.goal_route = []
            self.goal_route = self.a_star_search1(self.start, self.goal_door_location)
            #self.goal_route = self.d_star_lite_search()
            self.action_list = []
            self.action_list.append('S')
            self.planning = False
            #print len(self.goal_route)
            #print self.goal_route
        
        #Get next rotation and movement values.
        rotation,movement = self.route_plan()
        
        #Update route_map of rotations and movements for display
        self.route_map(rotation,movement)
        
        if self.location == self.goal_door_location and self.testing:
            self.reporting('success')
            
        return rotation*90,movement
    
#********************************************************************************************************

### The following helper functions are used during initialization and as aids to explore()

### 
###
    def reporting(self, report):
        if report == 'exploring':
            print '\nFound Goal in {} moves.'.format(self.steps)
            print 'Percent of cell walls mapped until goal found is {:.0%}.'.format(self.is_mapped())
            print '\nNumber of walls mapped per cell:'
            self.print_map1(self.maze_map)
            print '\nPercent of total walls mapped during explore is {:.0%}.'.format(self.is_mapped())
            print 'Total number of moves for 1st run is {}.'.format(self.steps)
            print '\nNumber of times each cell visited:'
            self.print_map3(self.count_map)
            print '\n'
        elif report == 'success':
            print '\nRoute to goal:'
            self.print_grid(self.rmap)
            print '\nTotal length of route is {} grid cells.'.format(len(self.goal_route))
            print 'Total movements to goal is {}.'.format(self.mcount)
            print 'Total rotations to goal is {}.'.format(self.rcount)
            print 'Total compute time is {:.4} seconds.'.format(time.clock() - self.start_time)
        else:
            print '\nNumber of walls mapped per cell:'
            #self.print_map1(self.maze_map)
            print '\nNumber of times each cell visited:'
            self.print_map3(self.count_map)
            print '\nTotal number of loops entered is {}.'.format(self.loop_count)
            print 'Total times Preferred Heading used {}.'.format(self.pref_head_count)
            print 'Rotate count is {}\n'.format(self.rotate_count)
    
    def init_maps(self):
        '''Initialize the maze_map and the count_map'''
        
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                self.maze_map[(i,j)] = {}
                self.count_map[(i,j)] = 0
                if i == 0:
                    self.maze_map[(i,j)].update({'l':0})
                if i == self.maze_dim-1:
                    self.maze_map[(i,j)].update({'r':0})
                if j == 0:
                    self.maze_map[(i,j)].update({'d':0})
                if j == self.maze_dim-1:
                    self.maze_map[(i,j)].update({'u':0})
                    
        return

    def is_mapped(self,level = 100):
        '''Check to see what percentage of the maze walls have been mapped.'''
        
        walls = []
        #Loop through maze_map and count number of walls that have been mapped
        for i in self.maze_map.keys():
            walls.append(len(self.maze_map[i]))
        wallnum = float(sum(walls))
        if level == 'goal':
            if self.goal:
                self.mapped = True
        else:
            threshold = (float(level)/100)*4*(self.maze_dim**2)
            if wallnum >= threshold:
                self.mapped = True
        #Track percentage of walls mapped to help in analysis
        #Want to determine minimum percentage of map necessary to achieve best scores
        percent = wallnum/(4*(self.maze_dim**2))
        return percent

    
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
            
    def print_map1(self,pmap):
        '''Function to print the map dictionary as an array.'''
        
        grid = []
        for i in reversed(range(self.maze_dim)):
            gridrow = []
            for j in range(self.maze_dim):
                #if len(pmap[(j,i)]) > 0:
                #    gridrow.append(len(pmap[(j,i)]))
                #else:
                #    gridrow.append('-')
                gridrow.append(len(pmap.get((j,i),'-')))
            grid.append(gridrow)
        for i in range(len(grid)):
            print '[%s]'%'  '.join(map(str,grid[i]))
        
        return
        
    def print_map2(self,vmap):
        '''Function to visualize value and policy dictionaries in D* Lite'''
        
        grids = {'u':{},'r':{},'d':{},'l':{}}
        for orientation in grids:
            for i in range(self.maze_dim):
                for j in range(self.maze_dim):
                    grids[orientation][(i,j)] = vmap[(i,j)][orientation]
    
        for orientation in grids:
            print orientation
            gridmap = []
            for j in reversed(range(self.maze_dim)):
                gridrow = []
                for i in range(self.maze_dim):
                    gridrow.append(grids[orientation].get((i,j)))
                gridmap.append(gridrow)
            for i in range(len(gridmap)):
                print '[%s]'%' '.join(map(str,[format(el,'^2') for el in gridmap[i]]))
                
        return
                
    def print_map3(self,cmap):
        '''Function to print the count map dictionary as an array.'''
        
        grid = []
        for i in reversed(range(self.maze_dim)):
            gridrow = []
            for j in range(self.maze_dim):
                if (j,i) in cmap:
                    gridrow.append(cmap[(j,i)])
                else:
                    gridrow.append('-')
            grid.append(gridrow)
        for i in range(len(grid)):
            print '[%s]'%' '.join(map(str,[format(el,'>2') for el in grid[i]]))
            #print '[%s]'%'  '.join(map(str,grid[i]))
        return
    
    def goal_door(self):
        '''Function to determine whether the goal door has been found and what its location is.'''
        
        if self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds:
            goal_door_location = self.location
            self.goal = True
        else:
            goal_door_location = None
        return goal_door_location
    
    def update_position(self, rot, mov):
        '''Use the values of the rotation and movement variables to track the robot's position.'''
        
        #Assign location to prev_location before updating
        self.prev_location = self.location
        # find the new heading after rotating from the current heading
        self.heading = [k for k,v in dir_rotation[self.heading].iteritems() if v == rot][0]
        # apply movment
        self.location = tuple(map(add,self.location,[i*mov for i in dir_move[self.heading]]))
        #if goal_door hasn't been found yet, check if this new position is the goal_door
        if not self.goal:
            self.goal_door_location = self.goal_door()
            
        #Keep track of which quadrant the robot is in.
        #Quadrant1
        if self.location[0] < self.maze_dim/2 and self.location[1] < self.maze_dim/2:
            self.quad_list = [True,False,False,False]
        #Quadrant2
        elif self.location[0] >= self.maze_dim/2 and self.location[1] < self.maze_dim/2:
            self.quad_list = [False,True,False,False]
        #Quadrant3
        elif self.location[0] >= self.maze_dim/2 and self.location[1] >= self.maze_dim/2:
            self.quad_list = [False,False,True,False]
        #Quadrant4
        elif self.location[0] < self.maze_dim/2 and self.location[1] >= self.maze_dim/2:
            self.quad_list = [False,False,False,True]
        
        self.map_strategy()
        
        return
    
    def update_maze_map1(self):
        '''For random exploration mapping that doesn't update any map cell that hasn't been visited by robot.'''
        
        #Update map with current location and wall sensor values.
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
                
        return
    
    def update_maze_map2(self):
        '''For smart_map exploration where sensor values are used to update map cells that haven't been directly visited by robot.'''
        
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
                    
        return
    
    def update_count_map(self):
        '''Keep track of the number of times the robot visits each cell.'''
        
        self.count_map[self.location] = self.count_map.get(self.location,0) + 1
        
        return
    
    def dead_ends(self):
        '''Check for cells that don't lead to a dead end.'''
        
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
            no_dead_ends.sort()
        return no_dead_ends,open_cells
    
    def map_strategy(self):
        '''Create a strategy for choosing which of mulitple available cells to move into based on moving
           counter-clockwise around the grid and reaching the boundaries.'''
        
        if not self.goal:
            if self.quad_list[0]:
                self.pref_heading = ['r','d','u','l']
            elif self.quad_list[1]:
                self.pref_heading = ['r','u','d','l']
            elif self.quad_list[2]:
                self.pref_heading = ['d','l','r','u']
            elif self.quad_list[3]:
                self.pref_heading = ['r','d','u','l']
        else:
            if self.quad_list[0]:
                self.pref_heading = ['r','u','d','l']
            elif self.quad_list[1]:
                self.pref_heading = ['r','d','u','l']
            elif self.quad_list[2]:
                self.pref_heading = ['u','l','r','d']
            elif self.quad_list[3]:
                self.pref_heading = ['l','u','d','r']
                
        return
            
    def nearest_unmapped(self):
        '''Search through expanding rings of neighbors of current location to find the nearest unvisited cell.'''
        
        x = self.location[0]
        y = self.location[1]
        cell_list = []
        next_cell = None
        not_found = True
        while not_found:
            for i in range(1, 2*self.maze_dim):
                cell_range = x + i
                for j in range(-i, i + 1):
                    x1 = x + j
                    for k in range(-i, i + 1):
                        y1 = y + k
                        if x1 != x and y1 != y and (x1,y1) in self.maze_map and (x1,y1) not in self.goals:
                            if self.count_map[(x1,y1)] == 0:
                                cell_list.append((x1,y1))
                if len(cell_list) > 1:
                    cq = []
                    [heapq.heappush(cq, (self.heuristic(c,self.location),c)) for c in cell_list]
                    next_cell = heapq.heappop(cq)
                    not_found = False
                elif len(cell_list) == 1:
                    next_cell = cell_list[0]
                    not_found = False
                
        return next_cell
    
    def random_explore(self):
        '''Purely random mapping only to use as a baseline for the poorest possible performance.'''
        
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
    
    def avoid_dead_ends_explore(self):
        '''Random selection of available openings while trying to avoid dead ends. Tries to improve on random_explore.'''
        
        #if there are more than 1 open directions to choose, pick one at random
        if np.count_nonzero(self.sensors) > 1:
            #If robot was in a dead end corridor at last location, it is now out. Set self.dead_end to false.
            if self.dead_end:
                self.dead_end = False
            #Check if any available openings lead to dead end and return those that aren't
            no_dead_ends,open_cells = self.dead_ends()
            #Pick one of the sensor list indices at random
            if len(no_dead_ends) > 0:
                rot_ind = random.choice(no_dead_ends)
                rotation = rotation_index[rot_ind]
                movement = 1
            else:
                movement = 0
                rotation = 1
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
        #Add the goal entrance location to the dead end list while the robot continues exploring.
        if self.goal and (self.location == self.goal_door_location):
            movement = -1
            rotation = 0
            self.dead_end = True
            self.dead_end_set.add(self.location)
        
        return rotation,movement
    
    def smart_map_explore1(self):
        '''This exploring function is meant to employ the faster mapping function as well as improved
           mapping strategy that will try to get to the goal as quickly as possible, then continue
           mapping until a mapping threshold is met, or until the reset threshold is met.'''
        
        #To prevent getting caught in loops, keep a tally of the sum of the rotations.
        #If the sum rises above 4 or below -4, the robot has come full circle.
        #The next time it can choose a different direction it needs to choose the one that maintains
        #or reduces the rotate count.
        #if abs(self.rotate_count) >= 4:
        #    self.loop = True
        #    self.loop_count += 1
        
        next_cell = []
            
        if np.count_nonzero(self.sensors) > 1:
            #If robot was in a dead end corridor at last location, it is now out. Set self.dead_end to false.
            if self.dead_end:
                self.dead_end = False
            #Check if any available openings lead to dead end and return those that aren't
            no_dead_ends,open_cells = self.dead_ends()
            
            #If there's more than one choice, choose the cell that has been least visited.
            #If they're equal and the goal door hasn't been found yet, pick the closest to the goal door.
            #A secondary sorting based on how much of each cell has already been mapped will be applied as tie-
            #breaker for the first.
            #If the cells are still tied, then use the map_strategy.
            if len(open_cells) > 1:
                #Create a new priority queue ranking the available cells according to number of visits
                oc1 = []
                [heapq.heappush(oc1, (self.count_map[o], o)) for o in open_cells]
                #Create another priority queue ranking the cells according to least walls mapped for breaking ties
                oc2 = []
                [heapq.heappush(oc2, (len(self.maze_map[o]), o)) for o in open_cells]
                
                #If self.loop is True, robot should chose the cell that reduces or doesn't change rotate_count.
                if not self.goal:                        #If goal door location not found yet
                    gq1 = []
                    gq2 = []                             #Find the closest goal cell to the current location
                    [heapq.heappush(gq1, (self.heuristic(g, self.location),g)) for g in self.goals]
                    goal = heapq.heappop(gq1)[1]
                    [heapq.heappush(gq2, (self.heuristic(goal,cell[1]),cell[1])) for cell in oc1]
                    
                    if oc2[0][0] < oc2[1][0]:            #First preference is for least mapped.
                        next_cell = oc2[0][1]
                        #print '1'
                    elif oc1[0][0] < oc1[1][0]:          #Second preference is for least visited.
                        next_cell = oc1[0][1]            
                        #print '2'
                    elif gq2[0][0] < gq2[1][0]:          #Third preference is for closest to the goal
                        next_cell = gq2[0][1]            
                        #print '3'
                    else:
                        if len(gq2) == 3 and gq2[0][0] < gq2[2][0]:                  
                            gq2.pop(-1)                  #If none of the above are true and there are 3 cells to choose from
                                                         #If the third cell is further than the first and second cells,
                                                         #Remove it from the list
                        #All else equal, pick at random.
                        next_cell = random.choice(gq2)[1]
                        #print '4'       
                else:
                    if oc2[0][0] < oc2[1][0]:            #First, choose the one least mapped,
                        next_cell = oc2[0][1]          
                        #print '5'
                    elif oc1[0][0] < oc1[1][0]:          #If all mapped, choose the next cell that was least visited
                        next_cell = oc1[0][1]            
                        #print '6' 
                    else:
                        if len(oc1) == 3 and oc1[0][0] < oc1[2][0]:                   
                            oc1.pop(-1)                  #If the first two cells were visited equally and there are 3 to choose from
                                                         #If the third cell is larger than the first,
                                                         #Remove it from the list
                        #If there are 2 or 3 cells all equidistant from the goal,
                        #that are all equally visited and that are equally mapped,
                        #choose one at random.
                        next_cell = random.choice(oc1)
                        #print '7'
                #print next_cell
                next_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,self.location)][0]
                rotation = dir_rotation[self.heading][next_heading]
                movement = 1
                
            elif len(open_cells) == 1:
                rotation = rotation_index[no_dead_ends[0]]
                movement = 1
            else:
                rotation = 0
                movement = -1
                
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
            
        #As for the goal mark the corridor coming out of the start as a dead end as well.
        if self.location == self.start:
            self.dead_end = True
            self.dead_end_set.add(self.location)
        
        self.rotate_count += rotation
        
        return rotation,movement

    def smart_map_explore2(self):
        '''This exploring function is meant to employ the faster mapping function as well as the nearest neighbor
           function to choose which turns to make based on proximity to nearest un/dermapped cell.'''
        
        '''Not implemented yet. For future improvements. This is just a copy of smart_map_explore1 for now.'''
        
        #To prevent getting caught in loops, keep a tally of the sum of the rotations.
        #If the sum rises above 4 or below -4, the robot has come full circle.
        #The next time it can choose a different direction it needs to choose the one that maintains
        #or reduces the rotate count.
        #if abs(self.rotate_count) >= 4:
        #    self.loop = True
        #    self.loop_count += 1
        
        next_cell = []
            
        if np.count_nonzero(self.sensors) > 1:
            #If robot was in a dead end corridor at last location, it is now out. Set self.dead_end to false.
            if self.dead_end:
                self.dead_end = False
            #Check if any available openings lead to dead end and return those that aren't
            no_dead_ends,open_cells = self.dead_ends()
            
            #If there's more than one choice, choose the cell that has been least visited.
            #If they're equal and the goal door hasn't been found yet, pick the closest to the goal door.
            #A secondary sorting based on how much of each cell has already been mapped will be applied as tie-
            #breaker for the first.
            #If the cells are still tied, then use the map_strategy.
            if len(open_cells) > 1:
                #Create a new priority queue ranking the available cells according to number of visits
                oc1 = []
                [heapq.heappush(oc1, (self.count_map[o], o)) for o in open_cells]
                #Create another priority queue ranking the cells according to least walls mapped for breaking ties
                oc2 = []
                [heapq.heappush(oc2, (len(self.maze_map[o]), o)) for o in open_cells]
                
                #If self.loop is True, robot should chose the cell that reduces or doesn't change rotate_count.
                if not self.goal:                        #If goal door location not found yet
                    gq1 = []
                    gq2 = []                             #Find the closest goal cell to the current location
                    [heapq.heappush(gq1, (self.heuristic(g, self.location),g)) for g in self.goals]
                    goal = heapq.heappop(gq1)[1]
                    [heapq.heappush(gq2, (self.heuristic(goal,cell[1]),cell[1])) for cell in oc1]
                    
                    if oc2[0][0] < oc2[1][0]:            #First preference is for least mapped.
                        next_cell = oc2[0][1]
                        #print '1'
                    elif oc1[0][0] < oc1[1][0]:          #Second preference is for least visited.
                        next_cell = oc1[0][1]            
                        #print '3'
                    elif gq2[0][0] < gq2[1][0]:          #Third preference is for closest to the goal
                        next_cell = gq2[0][1]            
                        #print '2'
                    else:
                        if len(gq2) == 3 and gq2[0][0] < gq2[2][0]:                  
                            gq2.pop(-1)                  #If none of the above are true and there are 3 cells to choose from
                                                         #If the third cell is further than the first and second cells,
                                                         #Remove it from the list
                                    
                        #Finally, pick based on map_strategy() prefenences.
                        hq = []                          
                        #for o in oc:                    
                        #    for k,v in dir_move.iteritems():
                        #        if v == map(sub,o[1],self.location):
                        #            heapq.heappush(hq,(self.pref_heading.index(k),o))
                        [heapq.heappush(hq,(self.pref_heading.index(k),o[1]))for o in gq2 for k,v in dir_move.iteritems() \
                         if v == map(sub,o[1],self.location)]
                        next_cell = heapq.heappop(hq)[1]
                        self.pref_head_count += 1
                        #print '4'            
                    #print '3'
                else:
                    if oc2[0][0] < oc2[1][0]:            #First, choose the one least mapped,
                        next_cell = oc2[0][1]          
                        #print '6'
                    elif oc1[0][0] < oc1[1][0]:          #Second, choose least visited
                        next_cell = oc1[0][1]            
                        #print '5' 
                    else:
                        if len(oc1) == 3 and oc1[0][0] < oc1[2][0]:                   
                            oc1.pop(-1)                  #If the first two cells were visited equally and there are 3 to choose from
                                                         #If the third cell is larger than the first,
                                                         #Remove it from the list
                        
                        #Finally, pick based on map_strategy() prefenences.
                        hq = []                          
                        [heapq.heappush(hq,(self.pref_heading.index(k),o[1]))for o in oc1 for k,v in dir_move.iteritems() \
                         if v == map(sub,o[1],self.location)]
                        next_cell = heapq.heappop(hq)[1]
                        self.pref_head_count += 1
                        #print '7'
                next_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,self.location)][0]
                rotation = dir_rotation[self.heading][next_heading]
                movement = 1
                
            elif len(open_cells) == 1:
                rotation = rotation_index[no_dead_ends[0]]
                movement = 1
            else:
                rotation = 0
                movement = -1
                
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
            
        #As for the goal mark the corridor coming out of the start as a dead end as well.
        if self.location == self.start:
            self.dead_end = True
            self.dead_end_set.add(self.location)
        
        self.rotate_count += rotation
        
        return rotation,movement
    
    
#********************************************************************************************************
#********************************************************************************************************

### These following helper functions are used as to aid the exploit() during the second run.
###
        
    def neighbors(self, cell):
        open_dirs = []
        neighbors = []
        #Check for available neighbor locations that are open
        open_dirs = [k for k,v in self.maze_map[cell].iteritems() if v != 0]
        #print self.maze_map[cell]
        neighbors = [tuple(map(add,cell,dir_move[i])) for i in open_dirs]
        #print neighbors
        
        return neighbors

    def cost1(self, prev_cell, cell, next_cell, start):
        #get current cell heading based on previous cell
        if cell != start:
            heading = [k for k,v in dir_move.iteritems() if v == map(sub,cell,prev_cell)][0]
        #get heading necessary to move into the next cell
        next_cell_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,cell)][0]
        #if these headings are equal, get cost for going straight
        if cell == start:
            cost = self.no_rotate_cost
        elif heading == next_cell_heading:
            cost = self.no_rotate_cost
        #if the headings are not the same, get cost for turning
        else:
            cost = self.rotate_cost
        return cost
        
    def cost2(self, prev_cell, cell, next_cell):
        #get current cell heading based on previous cell
        if cell not in self.goals:
            heading = [k for k,v in dir_move.iteritems() if v == map(sub,cell,prev_cell)][0]
        #get heading necessary to move into the next cell
        next_cell_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,cell)][0]
        #if these headings are equal, get cost for going straight
        if cell in self.goals:
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

    def a_star_search1(self, start, target):
        open = []
        heapq.heappush(open, (0, start))
        came_from = {}
        #cost_so_far = {}
        came_from[start] = None      #Every key in came_from is a cell in the maze and 
                                     #has for its value the previous cell the robot was in.
        self.cost_so_far[start] = 0  #Every key in cost_so_far is a cell in the maze and
                                     #has for its value the number of steps to get to that cell
                                     #from self.start (0,0) plus the cost of moving to this same
                                     #cell from the previous cell.
                
        found = False  # flag that is set when goal is found
        resign = False # flag set if robot can't find goal
        count = 0
        
        #This loop tracks the cost to get from any cell to the goal
        while not found and not resign:
            if len(open) == 0:
                resign = True
                print "Failed to find route to target."
            else:
                count += 1
                current = heapq.heappop(open)[1]
                
                if current == target:
                    found = True
                #else:
                elif current in self.maze_map:
                    #print 'test1'
                    for next_cell in self.neighbors(current):
                        # f = g + h: typical format for cost function for A* search
                        #Calculate the cost for moving from the current cell to any of the available neighhbors.
                        
                        #To calculate the current heading to determine whether a turn is needed we need the
                        #previous cell from which the robot moved to the current cell. This can be found in came_from.
                        move_cost = self.cost_so_far[current] + self.cost1(came_from[current],current, next_cell, start)
                        #if next not in cost_so_far or new_cost < cost_so_far[next]:
                        if move_cost < self.cost_so_far.get(next_cell, float("inf")):
                            self.cost_so_far[next_cell] = move_cost
                            priority = move_cost + self.heuristic(target, next_cell)
                            heapq.heappush(open,(priority, next_cell))
                            came_from[next_cell] = current
        
        #Once costs have all been calculated, we can now unpack the path to get from S to G
        current1 = target
        path = [current1]
        while current1 != start:
            current1 = came_from[current1]
            path.append(current1)
            
        path.reverse()
        
        return path

#This second implementation of A* Search runs in reverse, from goal to start, of the first implementation above.
#This allows for a route to be planned without actually waiting for the exact location of the goal entrance
#to be found.
    def a_star_search2(self):
        open = []
        came_from = {}
        #cost_so_far = {}
        for i in self.goals:
            heapq.heappush(open, (0, i))
            came_from[i] = None           #Every key in came_from is a cell in the maze and 
                                          #has for its value the previous cell the robot was in.
            self.cost_so_far[i] = 0            
                                          #Every key in cost_so_far is a cell in the maze and
                                          #has for its value the number of steps to get to that cell
                                          #from self.start (0,0) plus the cost of moving to this same
                                          #cell from the previous cell.
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
                
                #if current == self.goal_door_location:
                if current == self.start:
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
                        move_cost = self.cost_so_far[current] + self.cost2(came_from[current],current, next_cell)
                        #if next not in cost_so_far or new_cost < cost_so_far[next]:
                        if move_cost < self.cost_so_far.get(next_cell, float("inf")):
                            self.cost_so_far[next_cell] = move_cost
                            priority = move_cost + self.heuristic(self.start, next_cell)
                            heapq.heappush(open,(priority, next_cell))
                            came_from[next_cell] = current
        
        #Once costs have all been calculated, we can now unpack the path to get from S to G
        current1 = self.start
        path = [current1]
        while current1 not in self.goals:
            current1 = came_from[current1]
            path.append(current1)
        
        return path
    
    def d_star_lite_search(self):
        value = {}          #Every key in value is one the orientations - u, r, d, l - that have each cell as values.
                            #Each cell is also a key that contains the cost value for that cell in that orientation.
                
        policy = {}         #Every key in policy is one the orientations - u, r, d, l - that have each cell as values.
                            #Each cell is also a key that contains the policy action - L, #, R - for that cell in
                            #that orientation.
                
        path = []
        route = {}
        cost = [self.rotate_cost,self.no_rotate_cost,self.rotate_cost]    #Cost for left turn, no turn, right turn
        action_name = ['L', '#', 'R']    #Symbols for left turn, no turn, right turn
        
        init = 999
        for i in range(self.maze_dim):      
            for j in range(self.maze_dim):
                value[(i,j)] = {'u':init,'r':init,'d':init,'l':init}
                policy[(i,j)] = {'u':init,'r':init,'d':init,'l':init}
        
        change = True
        while change:
            change = False
            
            for cell in self.maze_map:
                for orientation in ['u','r','d','l']:
                    if cell == self.goal_door_location:
                    #if cell[0] in self.goal_bounds and cell[1] in self.goal_bounds:
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
        #self.print_map2(policy)
            
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
            if cell == (0,16):
                print orientation
            route[cell] = policy[cell][orientation]
            path.append(cell)
        
        #print_grid(route)
        
        return path
    
    
    def route_map(self, rot, mov):
        '''Function to create a list of moves from start (S) to goal (G) using '#' for no turn, 'L' for left turn, 'R' for right
           turn.'''
        
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
                self.rmap[self.goal_route[i]] = self.action_list[i]
        #return rmap
    
    def route_plan(self):
        '''Look through each set of three steps in goal_route from current location to see how many of them are in
           the same direction. This number will be the movement.'''
        
        movement = 0
        heading = [self.heading] #Initialize list with current heading to compare to the next three headings in goal_route
        if self.location != self.goal_door_location:
            for step in range(3):
                #Return the heading key in dir_move whose value is equal to the difference between each pair of successive
                #map locations in goal_route
                if self.index + step + 1 == len(self.goal_route):
                    break
                for k,v in dir_move.iteritems():
                    if v == map(sub,self.goal_route[self.index+step+1],self.goal_route[self.index+step]):
                        heading.append(k)
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
                self.rcount += 1
            
            self.update_position(rotation,movement)   
            self.mcount += 1
            self.index += movement
            
            return rotation,movement
    
    
