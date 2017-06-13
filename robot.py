import numpy as np
from operator import add, sub
import heapq
import collections
import random

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

        self.location = self.start = (0, 0)
        self.prev_location = ()
        self.heading = 'u'
        self.maze_dim = maze_dim
        self.steps = 0
        self.count = 0
        self.index = 0
        self.exploring = True
        self.planning = False
        self.goal = False
        self.reset = False
        self.rotate_cost = 2
        self.no_rotate_cost = 1
        self.goal_route = []
        self.maze_map = {}   
        '''{(0, 0): {'d': 0, 'l': 0, 'r': 0, 'u': 11},
            (0, 1): {'d': 1, 'l': 0, 'r': 0, 'u': 10},
            (0, 2): {'d': 2, 'l': 0, 'r': 3, 'u': 9}}
           
           The self.maze_map dict will look like above after 3 'up' moves from (0,0),
           for test_maze_01.txt
        '''
        self.dead_end = False
        self.dead_end_set = set()   #Cells that lead into dead ends and dead end corridors.
        
    # check if goal room entered
    def goal_door(self):
        goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        if self.location[0] in goal_bounds and self.location[1] in goal_bounds:
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
        self.heading = [k for k,v in dir_rotation[self.heading].items() if v == rot][0]
        # apply movment
        self.location = tuple(map(add,self.location,[i*mov for i in dir_move[self.heading]]))
        #if goal_door hasn't been found yet, check if this new position is the goal_door
        if not self.goal:
            self.goal_door_location = self.goal_door()
    
    def update_maze_map(self):
        #if current location not in the map, add it along with the wall info for all 4 neighbor cells
        self.maze_map.setdefault(tuple(self.location), \
                            dict((heading,i) for heading,i in zip(dir_sensors[self.heading],self.sensors)))
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
    
    def dead_ends(self):
        open_dirs = [] #Directions available to move from current location
        neighbor_cells = [] #Neighbor cells in the open_dirs list of available headings
        no_dead_ends = [] #List of sensor list indices that won't lead to a dead end
        #Check for available neighbor locations to current location in maze_map that are open
        open_dirs = [k for k,v in self.maze_map[self.location].iteritems() \
                     if k != dir_reverse[self.heading] and v != 0]
        for i in open_dirs:
            neighbor_cells = map(add,self.location,dir_move[i])
            if tuple(neighbor_cells) not in self.dead_end_set:
                no_dead_ends.append(dir_sensors[self.heading].index(i))
        return no_dead_ends
    
    def is_mapped(self):
        if len(self.maze_map) == self.maze_dim**2:
            return True
    
    #Purely random mapping only to use as a baseline for the poorest possible performance
    def random_explore(self):
        #randomly choose the index of a direction in the the sensor array
        if np.count_nonzero(self.sensors) > 1:
            rot_ind = random.choice(np.nonzero(self.sensors)[0])
            #Rotate and move
            rotation = rotation_index[rot_ind]
            movement = 1
        else:
            #If sensors all read 0, robot is in a dead end
            #self.dead_end = True
            movement = 0
            rotation = 1
            #self.dead_end_set.add(self.location)
        
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
            no_dead_ends = self.dead_ends()
            #Pick one of the sensor list indices at random
            rot_ind = random.choice(no_dead_ends)
            #Rotate and move
            rotation = rotation_index[rot_ind]
            movement = 1
        #If there's only 1 direction to move, just move there
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
            
        return rotation,movement

    def explore(self):
        if self.goal and (self.is_mapped() or self.steps == 900):
            self.planning = True #Flag to allow a_star_search to run prior to second run
            rotation,movement = ('Reset','Reset')
            self.exploring = False
            print 'Found goal and fully mapped.'
            return rotation,movement
        else:
            #Update the maze map with the current position
            self.update_maze_map()
            #rotation,movement = self.random_explore()
            rotation,movement = self.avoid_dead_ends_explore()
            self.update_position(rotation,movement)
            return rotation*90,movement
    
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
                
        found = False  # flag that is set when goal is found
        resign = False # flag set if robot can't find goal
        count = 0
                
        while not found and not resign:
            if len(open) == 0:
                resign = True
                print "Failed to find route to goal."
            else:
                count += 1
                current = heapq.heappop(open)[1]
                #print current
                            
                if current == self.goal_door_location:
                    found = True
                    #print self.goal_door_location
                #else:
                elif current in self.maze_map:
                    #print 'test1'
                    for next_cell in self.neighbors(current):
                        # f = g + h
                        #print came_from[current],current, next_cell
                        #print 'test2'
                        new_cost = cost_so_far[current] + self.cost(came_from[current],current, next_cell)
                        #print new_cost
                        #if next not in cost_so_far or new_cost < cost_so_far[next]:
                        if new_cost < cost_so_far.get(next_cell, float("inf")):
                            cost_so_far[next_cell] = new_cost
                            priority = new_cost + self.heuristic(self.goal_door_location, next_cell)
                            heapq.heappush(open,(priority, next_cell))
                            came_from[next_cell] = current
                            #print came_from[next_cell]
                            #print count
        
                            
        current1 = self.goal_door_location
        path = [current1]
        while current1 != self.start:
            #print current1, came_from
            current1 = came_from[current1]
            #print current1
            path.append(current1)
        path.reverse()
        
        return path
    
    def d_star_lite_search(self):
        
        return path
    
    def goal_seek(self):
        #To begin relocate robot to start and find the best route to goal_door
        if self.planning:
            self.location = self.start
            self.heading = 'u'
            self.goal_route = self.a_star_search()
            #goal_route = d_star_lite_search()
            steps = len(self.goal_route)
            print "Total length of route is {} steps.".format(steps)
            self.planning = False
        
        #Look through each set of three steps in goal_route from current location to see how many of them are in
        #the same direction. This number will be the movement.
        movement = 0
        heading = [self.heading]
        if self.location != self.goal_door_location:
            for step in range(4):
                heading.append([k for k,v in dir_move.iteritems() \
                        if v == map(sub,self.goal_route[self.index+step+1],self.goal_route[self.index+step])][0])
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
        else: 
            print "Goal reached in {} steps.".format(count)
        return rotation*90,movement

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
            #get next rotation, movement values
            rotation, movement = self.explore()
            #print self.location, self.heading, rotation, movement
            
        else:
            rotation, movement = self.goal_seek()

        return rotation, movement