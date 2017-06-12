import numpy as np
from operator import add, sub
import heapq
import collections

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
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.sensors = []
        self.steps = 0
        self.exploring = True
        self.goal = False
        self.rotate_cost = 2
        self.no_rotate_cost = 1
        
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
        #Assign location to prev_location before updating
        self.prev_location = self.location
        # find the new heading after rotating from the current heading
        self.heading = [k for k,v in dir_rotation[self.heading].items() if v == rot][0]
        # apply movment
        self.location = map(add,self.location,[i*mov for i in dir_move[self.heading]])
        #if goal_door hasn't been found yet, check if this new position is the goal_door
        if goal_door():
            self.goal_door = goal_door()
    
    def update_maze_map(self):
        #if current location not in the map, add it along with the wall info for all 4 neighbor cells
        self.maze_map.setdefault(tuple(self.location), \
                            dict((heading,i) for heading,i in zip(dir_sensors[self.heading],self.sensors)))
        #add to this dictionary the distance to wall value for the previous cell that the robot just came from
        #the exception is cell (0,0) at the start - it has the boundary wall behind it.
        if self.location == self.start:
            self.maze_map[tuple(self.location)].update({dir_reverse[self.heading]:0})
        else:
            #Create a new dictionary entry for the reverse heading distance to wall, then add it to maze_map at current location
            back_sensor = {dir_reverse[self.heading]:self.maze_map[tuple(self.prev_location)].get(dir_reverse[self.heading]) + 1}
            self.maze_map[tuple(self.location)].update(back_sensor)
    
    def dead_ends(self):
        open_dirs = [] #Directions available to move from current location
        neighbor_cells = [] #Neighbor cells in the open_dirs list of available headings
        no_dead_ends = [] #List of sensor list indices that won't lead to a dead end
        #Check for available neighbor locations to current location in maze_map that are open
        open_dirs = [k for k,v in self.maze_map[tuple(self.location)].items() \
                     if k != dir_reverse[self.heading] and v != 0]
        for i in open_dirs:
            neighbor_cells = map(add,self.location,dir_move[d])
            if tuple(neighbor_cells) not in self.dead_end_set:
                no_dead_ends.append(dir_sensors[self.heading].index(i))
        return no_dead_ends
    
    def is_mapped(self):
        if len(self.maze_map) == self.maze_dim**2:
            return True
    
    #Purely random mapping only to use as a baseline for the poorest possible performance
    def random_explore(self):
        #randomly choose the index of a direction in the the sensor array
        rot_ind = random.choice(self.sensors.flatten())
        #Rotate and move
        rotation = rotation_index[rot_ind]
        movement = 1
        
        return rotation*90, movement
    
    #Random selection of available openings while trying to avoid dead ends
    #Tries to improve on random_explore
    def avoid_dead_ends_explore(self):
        #if there are more than 1 open directions to choose, pick one at random
        if np.count_nonzero(self.sensors) > 1:
            #If robot was in a dead end corridor at last location, it is now out. Set self.dead_end to false.
            if self.dead_end:
                self.dead_end = False
            #Check if any available openings lead to dead end and return those that aren't
            no_dead_ends = dead_ends()
            #Pick one of the sensor list indices at random
            rot_ind = random.choice(no_dead_ends)
            #Rotate and move
            rotation = rotation_index[rot_ind]
            movement = 1
        #If there's only 1 direction to move, just move there
        elif np.count_nonzero(self.sensors) = 1:
            if self.dead_end:
                self.dead_end_set.add(self.location)
            rot_ind = np.flatnonzero(self.sensors).item()
            rotation = rotation_index[rot_ind]
            movement = 1
        else:
            #If sensors all read 0, robot is in a dead end
            self.dead_end = True
            movement = 0
            rotation = 1
            self.dead_end_set.add(self.location)
            
        return rotation*90,movement

    def explore(self):
        goal = goal_door()
        if self.goal and (is_mapped() or self.steps = 900):
            return rotation,movement = ('Reset','Reset')
        
        rotation,movement = self.random_explore()
        #rotation,movement = self.avoid_dead_ends_explore()
    
    def neighbors(cell):
        open_dirs = []
        neighbors = []
        #Check for available neighbor locations that are open
        open_dirs = [k for k,v in maze_map[cell].items() if v != 0]
        neighbors = [tuple(map(add,cell,dir_move[i])) for i in open_dirs]
        #print neighbors
        
        return neighbors

    def cost(prev_cell, cell, next_cell):
        #get current cell heading based on previous cell
        if cell != tuple(start):
            heading = [k for k,v in dir_move.iteritems() if v == map(sub,cell,prev_cell)][0]
        #get heading necessary to move into the next cell
        next_cell_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,cell)][0]
        #if these headings are equal, get cost for going straight
        if cell == tuple(start):
            cost = no_rotate_cost
        elif heading == next_cell_heading:
            cost = no_rotate_cost
        #if the headings are not the same, get cost for turning
        else:
            cost = rotate_cost
        return cost
        
    def heuristic(a, b):
        #Computes the Manhattan distance between a and b.
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)
        
    def a_star_search(mapping):
        open = []
        heapq.heappush(open, (0, tuple(self.start)))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None #Every key in came_from is a cell in the maze and 
                                             #has for its value the previous cell the robot was in
        cost_so_far[start] = 0  #Every key in cost_so_far is a cell in the maze and
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
                            
                if current == goal_door:
                    found = True
                    #print count
                else:
                    for next in neighbors(current):
                        # f = g + h
                        #print came_from[current],current, next
                        new_cost = cost_so_far[current] + cost(came_from[current],current, next)
                        #if next not in cost_so_far or new_cost < cost_so_far[next]:
                        if new_cost < cost_so_far.get(next, float("inf")):
                            cost_so_far[next] = new_cost
                            priority = new_cost + heuristic(goal_door, next)
                            heapq.heappush(open,(priority, next))
                            came_from[next] = current
                            #print count
        
                            
        current = goal_door
        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
            
        print "Total steps to goal are {}.".format(len(path))
        return path
    
    def d_star_lite_search():
        
        return path
    
    def goal_seek(self):
        planning = True
        
        if planning:
            planning = False
            goal_route = a_star_search()
        
        #Update position from last rotation and movement updates
        self.position = update_position(rotation,movement)
        
        return rotation, movement

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
            #Update the map with the new position
            self.update_maze_map()
            #get next rotation, movement values
            rotation, movement = self.explore()
            #Update position with rotation and movement updates
            self.update_position(rotation,movement)
        else:
            rotation, movement = self.goal_seek()

return rotation, movement