import numpy as np
from operator import add
import heapq
import collections

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = self.start = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.steps = 0
        self.exploring = True
        self.goal_door = False
        
        self.dir_sensors =  {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                             'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                             'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                             'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
        self.dir_reverse =  {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
                             'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
        self.dir_move =     {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
        self.dir_rotation = {'u': {'l':-1, 'u':0, 'r':1}, 'r': {'u':-1, 'r':0, 'd':1},
                            'd': {'r':-1, 'd':0, 'l':1}, 'l': {'d':-1, 'l':0, 'u':1},
                            'up': {'l':-1, 'u':0, 'r':1}, 'right': {'u':-1, 'r':0, 'd':1},
                            'down': {'r':-1, 'd':0, 'l':1}, 'left': {'d':-1, 'l':0, 'u':1}}
        self.rotation_index = [-1, 0, 1]
        
        self.maze_map = {}   '''{(0, 0): {'d': 0, 'l': 0, 'r': 0, 'u': 11},
                                 (0, 1): {'d': 1, 'l': 0, 'r': 0, 'u': 10},
                                 (0, 2): {'d': 2, 'l': 0, 'r': 3, 'u': 9}}
                                
                                The self.maze_map dict will look like above after 3 'up' moves from (0,0)
                            '''
        self.dead_end_map = {}
        self.closed = [[0 for row in range(maze_dim)] for col in range(maze_dim)]
        self.closed[self.start[0]][self.start[1]] = 1
        
        self.expand = [[0 for row in range(maze_dim)] for col in range(maze_dim)]
        
    # check for goal entered
    def goal_door(m_dim, loc):
        goal_bounds = [m_dim/2 - 1, m_dim/2]
        if loc[0] in goal_bounds and loc[1] in goal_bounds:
            goal_door_location = loc
        else:
            goal_door_location = None
        return goal_door_location
    
    def update_map(self,sens):
        #if current location not in the map, add it along with the wall info for all 4 neighbor cells
        self.maze_map.setdefault(self.location, \
                            dict((heading,i) for heading,i in zip(self.dir_sensors[self.heading],sens)))
        #add to this dictionary the previous cell that the robot just came from as having no wall either
        #the exception is cell (0,0) at the start - it has the boundary wall behind it.
        if self.location == self.start:
            self.maze_map[self.location].update({self.dir_reverse[self.heading]:0})
        else: 
            self.maze_map[tuple(self.location)][self.dir_reverse[self.heading]] = \
                self.maze_map[(0,1)].get(self.dir_reverse[self.heading],1) + 1
            
    def update_position(self, rot, mov):

        # find the new heading after rotating from the current heading
        self.heading = [k for k,v in self.dir_rotation[self.heading].items() if v == rot][0]
        # apply movment
        self.location = map(add,self.location,[i*mov for i in dir_move[self.heading]])
    
    def explore(self, sensors):
        #if there are more than 1 open directions to choose, pick one at random
        if np.count_nonzero(sensors):
            rot_ind = random.choice(np.nonzero(sensors)[0])
            rotation = self.rotation_index[rot_ind]
            movement = 1
        else:
            #If dead end, reverse until 3 maze_map directions > 0
            dead_end = True
            while dead_end:
                num_open = self.maze_map[
                movement = -1
                rotation =  0
                
            
        #Update position from last roation and movement updates
        update_position(rotation,movement)
        
        return rotation*90, movement
    
    def reconstruct_path(came_from, start, goal, num):
        current = goal
        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        if num == first:
            #path.append(start) # optional
            path.reverse() # optional
        return path

    def heuristic(a, b):
        #Computes the Manhattan distance between a and b.
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)
    
    def a_star_search(graph):
        #frontier = PriorityQueue()
        #frontier.put(start, 0)
        #came_from = {}
        #cost_so_far = {}
        #came_from[start] = None
        #cost_so_far[start] = 0
        
        #After reset, set robot back to starting position
        self.location = self.start
        self.heading = 'up'
        
        #while not frontier.empty():
        current = self.p_frontier.get()
            
        goal = goal_door(graph.dim, cell)
        
        for next in graph.neighbors(cell):
            new_cost = self.cost_so_far[cell] + graph.cost(cell, next)
            #if next not in cost_so_far or new_cost < cost_so_far[next]:
            if new_cost < self.cost_so_far.get(next, float("inf")):
                self.cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = cell
        
        return self.route

    
    def goal_seek(self, sensors):
        
        
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
        
        #Initialize rotation and movement at start of first run and start of second run
        if (self.exploring and self.start) or (not self.exploring and self.start):
            rotation = 0
            movement = 0
        
        if self.exploring:
            #Update the map with the new position
            update_map(np.array(sensors))
            #get next rotation, movement values
            rotation, movement = self.explore(np.array(sensors))
        else:
            rotation, movement = self.goal_seek(sensors)

return rotation, movement