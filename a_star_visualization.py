""" The script visualizes A* path search algorithm in Pygame. """


__author__ = "Andrey Ermishin"
__copyright__ = "Copyright (c) 2020"
__credits__ = []
__license__ = "GNU GPLv3"
__version__ = "1.0.0"
__maintainer__ = "Andrey Ermishin"
__email__ = "andrey.yermishin@gmail.com"
__status__ = "Production"


import random
import numpy as np
import heapq

import a_star_gui_pygame as gui


# global constants
EMPTY = 0
FULL = 1
START = 2
END = 3
SEARCH = 4
PATH = 5

REMOVED = (-1, -1)  # placeholder for a cell removed from a heap


class Grid:
    """ Numpy implementation of 2D grid of cells. """
    
    def __init__(self, width, height):
        """
        Initialize grid to be empty with given width and height.
        Indexed by rows (top to bottom), then by columns (left to right).
        """
        self.width = width
        self.height = height
        # Initialize grid with EMPTY values.
        self.cells = np.zeros((height, width))  # 2D grid with rows as y axis
    
    def get_grid_height(self):
        """ Return the height of the grid for use in GUI. """
        return self.height

    def get_grid_width(self):
        """ Return the width of the grid for use in GUI. """
        return self.width

    def clear(self):
        """ Clear grid to be empty. """
        self.cells[:] = EMPTY
    
    def clear_from(self, value):
        """ Update grid cells containing the value to EMPTY values. """
        # np.place(self.cells, self.cells == value, EMPTY)
        self.cells = np.where(self.cells == value, EMPTY, self.cells)
    
    def set_value(self, row, col, value):
        """ Set the cell with index (row, col) equal to value. """
        self.cells[row, col] = value
    
    def get_idx_value_pairs(self):
        """ Return an iterator yielding pairs of array indices and values. """
        return np.ndenumerate(self.cells)
    
    def is_empty(self, row, col):
        """ Check whether cell with index (row, col) is empty. """
        return self.cells[row, col] == EMPTY
    
    def four_neighbors(self, row, col):
        """
        Return horiz/vert neighbors of the cell (row, col).
        """
        ans = []
        if row > 0:
            ans.append((row - 1, col))
        if row < self.height - 1:
            ans.append((row + 1, col))
        if col > 0:
            ans.append((row, col - 1))
        if col < self.width - 1:
            ans.append((row, col + 1))
        return ans

    def eight_neighbors(self, row, col):
        """
        Return horiz/vert and diagonal neighbors of the cell (row, col).
        """
        ans = []
        for row_offset in range(-1, 2):
            for col_offset in range(-1, 2):
                i = row + row_offset
                j = col + col_offset
                if 0<=i<self.height and 0<=j<self.width and (i, j)!=(row, col):
                    ans.append((i, j))
        return ans
    
    def get_index(self, point, cell_size):
        """ Return index of a cell by its screen coordinates. """
        return (point[1] // cell_size, point[0] // cell_size)
    
    def __str__(self):
        """ Return multi-line string represenation of grid. """
        ans = '\n'.join(str(self.cells[row]) for row in range(self.height))
        return ans + '\n'


class Astar(Grid):
    """
    Class for simulating A* path search algorithm on grid with obstacles.
    """

    def __init__(self, grid_width, grid_height, obstacle_list=None,
                 start_pos=None, end_pos=None):
        """
        Create a simulation of given size with given obstacles,
        start and end positions.
        """
        Grid.__init__(self, grid_width, grid_height)
        
        if obstacle_list is not None:
            for cell in obstacle_list:
                self.set_value(cell[0], cell[1], FULL)
        
        if start_pos is not None:
            self.set_value(start_pos[0], start_pos[1], START)
        if end_pos is not None:
            self.set_value(end_pos[0], end_pos[1], END)
        self.start_end_points = (start_pos, end_pos)
    
        self.p_queue = []   # list of entries arranged in a heap
        self.entries = {}   # mapping of cells to entries in a heap
        self.count = 0   # unique sequence count for priority queue entries
        self.came_from = {} # dictionary of predecessors for every cell

        self.is_over = False
    
    def get_start_end_points(self):
        """ Return initial(!) start and end positions. """
        return self.start_end_points
    
    def clear(self):
        """ Clear all the grid. """
        Grid.clear(self)

        self.is_over = False

        self.clear_p_queue()
    
    def clear_p_queue(self):
        """ Clear priority queue to be empty. """
        self.p_queue = []
        self.entries = {}
        self.count = 0
        self.came_from = {}
    
    def add_cell(self, cell, priority=0):
        """ Add a new cell or update min priority of an existing cell. """
        new_cell_better = False
        if cell in self.entries and priority < self.entries[cell][0]:
            new_cell_better = True
            self.remove_cell(cell)
        if cell not in self.entries or new_cell_better:
            entry = [priority, self.count, cell]
            self.entries[cell] = entry
            heapq.heappush(self.p_queue, entry) # enqueue pointer to a list
            self.count += 1

    def remove_cell(self, cell):
        """
        Mark an existing cell as REMOVED.
        Removes cell from self.entries, but REMOVED will stay in the heap.
        """
        entry = self.entries.pop(cell)
        entry[-1] = REMOVED

    def pop_cell(self):
        """
        Remove and return the lowest priority cell.
        Raise KeyError if empty.
        """
        while self.p_queue:
            _priority, _count, cell = heapq.heappop(self.p_queue)
            if cell is not REMOVED:
                self.entries.pop(cell)
                return cell
        raise KeyError('Pop from an empty priority queue.')
    
    def reconstruct_path(self, current):
        """ Return path from start cell to current cell. """
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        return list(reversed(path))
    
    def a_star_search_iter(self, start_pos, end_pos):
        """
        The algorithm minimizes the sum of cost of the path 
        from start position to current position (g(x)) and 
        heuristic function h(x) that estimates cost of the cheapest 
        path from current to the end position (Euclidean distance).
        Algorithm uses min priority queue with entries like: 
        [priority, count, cell].
        Return/make a list of grid cells forming a path from start to end.
        """        
        if self.p_queue:
            cur_cell = self.pop_cell()
            for neighbor in self.eight_neighbors(*cur_cell):
                if neighbor == end_pos:
                    self.is_over = True
                    self.came_from[neighbor] = cur_cell
                    break
                elif self.is_empty(*neighbor):
                    g_x = len(self.reconstruct_path(cur_cell))
                    h_x = np.hypot(end_pos[0] - neighbor[0],
                                   end_pos[1] - neighbor[1])
                    cost = g_x + h_x
                    self.add_cell(neighbor, cost)
                    self.came_from[neighbor] = cur_cell
                    self.set_value(neighbor[0], neighbor[1], SEARCH)
    
    def a_star_search(self, start_pos, end_pos):
        """
        The algorithm minimizes the sum of cost of the path 
        from start position to current position (g(x)) and 
        heuristic function h(x) that estimates cost of the cheapest 
        path from current to the end position (Euclidean distance).
        Algorithm uses min priority queue with entries like: 
        [priority, count, cell].
        Return/make a list of grid cells forming a path from start to end.
        """
        self.add_cell(start_pos)
        
        while self.p_queue:
            cur_cell = self.pop_cell()
            for neighbor in self.eight_neighbors(*cur_cell):
                if neighbor == end_pos:
                    self.came_from[neighbor] = cur_cell
                    path = self.reconstruct_path(neighbor)
                    for cell in path[1:-1]:
                        self.set_value(cell[0], cell[1], PATH)
                    return
                elif self.is_empty(*neighbor):
                    g_x = len(self.reconstruct_path(cur_cell))
                    h_x = np.hypot(end_pos[0] - neighbor[0],
                                   end_pos[1] - neighbor[1])
                    cost = g_x + h_x
                    self.add_cell(neighbor, cost)
                    self.came_from[neighbor] = cur_cell
                    self.set_value(neighbor[0], neighbor[1], SEARCH)


# (y, x)
obstacles = [(row, col) for row in (15, 16) for col in range(12, 22)]
obstacles += [(row, col) for row in range(11, 15) for col in (20, 21)]
start, end = (5, 10), (20, 24)
gui.run(Astar(34, 24, obstacles, start, end))
# gui.run(Astar(34, 24)) # will be 1280x720 with buttons on the left
