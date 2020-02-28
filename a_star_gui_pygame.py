""" GUI class for 'a_star_visualization.py'. """


__author__ = "Andrei Ermishin"
__copyright__ = "Copyright (c) 2020"
__credits__ = []
__license__ = "GNU GPLv3"
__version__ = "1.0.0"
__maintainer__ = "Andrei Ermishin"
__email__ = "andrey.yermishin@gmail.com"
__status__ = "Production"

import sys
import pygame


# Global constants
EMPTY = 0
FULL = 1
START = 2
END = 3
SEARCH = 4
PATH = 5

BLACK = pygame.Color('black')
ORANGE = pygame.Color('orangered')
IND_RED = pygame.Color('indianred')
GREEN = pygame.Color('limegreen')
BLUE = pygame.Color('blue')
PURPLE = pygame.Color('purple')
# PURPLE4 = pygame.Color(85, 26, 139)
DARK_BLUE = pygame.Color('darkslateblue')
GRAY = pygame.Color('gray')
LIGHT_GRAY = pygame.Color('lightgray')

CELL_COLORS = {EMPTY: BLACK,
               FULL: GRAY,
               START: ORANGE,
               END: GREEN,
               SEARCH: PURPLE,
               PATH: BLUE}

ADD_MAP = {FULL: 0, START: 1, END: 2}

# GUI constants
CELL_SIZE = 30
WIDTH0 = 260    # for controls in Pygame

# for timer
TIMER_STOP = 0
timer_play_sim = pygame.USEREVENT + 1
TIMER_DELAY = 100   # 0.1s


# Button with hovering:
def button(text, surface, location, color_act, color_idle, mouse_pos):
    color = color_act if location.collidepoint(mouse_pos) else color_idle
    pygame.draw.rect(surface, color, location)

    # text over button with centering
    font = pygame.font.SysFont('arial', 20, bold=True)
    text_surface = font.render(text, True, BLACK)
    text_rect = text_surface.get_rect()
    text_rect.center = location.center
    surface.blit(text_surface, text_rect)


class AstarGUI:
    """
    GUI class for A* path search algorithm.
    """

    def __init__(self, simulation):
        """ Create a frame. """
        self.simulation = simulation
        self.grid_w = simulation.get_grid_width()
        self.grid_h = simulation.get_grid_height()

        # Set up window
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH0 + self.grid_w*CELL_SIZE,
                                               self.grid_h*CELL_SIZE))
        pygame.display.set_caption('A* path search algorithm simulation'
                                   + ' (by Andrei Ermishin)')

        self.item_type = FULL
        self.item_rect_idx = 0

        self.drag_started = False
        self.drag_points = set()

        pos_1, pos_2 = simulation.get_start_end_points()
        self.start_pos = pos_1 if pos_1 else ()
        self.end_pos = pos_2 if pos_2 else ()

        self.btn_obst = pygame.Rect(50, 40, WIDTH0-100, 40)
        self.btn_start = pygame.Rect(50, 100, WIDTH0-100, 40)
        self.btn_end = pygame.Rect(50, 160, WIDTH0-100, 40)
        self.btn_clear = pygame.Rect(80, 250, WIDTH0-160, 40)

        self.btn_play_sim = pygame.Rect(50, 420, WIDTH0-100, 40)
        self.btn_clear_search = pygame.Rect(80, 490, WIDTH0-160, 40)
        
        self.sim_running = False
        self.path = []

    def start(self):
        """ Start the GUI. """
        clock = pygame.time.Clock()
        running = True
    
        while running:

            # register event handlers
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_pos = event.pos
                    if mouse_pos[0] > WIDTH0:
                        if self.item_type == FULL:
                            self.drag_started = True
                            self.drag_points.add(self.pos_to_index(mouse_pos))
                        else:
                            self.add_item(mouse_pos)
                    # buttons callback
                    else:
                        if self.btn_obst.collidepoint(mouse_pos):
                            self.set_type(FULL)
                        elif self.btn_start.collidepoint(mouse_pos):
                            self.set_type(START)
                        elif self.btn_end.collidepoint(mouse_pos):
                            self.set_type(END)
                        elif self.btn_clear.collidepoint(mouse_pos):
                            self.clear()
                        elif self.btn_clear_search.collidepoint(mouse_pos):
                            self.clear_search()
                        elif self.btn_play_sim.collidepoint(mouse_pos):
                            # Redraw screen until stop pressed or finish.
                            # self.start() - will be Stop when pressed
                            self.search_simulation()
                
                if event.type == pygame.MOUSEMOTION and self.drag_started:
                    if event.pos[0] > WIDTH0:
                        self.drag_points.add(self.pos_to_index(event.pos))
                
                if event.type == pygame.MOUSEBUTTONUP and self.drag_started:
                    self.drag_started = False
                    self.add_obstacles()
                
                if self.sim_running and event.type == timer_play_sim:
                    self.play_sim()
            
            self.draw(self.screen)
            pygame.display.flip()

            # It micro pauses the while loop to run 30 times a second max.
            clock.tick(30)
        
        pygame.time.set_timer(timer_play_sim, TIMER_STOP)
        pygame.quit()
        sys.exit()

    def clear(self):
        """ Event handler for button that clears everything. """
        self.simulation.clear()

        self.drag_points.clear()
        self.start_pos = ()
        self.end_pos = ()
        self.sim_running = False
        self.path = []

    def clear_search(self):
        """ Clear grid cells containing SEARCH or PATH values. """
        if self.start_pos and self.end_pos:
            self.sim_running = False
            self.path = []
            self.simulation.clear_p_queue()
            self.simulation.clear_from(SEARCH)
            self.simulation.clear_from(PATH)
    
    def search_simulation(self):
        """ Start iterative simulation of A* search. """
        ### some nice peace of code here:
        only_author_has = 1
        real_code = only_author_has
    
    def play_sim(self):
        """ Execute one iteration of A* search algorithm and fill the grid. """
        if not self.simulation.is_over:
            self.simulation.a_star_search_iter(self.start_pos, self.end_pos)
        else:
            ### some nice peace of code here:
            only_author_has = 1
            real_code = only_author_has
                    

    def set_type(self, item_type):
        """ Set type of item to obstacles, start or end point. """
        self.item_type = item_type
        self.item_rect_idx = ADD_MAP[item_type]

    def pos_to_index(self, position):
        """ Return (row, col) converted from coordinates. """
        new_pos = (position[0] - WIDTH0, position[1])
        return self.simulation.get_index(new_pos, CELL_SIZE)
    
    def add_obstacles(self):
        """ Event handler to add new obstacles. """
        for row, col in self.drag_points:
            if self.simulation.is_empty(row, col):
                self.simulation.set_value(row, col, FULL)
    
    def add_item(self, click_position):
        """ Event handler to add new start and end points. """
        row, col = self.pos_to_index(click_position)
        if self.simulation.is_empty(row, col):
            if self.item_type == START:
                if self.start_pos:
                    self.simulation.set_value(self.start_pos[0],
                                              self.start_pos[1], EMPTY)
                self.start_pos = (row, col)
                self.simulation.set_value(row, col, START)
            elif self.item_type == END:
                if self.end_pos:
                    self.simulation.set_value(self.end_pos[0],
                                              self.end_pos[1], EMPTY)
                self.end_pos = (row, col)
                self.simulation.set_value(row, col, END)

    def draw_grid(self, surface):
        """ Draw entire grid. """
        # Boundary
        pygame.draw.rect(surface, DARK_BLUE,
                         [WIDTH0, 0,
                          self.grid_w * CELL_SIZE, self.grid_h * CELL_SIZE],
                         1)
        # Inner lines:
        for row in range(1, self.grid_h):
            for col in range(1, self.grid_w):
                # horizontal line
                pygame.draw.line(surface, DARK_BLUE, (WIDTH0, row*CELL_SIZE),
                                 (WIDTH0+self.grid_w*CELL_SIZE, row*CELL_SIZE),
                                 1)
                # vertical line
                pygame.draw.line(surface, DARK_BLUE, (WIDTH0+col*CELL_SIZE, 0),
                                 (WIDTH0+col*CELL_SIZE, self.grid_h*CELL_SIZE),
                                 1)
        
        for (row, col), value in self.simulation.get_idx_value_pairs():
            if value != EMPTY:
                margin = CELL_SIZE // 3 if value == SEARCH else 0
                margin = CELL_SIZE // 10 if value == PATH else margin
                pygame.draw.rect(surface, CELL_COLORS[value],
                                 [WIDTH0 + col*CELL_SIZE + margin,
                                  row*CELL_SIZE + margin,
                                  CELL_SIZE - 2*margin, CELL_SIZE - 2*margin])

    def draw(self, surface):
        """
        Handler for drawing the grid and buttons.
        """
        surface.fill(BLACK)

        mouse_pos = pygame.mouse.get_pos()

        # Draw buttons
        button('Add:   obstacles', surface, self.btn_obst,
               LIGHT_GRAY, GRAY, mouse_pos)
        button('Add:     start -->', surface, self.btn_start,
               LIGHT_GRAY, GRAY, mouse_pos)
        button('Add:     --> end', surface, self.btn_end,
               LIGHT_GRAY, GRAY, mouse_pos)
        button('Clear all', surface, self.btn_clear,
               IND_RED, GRAY, mouse_pos)
        button('Clear search', surface, self.btn_clear_search,
               IND_RED, GRAY, mouse_pos)
        
        if not self.sim_running:
            button('Simulation', surface, self.btn_play_sim,
                GREEN if self.start_pos and self.end_pos else LIGHT_GRAY,
                GRAY, mouse_pos)
        else:
            button('<Stop>', surface, self.btn_play_sim,
                   IND_RED, GRAY, mouse_pos)
        
        # Draw toggle item's rect.
        pygame.draw.rect(surface, PURPLE,
                         [45, 35 + self.item_rect_idx*60, WIDTH0 - 90, 50], 2)
        
        self.draw_grid(surface)


def run(sim):
    """ Start interactive simulation. """
    gui = AstarGUI(sim)
    gui.start()
