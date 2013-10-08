import heapq
import time
import sys
import Image
import math
import pdb

t_start = time.time()

preprocessRadio = 10

"""debugger"""
def info(type, value, tb):
   if hasattr(sys, 'ps1') or not sys.stderr.isatty():
      # we are in interactive mode or we don't have a tty-like
      # device, so we call the default hook
      sys.__excepthook__(type, value, tb)
   else:
      import traceback
      # we are NOT in interactive mode, print the exception...
      traceback.print_exception(type, value, tb)
      print
      # ...then start the debugger in post-mortem mode.
      pdb.pm()
sys.excepthook = info

"""pre-process"""
def preprocess(image):
    base_pixels = image.load()
    width,height = image.size
    new_pixels = [[(255,255,255) for x in xrange(height)] for x in xrange(width)]
    for y in range(height):
        for x in range(width):
            if not isWalkable(base_pixels[x,y]):
                for xi in range(-preprocessRadio,preprocessRadio+1):
                    for yi in range(-preprocessRadio,preprocessRadio+1):
                        if (x+xi >= 0 and x+xi< width and y+yi>= 0 and y+yi < height):
                            new_pixels[x+xi][y+yi] = (0,0,0)
            if isred(base_pixels[x,y]):
                new_pixels[x][y] = (255,0,0)
            if isblue(base_pixels[x,y]):
                new_pixels[x][y] = (0,0,255)
    return new_pixels 

"""pos start"""
def define_start(image):
    base_pixels = image.load()
    width,height = image.size

    for x in range(width):
        if iswhite(base_pixels[x, 0]):
            global start
            start = (x, 0)

def define_end(image):
    base_pixels = image.load()
    width,height = image.size

    for x in range(width):
        if iswhite(base_pixels[x, height-1]):
            global end
            end = (x, height-1)

"""color start"""
def define_color_start_end(image):
    base_pixels = image.load()
    width,height = image.size

    for y in range(height):
        for x in range(width):
            if isblue(base_pixels[x,y]):
                global start
                start = (x,y)
            if isred(base_pixels[x,y]):
                global end
                end = (x,y)

def isblue(value):
    r,g,b = value
    return r < 128 and g < 128 and b > 128

def isred(value):
    r,g,b = value
    return r > 128 and g < 128 and b < 128

def iswhite(value):
    r, g, b = value
    return r > 128 and g > 128 and b > 128

def isWalkable(value):
    r, g, b = value
    return r > 128 or g > 128 or b > 128

def astar(start_node, target_node):
    """The A* pathfinding algorithm"""
    global closed
    closed = set()
    open_set = set()
    open = []
    """ensure start_node is terminating node in path reconstruction"""
    if hasattr(start_node, '_came_from'):
        del start_node._came_from
   
    h = start_node._h = start_node.heuristic(target_node)
    g = start_node._g = 0
    f = start_node._h 
   
    start_triplet = [f, h, start_node]
    heapq.heappush(open, start_triplet)
    open_d = {start_node: start_triplet}
    while open:
        f, h, node = heapq.heappop(open)
        del open_d[node]
        if node == target_node:
            return reconstruct_path(node)
        closed.add(node)
        for neighbor in node.get_neighbors():
            if neighbor in closed:
                continue
         
            tentative_g = node._g + node.move_cost(neighbor)
            if neighbor not in open_d:
                neighbor._came_from = node
                neighbor._g = tentative_g
                h = neighbor._h = neighbor.heuristic(target_node)
                d = open_d[neighbor] = [tentative_g + h, h, neighbor]
                heapq.heappush(open, d)
            else:
                neighbor = open_d[neighbor][2] 
                if tentative_g < neighbor._g:
                    neighbor._came_from = node
                    neighbor._g = tentative_g
                    open_d[neighbor][0] = tentative_g + neighbor._h
                    heapq.heapify(open)
                  
    """ there is no path"""
    return None

def reconstruct_path(target_node):
    path = []
    node = target_node
    while hasattr(node, '_came_from'):
        path.append(node)
        node = node._came_from
    return reversed(path)

"""heuristic"""
def manhattan(pos1, pos2):
    return sum(abs(a - b) for a, b in zip(pos1, pos2))

"""nodes"""
class RectNode(object):
    __slots__ = ('walkable', 'neighbor_gen', '_move_cost', 'pos',
        'default_walkable', '_heuristic',
        '_came_from', '_h', '_g') 
    def __init__(self, pos,
            move_cost=1, walkable=None, default_walkable=True,
            neighbor_gen=None, heuristic=manhattan):
        """Create a RectNode
        with position `pos` and that generates neighbors by calling
        `neighbor_gen` with similar arguments
        `move_cost` is a constant cost for moving directly from one node to
            the next
        `walkable` is a map from position->walkable for any tile position
            if a position is not in `walkable`, it is assumed
            `default_walkable` (default_walkable is True by default)"""
        if walkable is None:
            walkable = {}
        self.walkable = walkable
        if neighbor_gen is None:
            neighbor_gen = type(self)
        self.neighbor_gen = neighbor_gen
        self._move_cost = move_cost
        self.pos = pos
        self.default_walkable = default_walkable
        self._heuristic = heuristic
   
    def __hash__(self):
        return hash(self.pos)
   
    def __eq__(self, o):
        return self.pos == o.pos
   
    def _get_x(self):
        return self.pos[0]
   
    def _get_y(self):
        return self.pos[1]
   
    x = property(fget=_get_x)
    y = property(fget=_get_y)
   
    def get_neighbors(self):
        """Get all the traversable neighbor nodes
        use neighbor_gen to generate nodes given positions"""
        for i in ((1,0), (-1,0), (0, 1), (0, -1)):
            pos = self.x - i[0], self.y - i[1]
            if self.walkable.get(pos, self.default_walkable):
                yield self.neighbor_gen(pos, walkable=self.walkable,
                        default_walkable=self.default_walkable,
                        neighbor_gen=self.neighbor_gen,
                        heuristic=self._heuristic)
   
    def heuristic(self, node):
        return self._heuristic(self.pos, node.pos)
   
    def move_cost(self, node):
        return self._move_cost


"""load file"""

def read_tiles(image, pixels):
    """Read file `f` and yield (position, char) tuples"""
    if pixels == None:
        base_pixels = image.load()
    else:
        base_pixels = pixels
    width,height = image.size

    for y in range(height):
        for x in range(width):
            yield (x, y), base_pixels[x][y]

def file_to_tile(image, start, end, pixels = None):
    walkable = {}
    start_pos = target_pos = None
    for pos, pixel in read_tiles(image, pixels):
        if pos == start:
            start_pos = pos
        elif pos == end:
            target_pos = pos
        elif not isWalkable(pixel):
            walkable[pos] = False

    w,h = image.size
    for x in range(w):
        walkable[(x, -1)] = False
        walkable[(x, h)] = False
    for y in range(h):
        walkable[-1, y] = False
        walkable[w, y] = False

    start_node = RectNode(start_pos,
        walkable=walkable, heuristic=manhattan)
    if target_pos is None:
        target_node = None
    else:
        target_node = RectNode(target_pos,
            walkable=walkable, heuristic=manhattan)
    return start_node, target_node


# invoke: python mazesolver.py <mazefile> <outputfile>[.jpg|.png|etc.]
base_img = Image.open(sys.argv[1])
define_color_start_end(base_img)
"""
define_start(base_img)
define_end(base_img)
"""
pixels = preprocess(base_img)

debug_img = Image.open(sys.argv[1])
debug_pixels = debug_img.load()
w,h = debug_img.size
for y in range(h):
    for x in range(w):
        debug_pixels[x,y] = pixels[x][y]

debug_img.save("debug.png")


s,t = file_to_tile(base_img, start, end, pixels)
path = astar(s,t)

path_img = Image.open(sys.argv[1])
path_pixels = path_img.load()

if (path):
    green = (0,255,0)
    red = (255,0,0)
    blue = (0,0,255)
    for c in closed:
        x,y = c.pos
        #path_pixels[x,y] = (66,99,66)
    
    for position in path:
        x,y = position.pos
        #pdb.set_trace()
        path_pixels[x,y] = green



    path_pixels[start[0], start[1]] = blue
    path_pixels[end[0], end[1]] = red

    path_img.save(sys.argv[2])
else:
    print "Error"

t_end = time.time()

print math.floor(t_end - t_start)