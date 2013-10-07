import sys

from Queue import Queue
import Image

def define_start():
	for x in range(width):
		if iswhite(base_pixels[x, 0]):
			global start
			start = (x, 0)

def define_end():
	for x in range(width):
		if iswhite(base_pixels[x, height-1]):
			global end
			end = (x, height-1)

def iswhite(value):
    r, g, b = value
    return r > 250 and g > 250 and b > 250

def getadjacent(n):
    x,y = n
    adjacent = []
    if x - 1 >= 0 and iswhite(base_pixels[x-1, y]): adjacent.append((x - 1, y))
    if x + 1 < width and iswhite(base_pixels[x+1, y]): adjacent.append((x + 1, y))
    if y - 1 >= 0 and iswhite(base_pixels[x, y-1]): adjacent.append((x, y - 1))
    if y + 1 < height and iswhite(base_pixels[x, y+1]): adjacent.append((x, y + 1))

    return adjacent

def BFS(start, end, pixels):

    queue = Queue()
    queue.put([start]) # Wrapping the start tuple in a list
    passed.append(start)

    while not queue.empty():

        path = queue.get()
        pixel = path[-1]
        
        if pixel == end:
            return path

        for adjacent in getadjacent(pixel):
            x,y = adjacent

            if (x,y) not in passed:
            	passed.append((x,y))
            	#pixels[x,y] = (127,127,127) # see note
                new_path = list(path)
                new_path.append(adjacent)
                queue.put(new_path)

    print "Queue has been exhausted. No answer was found."


# invoke: python mazesolver.py <mazefile> <outputfile>[.jpg|.png|etc.]
base_img = Image.open(sys.argv[1])
base_pixels = base_img.load()
(width,height) = base_img.size
define_start()
define_end()
passed = []
path = BFS(start, end, base_pixels)

path_img = Image.open(sys.argv[1])
path_pixels = path_img.load()

for position in path:
    x,y = position
    path_pixels[x,y] = (0,255,0) # green

path_img.save(sys.argv[2])