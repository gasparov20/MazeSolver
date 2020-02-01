# Andrew Gasparovich
# CIS 479 Project 1
# Dr. Shengquan Wang

from queue import LifoQueue


class Maze:
    def __init__(self):
        # Keeps track of the order in which nodes are visited/expanded
        self.currentIndex = 1

        # Keep track of which nodes have been visited
        self.visited = [[0, 0, 0, 0, 0, 0, 0],
                   [0, 0, -1, -1, -1, -1, 0],
                   [0, 0, -1, 0, 0, -1, 0],
                   [0, -1, -1, 0, 0, -1, 0],
                   [0, 0, 0, 0, -1, -1, 0],
                   [0, 0, 0, 0, 0, 0, 0]]

        # Keeps track of square weights
        self.maze = [[0, 0, 0, 0, 0, 0, 0],
                [0, 0, -1, -1, -1, -1, 0],
                [0, 0, -1, 0, 0, -1, 0],
                [0, -1, -1, 0, 0, -1, 0],
                [0, 0, 0, 0, -1, -1, 0],
                [0, 0, 0, 0, 0, 0, 0]]

    # Initiate DFS procedure
    def dfs(self, start, goal):
        stack = LifoQueue()
        self.dfs_util(start, goal, stack)

    # Recursive DFS
    def dfs_util(self, start, goal, stack):
        # If we find the goal, we're done!
        if start[0] == goal[0] and start[1] == goal[1]:
            return

        # Mark the current node as visited.
        self.visited[start[0]][start[1]] = 1

        # Check the west square if it has been visited (if it exists). If it has not visited
        # or assigned a weight, assign it a weight.
        if start[1] - 1 >= 0 and self.maze[start[0]][start[1] - 1] == 0 and self.visited[start[0]][start[1] - 1] == 0:
            stack.put((start[0], start[1] - 1))
            self.maze[start[0]][start[1] - 1] = self.currentIndex
            self.currentIndex += 1

        # Check the north square if it has been visited (if it exists). If it has not visited
        # or assigned a weight, assign it a weight.
        if start[0] - 1 >= 0 and self.maze[start[0] - 1][start[1]] == 0 and self.visited[start[0] - 1][start[1]] == 0:
            stack.put((start[0] - 1, start[1]))
            self.maze[start[0] - 1][start[1]] = self.currentIndex
            self.currentIndex += 1

        # Check the east square if it has been visited (if it exists). If it has not visited
        # or assigned a weight, assign it a weight.
        if start[1] + 1 <= 6 and self.maze[start[0]][start[1] + 1] == 0 and self.visited[start[0]][start[1] + 1] == 0:
            stack.put((start[0], start[1] + 1))
            self.maze[start[0]][start[1] + 1] = self.currentIndex
            self.currentIndex += 1

        # Check the south square if it has been visited (if it exists). If it has not visited
        # or assigned a weight, assign it a weight.
        if start[0] + 1 <= 5 and self.maze[start[0] + 1][start[1]] == 0 and self.visited[start[0] + 1][start[1]] == 0:
            stack.put((start[0] + 1, start[1]))
            self.maze[start[0] + 1][start[1]] = self.currentIndex
            self.currentIndex += 1

        # Pop a value off the stack. It will have the highest weight.
        x = stack.get()

        # Recurse using the coordinates of the square with the highest weight that
        # is adjacent to the current square.
        self.dfs_util(x, goal, stack)

    # A* search function
    def astar(self, start, goal):
        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, goal)
        end_node.g = end_node.h = end_node.f = 0

        # Marking the start node visited so it will end up being 00
        self.visited[start[0]][start[1]] = 1

        # Creating open and closed lists
        open_list = []
        closed_list = []

        # Start off with the start node in the open list
        open_list.append(start_node)

        # Loop until the open list is empty
        while len(open_list) > 0:
            # q will be the current node we're examining and finding adjacent nodes from
            q = open_list[0]
            q_index = 0

            # Find the node in the open list with the lowest f value and store its index
            for index, item in enumerate(open_list):
                if item.f < q.f:
                    q = item
                    q_index = index

            # Remove the lowest f value node from open list and add it to the closed list
            open_list.pop(q_index)
            closed_list.append(q)

            # If we're at the goal position, we're done
            if q.position[0] == goal[0] and q.position[1] == goal[1]:
                return

            adjacent = []  # initializing list of adjacent nodes
            for new_position in [(0, -1), (-1, 0), (0, 1), (1, 0)]:  # we're only concerned with moving W, N, E, S

                # Get adjacent node position
                node_position = (q.position[0] + new_position[0], q.position[1] + new_position[1])

                # If the adjacent node is not actually in the maze, skip it
                if node_position[0] > (len(self.maze) - 1) or node_position[0] < 0 or node_position[1] >\
                        (len(self.maze[len(self.maze) - 1]) - 1) or node_position[1] < 0:
                    continue

                # Make sure that the adjacent node is not a wall or a node that's already explored
                if self.maze[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node, keeping track of its parent, and add it to the adjacent nodes list
                new_node = Node(q, node_position)
                adjacent.append(new_node)

                # If the new node has not been visited before, assign it an index and put the index into the maze
                if self.visited[new_node.position[0]][new_node.position[1]] == 0:
                    self.maze[new_node.position[0]][new_node.position[1]] = self.currentIndex
                    new_node.id = self.currentIndex
                    self.currentIndex += 1
                    self.visited[new_node.position[0]][new_node.position[1]] = 1

            # Loop through adjacent nodes
            for new_node in adjacent:

                # If the adjacent node is already in the closed list, skip it
                for closed_child in closed_list:
                    if new_node == closed_child:
                        continue

                # Create the f, g, and h values

                # east/west is always a cost of 2 so we'll do that part first
                new_node.g = 2 * abs(new_node.position[1] - q.position[1])
                new_node.h = 2 * abs(new_node.position[1] - end_node.position[1])

                # new node is north of parent/end node, (parent -> new node) cost is 1
                if new_node.position[0] < q.position[0]:
                    new_node.g += (1 * abs(new_node.position[0] - q.position[0]))
                if new_node.position[0] > end_node.position[0]:
                    new_node.h += (1 * abs(new_node.position[0] - end_node.position[0]))

                # new node is south of parent node, (parent -> new node) cost is 3
                if new_node.position[0] > q.position[0]:
                    new_node.g += (3 * abs(new_node.position[0] - q.position[0]))
                if new_node.position[0] < end_node.position[0]:
                    new_node.h += (3 * abs(new_node.position[0] - end_node.position[0]))
                
                # finalizing f value
                new_node.g += q.g  # must add g value of parent node
                new_node.f = new_node.g + new_node.h

                # If the new node is in the open list already and the previous path is shorter, skip it
                for open_node in open_list:
                    if new_node == open_node and new_node.g > open_node.g:
                        continue

                # Add the new node to the open list and sort it in ascending order based on f value
                open_list.append(new_node)
                open_list.sort(key=take_f)
                
    def format_maze(self, start2):
        for row in range(6):
            for col in range(7):
                # Casting each grid element as a string
                self.maze[row][col] = str(self.maze[row][col])

                # Making unvisited squares empty
                if self.maze[row][col] == '0' and row != start2[0] and col != start2[1]:
                    self.maze[row][col] = '  '

                # Making single digit squares two digits for formatting
                if len(self.maze[row][col]) == 1:
                    temp = self.maze[row][col][0]
                    self.maze[row][col] = "0"
                    self.maze[row][col] += temp

                # Changing self.maze walls from -1 to ##
                if self.maze[row][col] == "-1":
                    self.maze[row][col] = "##"


# for sorting the open_list based on f value
def take_f(node):
    return node.f


# This class is necessary for A* search
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
        self.id = 0

    def __eq__(self, other):
        return self.position == other.position


# "main function"

# Instantiate a maze object to perform DFS
d = Maze()

# Instantiate a maze object to perform A* search
a = Maze()

# Setting starting and ending coordinates
start = (0, 1)
goal = (3, 4)

# Call DFS function
d.dfs(start, goal)

# Call A* function
a.astar(start, goal)

# Make the output look nice for DFS maze
d.format_maze(start)
a.format_maze(start)

print()

# Printing grids to look exactly like assignment sheet
print("DFS")
print("--------------------------")
for r in range(6):
    for c in range(7):
        print(d.maze[r][c], end="  ")
    print()

print()
print("A*")
print("--------------------------")
for r1 in range(6):
    for c1 in range(7):
        print(a.maze[r1][c1], end="  ")
    print()
