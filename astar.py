from Map import *


class Node:
    """ A node class for A* Pathfinding. Parameters are the parent of the node, the postition
    in the grid and a list of its children. In our case it is the four cells around the current
    node """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.kids = []

        # g value is the cost from start node to this node
        self.g = 0
        # h value is a heuristic function that will calculate the cost from this node to the end node
        self.h = 0
        # f value is the sum og the g and h values
        self.f = 0

    def __eq__(self, other):
        """ Function to compare two nodes, return true if their position is the same"""
        return self.position == other.position


def astar(maze, start, end, task):
    """ Returns a list of tuples as a path from the given start node to the given end node in the given maze """
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # open and closed list
    open_list = []
    closed_list = []

    # Add the start node to the open list
    open_list.append(start_node)

    # Loop until the open list is empty
    while len(open_list) > 0:

        # Get the first element from the open list
        current_node = open_list[0]
        current_index = 0
        # This will keep the open list as a priority queue so the node with the
        # lowest f value is at the start
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal and return the path from the start node to the end node
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children of the current node. It will be the positions to the left,
        # right, bottom, top. Loop through these cells
        children = []
        # This loop essentially just checks if the neighbor cells are walkable. If it is
        # then we append it on the children list at the end of this loop.
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares

            # Find the node position to the current child
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within the range of the grid, to it do not hit the edge
            if node_position[0] > (len(maze) - 1) \
                    or node_position[0] < 0 \
                    or node_position[1] > (len(maze[len(maze)-1]) -1) \
                    or node_position[1] < 0:
                continue

            # Make sure walkable terrain. -1 is defined as not walkable areas in the maps
            if maze[node_position[0]][node_position[1]] == -1:
                continue
            # Create new node to this child
            new_node = Node(current_node, node_position)

            # Put this node into the children node
            children.append(new_node)

        # Loop through children
        for child in children:

            current_node.kids.append(child)

            # If the child has not been discovered yet, then find the f value to this child, which
            # is done in attach_and_eval(), and finally add it to the open list.
            if child not in open_list and child not in closed_list:
                attach_and_eval(child, current_node, end_node, task)  # The assigment task number, to find right cost
                open_list.append(child)
            # If it is in open or closed list and it has found a better path from to this node,
            # then calculate the f value of this child again, and
            elif current_node.g + arc_cost(child, task) < child.g:
                attach_and_eval(child, current_node, end_node, task)
                propagate_path_improvements(child, task)


def attach_and_eval(child_node, parent_node, end_node, task):
    """ This will find the g, h and f values of the child node. """
    child_node.parent = parent_node

    # Create the f, g, and h values
    child_node.g = parent_node.g + arc_cost(child_node, task)
    child_node.h = abs(child_node.position[0] - end_node.position[0]) + (child_node.position[1] - end_node.position[1])
    child_node.f = child_node.g + child_node.h


def propagate_path_improvements(parent_node, task):
    """ Check if there is a better path from the parent_node to the child """
    for kid in parent_node.kids:
        # If new path to parent node plus arc cost to kid is better than the previous geodesic distance, it is replaced
        # and the change the kid's parent and update the kid's f value. Then we check the descendants recursive to
        # update the path to a better solution
        if parent_node.g + arc_cost(kid, task) < kid.g:
            kid.parent = parent_node
            kid.g = parent_node.g + arc_cost(kid, task)
            kid.f = kid.g + kid.h
            propagate_path_improvements(kid, task)


def arc_cost(child_node, task):
    # Returns cost from parent to child node. This may fore example be 1, 2, 3 or 4
    # It is defined in Map_obj, and we have to give the task to get the correct Map for this task
    map_obj = Map_Obj(task=task)
    cost = map_obj.get_cell_value(child_node.position)
    return cost


def main():
    # This loop will loop through all the task and print out a visualization of the path.
    # We have used 1000 for not walkable areas, just because it differentiate it clearly from the other cost.
    # The number 55 is the best path found
    for i in range(4):
        print("\n--------------------------------------------------------------------------------------------------\n")
        print("TASK", i+1, "\n")
        map_obj = Map_Obj(task=i+1)
        maze = map_obj.get_maps()[0]
        start = tuple(map_obj.get_start_pos())  # Get the start node
        print(f'START NODE: {start}')
        end = tuple(map_obj.get_end_goal_pos())  # Get the end node
        print(f'END NODE: {end}')

        path = astar(maze, start, end, i+1)
        print(path)

        print("The path is marked with 55")
        print("1000 marks unwalkable path \n")

        # To print a readable map, we create our own variable
        new_maze = map_obj.get_maps()[0]

        # We replace -1 with 1000 for better readability
        for k in range(len(new_maze)):
            for j in range(len(new_maze[k])):
                if new_maze[k][j] == -1:
                    new_maze[k][j] = 1000

        # This loop marks the path with the number 55
        for point in path:
            x, y = point
            new_maze[x][y] = 55

        print(new_maze)


if __name__ == '__main__':
    main()


