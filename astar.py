from Map import *

class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.kids = []

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end, task):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == -1:
                continue
            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            current_node.kids.append(child)

            if child not in open_list and child not in closed_list:
                attach_and_eval(child, current_node, end_node, task)
                open_list.append(child)
            elif current_node.g + arc_cost(child, task) < child.g:
                attach_and_eval(child, current_node, end_node, task)
                propagate_path_improvements(child, task)


def attach_and_eval(child_node, parent_node, end_node, task):
    child_node.parent = parent_node

    # Create the f, g, and h values
    child_node.g = parent_node.g + arc_cost(child_node, task)
    child_node.h = abs(child_node.position[0] - end_node.position[0]) + (child_node.position[1] - end_node.position[1])
    child_node.f = child_node.g + child_node.h


def propagate_path_improvements(parent_node, task):
    for kid in parent_node.kids:
        # If new path to parent node plus arc cost to kid is better than the previous geodesic distance, it is replaced
        if parent_node.g + arc_cost(kid, task) < kid.g:
            kid.parent = parent_node
            kid.g = parent_node.g + arc_cost(kid, task)
            kid.f = kid.g + kid.h
            propagate_path_improvements(kid, task)


def arc_cost(child_node, task):
    # Find cost from parent to child node
    map_obj = Map_Obj(task=task)
    cost = map_obj.get_cell_value(child_node.position)
    return cost


def main():
    for i in range(4):
        print("\n--------------------------------------------------------------------------------------------------\n")
        print("TASK", i+1, "\n")
        map_obj = Map_Obj(task=i+1)
        maze = map_obj.get_maps()[0]
        start = tuple(map_obj.get_start_pos())
        print(f'START NODE: {start}')
        end = tuple(map_obj.get_end_goal_pos())
        print(f'END NODE: {end}')

        path = astar(maze, start, end, i+1)
        print(path)

        print("The path is marked with 55")
        print("1000 marks unwalkable path \n")

        # To print a readable map, we create our own variable
        new_maze = map_obj.get_maps()[0]

        # We replace -1 with 1000 for better readability
        for i in range(len(new_maze)):
            for j in range(len(new_maze[i])):
                if new_maze[i][j] == -1:
                    new_maze[i][j] = 1000

        # The path is marked with the number 55
        for point in path:
            x, y = point
            new_maze[x][y] = 55

        print(new_maze)


if __name__ == '__main__':
    main()


