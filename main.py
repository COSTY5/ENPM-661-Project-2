import numpy as np
import matplotlib.pyplot as plt
import time
import heapq

class Node:

    def __init__(self, x, y, cost, parent_node):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_node = parent_node

    # To support Node Comparision
    def __lt__(self, other):
        return self.cost < other.cost


#Functions to move the robot
def move_u(x, y, cost):
    y += 1
    cost += 1
    return x, y, cost

def move_d(x, y, cost):
    y -= 1
    cost += 1
    return x, y, cost

def move_l(x, y, cost):
    x -= 1
    cost += 1
    return x, y, cost

def move_r(x, y, cost):
    x += 1
    cost += 1
    return x, y, cost

def move_ur(x, y, cost):
    x += 1
    y += 1
    cost += np.sqrt(2)
    return x, y, cost

def move_dr(x, y, cost):
    x += 1
    y -= 1
    cost += np.sqrt(2)
    return x, y, cost

def move_ul(x, y, cost):
    x -= 1
    y += 1
    cost += np.sqrt(2)
    return x, y, cost

def move_dl(x, y, cost):
    x -= 1
    y -= 1
    cost += np.sqrt(2)
    return x, y, cost

#A function that moves the robots
def move_bot(move, x, y, cost):
    if move == 'Up':
        return move_u(x, y, cost)
    elif move == 'UpRight':
        return move_ur(x, y, cost)
    elif move == 'Right':
        return move_r(x, y, cost)
    elif move == 'DownRight':
        return move_dr(x, y, cost)
    elif move == 'Down':
        return move_d(x, y, cost)
    elif move == 'DownLeft':
        return move_dl(x, y, cost)
    elif move == 'Left':
        return move_l(x, y, cost)
    elif move == 'UpLeft':
        return move_ul(x, y, cost)
    else:
        return None

#A function that generates the map
def create_map(width, height):
    # Generating Obstacle Space
    ws = np.full((height, width), 0)

    for y in range(0, height):
        for x in range(0, width):

            r11_buffer = (x + 5) - 100
            r12_buffer = (y - 5) - 100
            r13_buffer = (x - 5) - 150


            r21_buffer = (x + 5) - 100
            r23_buffer = (x - 5) - 150
            r24_buffer = (y + 5) - 150

            h6_buffer = (y + 5) + 0.58 * (x + 5) - 223.18
            h5_buffer = (y + 5) - 0.58 * (x - 5) + 123.21
            h4_buffer = (x - 6.5) - 364.95
            h3_buffer = (y - 5) + 0.58 * (x - 5) - 373.21
            h2_buffer = (y - 5) - 0.58 * (x + 5) - 26.82
            h1_buffer = (x + 6.5) - 235.040

            t1_buffer = (x + 5) - 460
            t2_buffer = (y - 5) + 2 * (x - 5) - 1145
            t3_buffer = (y + 5) - 2 * (x - 5) + 895

            if ((
                    h6_buffer > 0 and h5_buffer > 0 and h4_buffer < 0 and h3_buffer < 0 and h2_buffer < 0 and h1_buffer > 0) or (
                    r11_buffer > 0 and r12_buffer < 0 and r13_buffer < 0) or (
                    r21_buffer > 0 and r23_buffer < 0 and r24_buffer > 0) or (
                    t1_buffer > 0 and t2_buffer < 0 and t3_buffer > 0)):
                ws[y, x] = 1


            r11 = (x) - 100
            r12 = (y) - 100
            r13 = (x) - 150

            r21 = (x) - 100
            r23 = (x) - 150
            r24 = (y) - 150

            h6 = (y) + 0.58 * (x) - 223.18
            h5 = (y) - 0.58 * (x) + 123.21
            h4 = (x) - 364.95
            h3 = (y) + 0.58 * (x) - 373.21
            h2 = (y) - 0.58 * (x) - 26.82
            h1 = (x) - 235.04

            t1 = (x) - 460
            t2 = (y) + 2 * (x) - 1145
            t3 = (y) - 2 * (x) + 895

            if ((h6 > 0 and h5 > 0 and h4 < 0 and h3 < 0 and h2 < 0 and h1 > 0) or (
                    r11 > 0 and r12 < 0 and r13 < 0) or (r21 > 0 and r23 < 0 and r24 > 0) or (
                    t1 > 0 and t2 < 0 and t3 > 0)):
                ws[y, x] = 2

    return ws


#Checking if the goal has reached
def goal_checker(current, goal):
    if (current.x == goal.x) and (current.y == goal.y):
        return True
    else:
        return False


#Checking the validity of the move
def validity_checker(x, y, ws):
    size = ws.shape

    if (x > size[1] or x < 0 or y > size[0] or y < 0):
        return False
    else:
        try:
            if (ws[y][x] == 1) or (ws[y][x] == 2):
                return False
        except:
            pass
    return True


#A function that generates an unique identification number for every node
def unique_id(node):
    id = 3333 * node.x + 113 * node.y
    return id


#A function for implenenting Dijkstra algorithm
def dijkstra(start, goal, ws):
    if goal_checker(start, goal):
        return None, 1

    goal_node = goal
    start_node = start

    ue_node = {}
    e_node = {}
    all_nodes = []
    openlist = []

    # A list of all the permitted moves
    possible_moves = ['Up', 'UpRight', 'Right', 'DownRight', 'Down', 'DownLeft', 'Left', 'UpLeft']

    #Allocating a unique identification for every node
    start_key = unique_id(start_node)
    ue_node[(start_key)] = start_node

    heapq.heappush(openlist, [start_node.cost, start_node])

    #A loop that runs until all the Open List nodes have been explored
    while (len(openlist) != 0):

        current_node = (heapq.heappop(openlist))[1]
        all_nodes.append([current_node.x, current_node.y])
        current_id = unique_id(current_node)

        if goal_checker(current_node, goal_node):
            goal_node.parent_node = current_node.parent_node
            goal_node.cost = current_node.cost
            print("Goal Node found")
            return all_nodes, 1

        if current_id in e_node:
            continue
        else:
            e_node[current_id] = current_node

        del ue_node[current_id]

        for moves in possible_moves:

            x, y, cost = move_bot(moves, current_node.x, current_node.y, current_node.cost)

            new_node = Node(x, y, cost, current_node)
            new_node_id = unique_id(new_node)

            if not validity_checker(new_node.x, new_node.y, ws):
                continue
            elif new_node_id in e_node:
                continue

            if new_node_id in ue_node:
                if new_node.cost < ue_node[new_node_id].cost:
                    ue_node[new_node_id].cost = new_node.cost
                    ue_node[new_node_id].parent_node = new_node.parent_node
            else:
                ue_node[new_node_id] = new_node
            heapq.heappush(openlist, [new_node.cost, new_node])

    return all_nodes, 0


#A function for tracing the optimal path created for reaching the goal from start point
def path_tracer(goal_node):
    backtrack_stack = []
    backtrack_stack.append(goal_node)

    while backtrack_stack[-1].parent_node != -1:
        parent = backtrack_stack[-1].parent_node
        backtrack_stack.append(parent)

    backtrack_stack.reverse()

    x_trace = [node.x for node in backtrack_stack]
    y_trace = [node.y for node in backtrack_stack]

    return x_trace, y_trace

class PathPlotter:
    def __init__(self, start_node, goal_node, ws):
        self.start_node = start_node
        self.goal_node = goal_node
        self.ws = ws

        ## Plot the Start and Goal Positions
        plt.plot(start_node.x, start_node.y, "Db")
        plt.plot(goal_node.x, goal_node.y, "Dg")

        ## Plot Map
        plt.imshow(ws, "seismic")
        self.ax = plt.gca()
        self.ax.invert_yaxis()

    # Plot the explored nodes
    def plot_e_node(self, e_node):
        # Plotting the Explored Nodes
        for i in range(len(e_node)):
            plt.plot(e_node[i][0], e_node[i][1], "3y")

    def plot_path(self, x_trace, y_trace):
        plt.plot(x_trace, y_trace, "--r")
        plt.show()
        plt.pause(3)
        plt.close('all')


# Main Function
if __name__ == '__main__':

    #Workspace width and Height
    width = 600
    height = 250

    #Coordinates of Obstacles
    ws = create_map(width, height)

    #Start and Goal Coordinates from the User
    start_x = int(input("Enter Start Position's X Coordinate: "))
    start_y = int(input("Enter Start Position's Y Coordinate: "))

    goal_x = int(input("Enter Goal Position's X Coordinate: "))
    goal_y = int(input("Enter Goal Position's Y Coordinate: "))

    #Starting timer for noting the Time of Execution
    start_time = time.time()

    #Checking the validity of the start node in the workspace
    if not validity_checker(start_x, start_y, ws):
        print("Enter Start Node within the permitted Robot Space")
        exit(-1)

    #Checking the validaty of the start node in the workspace
    if not validity_checker(goal_x, goal_y, ws):
        print("Enter Goal Node within the permitted Robot Space")
        exit(-1)

    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    #Implementing the Dijkstra Algorithm
    e_node, goal_status = dijkstra(start_node, goal_node, ws)

    #Backtracking
    if (goal_status == 1):
        x_trace, y_trace = path_tracer(goal_node)

        #Plotting the optimal path
        plotter = PathPlotter(start_node, goal_node, ws)
        plotter.plot_e_node(e_node)
        plotter.plot_path(x_trace, y_trace)
        cost = goal_node.cost
        print(f"Cost of reaching the goal: {cost:.2f}")
    else:
        print("Goal could not be planned for the given points")

    end_time = time.time()
    print(f"Time Taken to Execute the Program is: {end_time - start_time}")

