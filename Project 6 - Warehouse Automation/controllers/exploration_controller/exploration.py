from robot import Robot_Sim
from utils import *
import math
import random

def get_wheel_velocities(robbie, coord):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)
    
    Returns: 
        vr, vl: velocities of the robot's left and right wheels
    """

    # Calculate the desired change in position
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)
    dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)
    
    # Turn in place first
    angle = math.atan2(dy_robot, dx_robot)
    threshold = 0.1
    if angle < -threshold:
        return -0.01, 0.01
    elif angle > threshold:
        return 0.01, -0.01
    
    # Using desired linear velocity, set left and right wheel velocity
    linear_v = 0.05 * dist_to_coord
    w = 0.3 * math.atan2(dy_robot, dx_robot)
    vl = (linear_v - robbie.wheel_dist / 2 * w) 
    vr = (linear_v + robbie.wheel_dist / 2 * w)    
    return vr, vl


def get_neighbors(cell):
    """
    Get neighbors of a given cell
    """
    return [
        (cell[0]+1, cell[1]),
        (cell[0]-1, cell[1]),
        (cell[0], cell[1]+1),
        (cell[0], cell[1]-1)
    ]


def frontier_planning(robbie, grid):
    """
        Function for defining frontier planning.

        Arguments:
            robbie: instance of the robot
            grid: instance of the grid

        Returns:
            robbie: 'updated' instance of the robot
            OPTIONAL: robbie.next_coord: new destination coordinate

        Notes:
            The lecture notes should provide you with an ample description of frontier planning.
            You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.

    """
    ## TODO: STUDENT CODE START ##

    # Get all frontier cells based on what's adjacent to explored cells and not an obstacle ------------------------------------------
    frontier_cells = []
    for cell in robbie.explored_cells:
        # Getting all neighbors of the cell
        for neighbor in get_neighbors(cell):
            if neighbor not in robbie.explored_cells and grid.is_free(*neighbor):
                frontier_cells.append(neighbor)

    # Separate the adjacenct cells into separate frontiers ---------------------------------------------------------------------------
    frontiers = separate_adjacent_coordinates(frontier_cells, grid)
    
    # Compute the centroids of the frontiers -----------------------------------------------------------------------------------------
    centroids = [find_centroid(frontier) for frontier in frontiers]

    # Pick a centroid based on some heuristic such as sorting the centroids based on their distances to the robbie's current position
    centroids.sort(key=lambda centroid: grid_distance(centroid[0], centroid[1], robbie.x, robbie.y))

    # Choose the centroid which is not same as robot's position and the centroid is not in obstacle ----------------------------------
    for centroid in centroids:
        # If the centroid is free and not the same as the robot's current position, set it as the next coord
        if grid.is_free(*centroid) and centroid != (robbie.x, robbie.y):
            robbie.next_coord = centroid
            break

    # In case no centroid is chosen, pick a random point from the frontier -----------------------------------------------------------
    if not hasattr(robbie, 'next_coord') or robbie.next_coord is None:
        if frontier_cells:
            robbie.next_coord = random.choice(frontier_cells)
            
    ## STUDENT CODE END ##
    
    return robbie, robbie.next_coord


def exploration_state_machine(robbie, grid):
    """
    Use frontier planning, or another exploration algorithm, to explore the grid.

    Arguments:
        robbie: instance of the robot
        grid: instance of the grid

    Returns: 
        robbie: 'updated' instance of the robot

    Notes:
        Robot is considered as Point object located at the center of the traingle. 
        Robot explores the map in the discretized space
        You may use the 'rrt' function (see grid.py) to find a new path whenever the robot encounters an obstacle.
        Please note that the use of rrt slows down your code, so it should be used sparingly.
        The 'get_wheel_velocities' functions is useful in setting the robot's velocities.
        You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.
        Feel free to create other helper functions (in this file) as necessary.

    Alert:
        In this part, the task is to let the robot find all markers by exploring the map,
        which means using 'grid.markers' will lead  cause zero point on GraderScope.

    """
    ### TODO: STUDENT CODE START ###

    # Placeholder for wheel velocities (no need anymore but it doesn't matter anyway)
    # robbie.vl = 1
    # robbie.vr = 2

    # Sensing: Get the free space in robot's current FOV -----------------------------------------------------------------------------
    # Originally, there should be implementation here, but it will be used later when trying to move to the next coord
    # free_space = robbie.get_free_cells_in_fov(grid)

    # Planning: If you do not know robbie's next coordinate or have already reached it, 
    # run your choice of exploration to get robbie's next coordinate -----------------------------------------------------------------
    if robbie.next_coord is None or grid_distance(robbie.x, robbie.y, robbie.next_coord[0], robbie.next_coord[1]) <= 1:
        robbie, robbie.next_coord = frontier_planning(robbie, grid)

    # If moving to next coordinate results in a collision, then perform RRT and set that as the next coord. --------------------------
    if robbie.next_coord is not None and grid.is_collision_with_obstacles((robbie.x, robbie.y), robbie.next_coord):
        path = grid.rrt((robbie.x, robbie.y), robbie.next_coord)
        if path and len(path) > 1:
            robbie.path = path
            robbie.next_coord = path[1].xy # Set the next coord as the next point in the path
        else:
            robbie.next_coord = None # If no path is found, set next coord as None

    if robbie.next_coord is None: # If no valid next coord is found, choose a random unexplored space as the next coord
        unexplored_cells = set(grid.empty) - robbie.explored_cells
        if unexplored_cells:
            robbie.next_coord = random.choice(list(unexplored_cells))
        else:
            # /If all cells are explored, move towards a random free space
            free_cells = robbie.get_free_cells_in_fov(grid)
            if free_cells:
                robbie.next_coord = random.choice(free_cells)

    # Now that you know the next coordinate, set Robbie's wheel velocities ---------------------------------------------------------
    if robbie.next_coord is not None:
        robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.next_coord)
        # Move the robot, and if an error occurs, show error message
        try:
            robbie.move_diff_drive(grid, robbie.vl, robbie.vr, robbie.TIMESTEP)
        except Exception as e:
            print(f"Moving robot encountered an error: {e}")

    ### STUDENT CODE END ###
    return robbie