from pyamaze import maze, agent, textLabel
from queue import PriorityQueue

def h(cell1, cell2):
    
    x1, y1 = cell1  # Unpack first cell coordinates (row, col)
    x2, y2 = cell2  # Unpack second cell coordinates
    return abs(x1-x2) + abs(y1-y2)  # Sum of vertical and horizontal distances

def a_star(m):
    
    # Define start (bottom-right) and goal (top-left) positions
    start = (m.rows, m.cols)
    goal = (1,1)
    
    # Initialize g_scores (actual cost from start to each cell)
    g_score = {cell: float('inf') for cell in m.grid}  # Start with infinity
    g_score[start] = 0  # Cost from start to itself is 0
    
    # Initialize f_scores (estimated total cost: g_score + heuristic)
    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = h(start, goal)  # Initial f_score for start cell
    
    # Priority queue for open cells to explore, ordered by f_score
    open = PriorityQueue()
    open.put((f_score[start], h(start, goal), start))
    
    # Dictionary to store the path (child: parent relationships)
    a_path = {}
    
    # Main A* algorithm loop
    #continues as long as there are unexplored cells.
    while not open.empty():
        # Get cell with lowest f_score from priority queue
        current_cell = open.get()[2]
        
        # Stop if we've reached the goal
        if current_cell == goal:
            break
        
        # Check all four possible directions (East, North, South, West)
        for d in 'ENSW':
            # If there's no wall in this direction
            if m.maze_map[current_cell][d]:
                # Calculate child cell coordinates based on direction
                if d == 'E':  # Move right
                    child_cell = (current_cell[0], current_cell[1]+1)
                elif d == 'W':  # Move left
                    child_cell = (current_cell[0], current_cell[1]-1)
                elif d == 'N':  # Move up
                    child_cell = (current_cell[0]-1, current_cell[1])
                elif d == 'S':  # Move down
                    child_cell = (current_cell[0]+1, current_cell[1])
                
                # Calculate tentative scores
                temp_g_score = g_score[current_cell] + 1  # Each step costs 1
                temp_f_score = temp_g_score + h(child_cell, goal)  # g + h
                
                # If we found a better path to this child cell
                if temp_f_score < f_score[child_cell]:
                    # Update scores and path
                    g_score[child_cell] = temp_g_score
                    f_score[child_cell] = temp_f_score
                    # Add to priority queue
                    open.put((temp_f_score, h(child_cell, goal), child_cell))
                    # Record how we got to this child cell
                    a_path[child_cell] = current_cell
    
    # Reconstruct the path from goal back to start
    inversePath = {}
    cell = goal  # Start at goal
    while cell != start:
        # Map each cell to its successor in the path
        inversePath[a_path[cell]] = cell
        # Move to parent cell
        cell = a_path[cell]
    
    return inversePath


m = maze(10, 10)  
m.CreateMaze(1, 1, loopPercent=100) 


path = a_star(m)  

a = agent(m, footprints=True) 
m.tracePath({a: path}) 
l = textLabel(m, "A* Path Length: ", len(path)+1)  

m.run()