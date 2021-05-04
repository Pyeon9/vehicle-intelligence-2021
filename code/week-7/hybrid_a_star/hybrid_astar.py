import numpy as np

class HybridAStar:
    # Determine how many grid cells to have for theta-axis.
    NUM_THETA_CELLS = 90

    # Define min, max, and resolution of steering angles
    omega_min = -35
    omega_max = 35
    omega_step = 5

    # A very simple bicycle model
    speed = 0.5
    length = 0.5

    # Initialize the search structure.
    def __init__(self, dim):
        self.dim = dim
        self.closed = np.zeros(self.dim, dtype=np.int)
        self.came_from = np.full(self.dim, None)
        
    def State(self, x, y, theta, g, f):
        state = {
                    'f': f,
                    'g': g,
                    'x': x,
                    'y': y,
                    't': theta,
                }
        return state

    # Expand from a given state by enumerating reachable states.
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []

        # Consider a discrete selection of steering angles.
        for delta_t in range(self.omega_min, self.omega_max + 1, self.omega_step):
            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.
            omega = self.speed / self.length * np.tan(np.deg2rad(delta_t))
            next_x = x + self.speed * np.cos(theta)
            next_y = y + self.speed * np.sin(theta)
            next_theta = (theta + omega) % (2*np.pi)
            
            if 0 <= self.idx(next_x) < self.dim[1] and 0 <= self.idx(next_y) < self.dim[2]:
                next_f = g2 + self.heuristic(next_x, next_y, goal)
                next_state = self.State(next_x, next_y, next_theta, g2, next_f)
                next_states.append(next_state)
            
        return next_states
    

    # Perform a breadth-first search based on the Hybrid A* algorithm.
    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close(mark as 1) the initial cell and record the starting state 
        # for the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.
        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for n in next_states:
                idx_x, idx_y = self.idx(n['x']), self.idx(n['y'])
                stack = self.theta_to_stack_num(n['t'])

                # 맵 상의 갈 수 있는 곳인가
                if grid[idx_x, idx_y] == 0:
                    # 이전 지점과 다음 지점을 잇는 경로 상에 장애물(못가는 곳)이 있는지 체크
                    # 즉, 충돌 조건 체크
                    dist_x, dist_y = abs(self.idx(x) - idx_x), abs(self.idx(y) - idx_y)
                    min_x, min_y = min(self.idx(x), idx_x), min(self.idx(y), idx_y)
                    possible = True
                    for d_x in range(dist_x + 1):
                        for d_y in range(dist_y + 1):
                            if grid[min_x + d_x, min_y + d_y] != 0:
                                possible = False
                    # 갈 수 있는 경로이며 닫히지 않은 상태라면 업데이트            
                    if possible and self.closed[stack][idx_x][idx_y] == 0:
                        self.closed[stack][idx_x][idx_y] = 1
                        total_closed += 1
                        self.came_from[stack][idx_x][idx_y] = curr
                        opened.append(n)
                
        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        # TODO: implement a function that calculate the stack number
        # given theta represented in radian. Note that the calculation
        # should partition 360 degrees (2 * PI rad) into different
        # cells whose number is given by NUM_THETA_CELLS.
        deg = theta * 180 / np.pi
        interval = 360 / self.NUM_THETA_CELLS
        stack_num = deg // interval 
        stack_num = stack_num if not stack_num == self.NUM_THETA_CELLS else 0
        
        return int(stack_num)

    # Calculate the index of the grid cell based on the vehicle's position.
    def idx(self, pos):
        # We simply assume that each of the grid cell is the size 1 X 1.
        return int(np.floor(pos))

    # Implement a heuristic function to be used in the hybrid A* algorithm.
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        L1_dist = abs(goal[0]-x) + abs(goal[1]-y)
        return L1_dist

    # Reconstruct the path taken by the hybrid A* algorithm.
    def reconstruct_path(self, start, goal):
        # Start from the final state, and follow the link to the
        # previous state using the came_from matrix.
        curr = self.final
        x, y = curr['x'], curr['y']
        path = []
        while x != start[0] and y != start[1]:
            path.append(curr)
            stack = self.theta_to_stack_num(curr['t'])
            x, y = curr['x'], curr['y']
            curr = self.came_from[stack][self.idx(x)][self.idx(y)]
        # Reverse the path so that it begins at the starting state
        # and ends at the final state.
        path.reverse()
        return path
