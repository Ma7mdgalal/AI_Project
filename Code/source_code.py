import tkinter as tk
from tkinter import messagebox
import time
import random
import heapq
from collections import deque

# =============================================================================
# 1. ALGORITHMS CLASS
# =============================================================================

class SearchAlgorithms:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.rows = len(grid)
        self.cols = len(grid[0])

    def get_neighbors(self, node):
        r, c = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and self.grid[nr][nc] == 0:
                neighbors.append((nr, nc))
        return neighbors

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # --- 1. BFS ---
    def bfs(self):
        queue = deque([(self.start, [self.start])])
        visited = set([self.start])
        nodes_expanded = 0
        while queue:
            current, path = queue.popleft()
            nodes_expanded += 1
            if current == self.goal:
                return path, nodes_expanded
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None, nodes_expanded

    # --- 2. DFS ---
    def dfs(self):
        stack = [(self.start, [self.start])]
        visited = set([self.start])
        nodes_expanded = 0
        while stack:
            current, path = stack.pop()
            nodes_expanded += 1
            if current == self.goal:
                return path, nodes_expanded
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append((neighbor, path + [neighbor]))
        return None, nodes_expanded

    # --- 3. UCS ---
    def ucs(self):
        pq = [(0, self.start, [self.start])]
        visited = set()
        nodes_expanded = 0
        while pq:
            cost, current, path = heapq.heappop(pq)
            if current in visited: continue
            visited.add(current)
            nodes_expanded += 1
            if current == self.goal:
                return path, nodes_expanded
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + 1, neighbor, path + [neighbor]))
        return None, nodes_expanded

    # --- 4. IDS ---
    def ids(self):
        def dls(node, path, depth, visited_cycle):
            nonlocal nodes_expanded
            nodes_expanded += 1
            if node == self.goal:
                return path
            if depth <= 0:
                return None
            for neighbor in self.get_neighbors(node):
                if neighbor not in visited_cycle:
                    result = dls(neighbor, path + [neighbor], depth - 1, visited_cycle | {neighbor})
                    if result: return result
            return None

        max_depth = self.rows * self.cols
        nodes_expanded = 0
        for depth in range(max_depth):
            result = dls(self.start, [self.start], depth, {self.start})
            if result:
                return result, nodes_expanded
        return None, nodes_expanded

    # --- 5. A* Search ---
    def astar(self):
        pq = [(0 + self.heuristic(self.start, self.goal), self.start, [self.start])]
        visited = set()
        g_score = {self.start: 0}
        nodes_expanded = 0
        while pq:
            _, current, path = heapq.heappop(pq)
            if current == self.goal:
                return path, nodes_expanded
            if current in visited: continue
            visited.add(current)
            nodes_expanded += 1
            for neighbor in self.get_neighbors(current):
                new_g = g_score[current] + 1
                if neighbor not in g_score or new_g < g_score[neighbor]:
                    g_score[neighbor] = new_g
                    f_score = new_g + self.heuristic(neighbor, self.goal)
                    heapq.heappush(pq, (f_score, neighbor, path + [neighbor]))
        return None, nodes_expanded

# =============================================================================
# 2. GUI APPLICATION
# =============================================================================

class MazeApp:
    def __init__(self, root, size=10):
        self.root = root
        self.root.title("Team 8 - AI Maze Solver")
        self.size = size
        self.cell_size = 30
        self.grid = []
        
        self.main_frame = tk.Frame(root)
        self.main_frame.pack(padx=10, pady=10)
        
        self.canvas = tk.Canvas(self.main_frame, width=size*self.cell_size, height=size*self.cell_size, bg='white', highlightthickness=1, highlightbackground="black")
        self.canvas.pack(side=tk.LEFT)
        
        self.sidebar = tk.Frame(self.main_frame)
        self.sidebar.pack(side=tk.RIGHT, padx=10, fill=tk.Y)
        
        tk.Label(self.sidebar, text="Control Panel", font=("Arial", 12, "bold")).pack(pady=5)
        tk.Button(self.sidebar, text="Generate New Maze", command=self.generate_maze, bg="#dddddd").pack(fill=tk.X, pady=5)
        
        tk.Label(self.sidebar, text="Select Algorithm:", font=("Arial", 10, "bold")).pack(pady=(15,5))
        
        # القائمة بعد حذف الخوارزميتين
        self.algos = [
            ("BFS", "bfs"),
            ("DFS", "dfs"),
            ("UCS", "ucs"),
            ("IDS", "ids"),
            ("A* Search", "astar")
        ]
        
        for name, func_name in self.algos:
            tk.Button(self.sidebar, text=name, command=lambda f=func_name, n=name: self.run_algo(f, n)).pack(fill=tk.X, pady=2)
            
        self.result_label = tk.Label(self.sidebar, text="Status: Ready", fg="blue", justify=tk.LEFT)
        self.result_label.pack(pady=20)
        
        self.generate_maze()

    def generate_maze(self):
        # نسبة الحوائط 0.2 عشان الحل يطلع أسهل
        self.grid = [[1 if random.random() < 0.2 else 0 for _ in range(self.size)] for _ in range(self.size)]
        self.grid[0][0] = 0
        self.grid[self.size-1][self.size-1] = 0
        self.solver = SearchAlgorithms(self.grid, (0,0), (self.size-1, self.size-1))
        self.draw_grid()
        self.result_label.config(text="Status: New Maze Generated", fg="blue")

    def draw_grid(self, path=None):
        self.canvas.delete("all")
        for r in range(self.size):
            for c in range(self.size):
                color = 'white'
                if self.grid[r][c] == 1: color = '#404040'
                elif (r, c) == (0,0): color = '#00cc00'
                elif (r, c) == (self.size-1, self.size-1): color = '#cc0000'
                self.canvas.create_rectangle(c*self.cell_size, r*self.cell_size,
                                           (c+1)*self.cell_size, (r+1)*self.cell_size,
                                           fill=color, outline='#e0e0e0')
        if path:
            if len(path) > 1:
                coords = []
                for r, c in path:
                    coords.append(c*self.cell_size + self.cell_size/2)
                    coords.append(r*self.cell_size + self.cell_size/2)
                self.canvas.create_line(coords, fill="blue", width=3)

    def run_algo(self, func_name, algo_name):
        self.draw_grid()
        self.root.update()
        start_time = time.time()
        algo_func = getattr(self.solver, func_name)
        path, nodes = algo_func()
        elapsed_time = (time.time() - start_time) * 1000
        
        if path:
            self.draw_grid(path)
            res = f"Algorithm: {algo_name}\nPath Cost: {len(path)-1}\nNodes Exp.: {nodes}\nTime: {elapsed_time:.2f} ms"
            self.result_label.config(text=res, fg="green")
        else:
            res = f"Algorithm: {algo_name}\nResult: Failed\nNodes Exp.: {nodes}"
            self.result_label.config(text=res, fg="red")

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("800x600")
    app = MazeApp(root)
    root.mainloop()