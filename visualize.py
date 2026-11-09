import tkinter as tk
import turtle
import maze_solver as ms

CELL = 40
PADDING = 40
MAZE = ms.BENCHMARK_MAZE

COLOURS = {
    'wall':    "#1f2a44",  # Navy blue for walls
    'open':    "#f4f4f4",  # Light grey for open paths
    'visited': "#ffd580",  # light orange for explored nodes
    'path':    "#800080",  # purple for the final solution path
    'start':   "#008000",  # green for the starting node (S)
    'goal':    "#fa0522",  # red for the goal node (G)
    'text':    "#000000",
}


class MazeVisualizer:

    def __init__(self, rows):
        # Load the grid layout, start node, and goal node from maze_solver
        self.grid, self.start, self.goal = ms.load_grid(rows)
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])

        self.screen = turtle.Screen()
        self.screen.setup(
            width=self.cols * CELL + (2 * PADDING),
            height=self.rows * CELL + (2 * PADDING) + 60,
        )
        self.screen.title("Maze Solver - 620171166")
        self._maximize_window()


        self.pen = turtle.Turtle(visible=False)
        self.screen.tracer(0, 0)

        self._draw_base_maze()
        self.screen.update()
        self._setup_key_bindings()

    def _maximize_window(self):
        root = self.screen.getcanvas().winfo_toplevel()
        try:
            root.state("zoomed")  # Windows
        except tk.TclError:
            try:
                root.attributes("-zoomed", True)  # Linux
            except tk.TclError:
                w, h = root.winfo_screenwidth(), root.winfo_screenheight()
                root.geometry(f"{w}x{h}+0+0")
        root.update_idletasks()

    # --- Coordinate Transformation Helpers ---
    def _compute_screen_coordinates(self, r, c):
        """
        Converts matrix (row, col) indices into Cartesian
        (x, y) coordinates for turtle graphics rendering.
        """
        x = -self.cols * CELL / 2 + c * CELL + CELL / 2
        y = self.rows * CELL / 2 - r * CELL - CELL / 2
        return x, y

    def _draw_cell(self, r, c, color):
        """Fills a grid cell at (r, c) with the designated color."""

        x, y = self._compute_screen_coordinates(r, c)
        self.pen.penup()
        self.pen.goto(x - CELL / 2, y - CELL / 2)
        self.pen.setheading(0)
        self.pen.pendown()
        self.pen.color(color)
        self.pen.begin_fill()
        for _ in range(4):
            self.pen.forward(CELL)
            self.pen.left(90)
        self.pen.end_fill()

    def _draw_controls_panel(self):
        y_pos = (self.rows * CELL / 2) + 16
        controls_text = "Controls:   [1] Breadth-First   [2] Depth-First   [3] A*   [4] Greedy Best-First   [R] Reset"

        self.pen.penup()
        self.pen.goto(0, y_pos)
        self.pen.color(COLOURS['text'])
        self.pen.write(controls_text, align="center", font=("Arial", 10, "bold"))

    def _draw_base_maze(self):
        #Renders the grid layout
        for r in range(self.rows):
            for c in range(self.cols):
                color = COLOURS['wall'] if self.grid[r][c] == 1 else COLOURS['open']
                self._draw_cell(r, c, color)

        self._draw_cell(*self.start, COLOURS['start'])
        self._draw_cell(*self.goal,  COLOURS['goal'])

        self._draw_controls_panel()

    def _setup_key_bindings(self):
        s = self.screen
        s.listen()
        s.onkey(lambda: self.run_algorithm(ms.bfs), "1")
        s.onkey(lambda: self.run_algorithm(ms.dfs), "2")
        s.onkey(lambda: self.run_algorithm(ms.astar), "3")
        s.onkey(lambda: self.run_algorithm(ms.gbfs), "4")
        s.onkey(self.reset_display, "r")

    def reset_display(self):
        self.screen.tracer(0, 0)
        self._draw_base_maze()
        self.screen.update()

    def run_algorithm(self, algorithm_fn):
        self.reset_display()
        self.screen.tracer(0, 0)

        path, visited, stats = algorithm_fn(self.grid, self.start, self.goal)

        # 1. Animate the search exploration (visited nodes)
        for r, c in visited:
            if (r, c) != self.start and (r, c) != self.goal:
                self._draw_cell(r, c, COLOURS['visited'])
            self.screen.update()

        # 2. Animate the final solution path highlight
        for r, c in path:
            if (r, c) != self.start and (r, c) != self.goal:
                self._draw_cell(r, c, COLOURS['path'])
            self.screen.update()

        # 3. Re-draw start and goal markers cleanly on top of paths
        self._draw_cell(*self.start, COLOURS['start'])
        self._draw_cell(*self.goal,  COLOURS['goal'])
        self.screen.update()
        print(f"{algorithm_fn.__name__.upper()} Stats: {stats}")

    def start_main_loop(self):
        turtle.done()


def main():
    visualizer = MazeVisualizer(MAZE)
    visualizer.start_main_loop()


if __name__ == "__main__":
    main()