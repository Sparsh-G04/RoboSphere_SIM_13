import requests
import time
import heapq
import math
from PIL import Image
import base64
import io

# --- Configuration ---
API_BASE_URL = "http://localhost:5001"
CANVAS_WIDTH = 650
CANVAS_HEIGHT = 600
ROBOT_RADIUS = 18
OBSTACLE_SIZE = 25
CELL_SIZE = 20
OBSTACLE_CLEARANCE = (OBSTACLE_SIZE / 2) + ROBOT_RADIUS + 5


# --- API Client ---
class APIClient:
    def __init__(self, base_url):
        self.base_url = base_url

    def get_current_state(self):
        try:
            response = requests.get(f"{self.base_url}/capture", timeout=3)
            if response.status_code == 200:
                return response.json()
            return None
        except requests.exceptions.RequestException:
            return None

    def get_collision_count(self):
        try:
            response = requests.get(f"{self.base_url}/collisions", timeout=1)
            if response.status_code == 200:
                return response.json().get("count", 0)
        except requests.exceptions.RequestException:
            return 0
        return 0

    def move_robot_to(self, x, y):
        try:
            requests.post(f"{self.base_url}/move", json={"x": x, "y": y}, timeout=2)
            print(f"➡️ Sent move command to: ({int(x)}, {int(y)})")
        except requests.exceptions.RequestException as e:
            print(f"Error sending move command: {e}")

    def set_random_obstacles(self, count=15):
        try:
            response = requests.post(f"{self.base_url}/obstacles/random", json={"count": count})
            if response.status_code == 200:
                print(f"🟦 Successfully set {count} random obstacles.")
                return response.json().get("obstacles", [])
        except requests.exceptions.RequestException as e:
            print(f"Error setting obstacles: {e}")
        return []


# --- AO* Algorithm Node ---
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# --- AO* Search (And-Or extension of A*) ---
def ao_star_search(grid, start, end):
    start_node = Node(start)
    goal_node = Node(end)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        current_node = heapq.heappop(open_list)[1]

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == goal_node:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        (x, y) = current_node.position
        neighbors = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        for move_x, move_y in neighbors:
            new_pos = (x + move_x, y + move_y)
            if (0 <= new_pos[0] < len(grid[0]) and
                0 <= new_pos[1] < len(grid) and
                grid[new_pos[1]][new_pos[0]] == 0):

                neighbor = Node(new_pos, current_node)
                step_cost = 1.414 if move_x != 0 and move_y != 0 else 1
                neighbor.g = current_node.g + step_cost
                neighbor.h = heuristic(neighbor.position, goal_node.position)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, (neighbor.f, neighbor))

    return None


# --- Grid Helpers ---
def create_grid(obstacles):
    grid_width = CANVAS_WIDTH // CELL_SIZE
    grid_height = CANVAS_HEIGHT // CELL_SIZE
    grid = [[0 for _ in range(grid_width)] for _ in range(grid_height)]

    for obs in obstacles:
        for r in range(grid_height):
            for c in range(grid_width):
                cx = c * CELL_SIZE + CELL_SIZE / 2
                cy = r * CELL_SIZE + CELL_SIZE / 2
                distance = math.sqrt((cx - obs['x'])**2 + (cy - obs['y'])**2)
                if distance < OBSTACLE_CLEARANCE:
                    grid[r][c] = 1
    return grid


def pixel_to_grid(pos):
    return (int(pos['x'] / CELL_SIZE), int(pos['y'] / CELL_SIZE))


def grid_to_pixel(pos):
    return (pos[0] * CELL_SIZE + CELL_SIZE / 2, pos[1] * CELL_SIZE + CELL_SIZE / 2)


# --- Vision ---
def detect_obstacles_from_image(base64_data):
    header, encoded = base64_data.split(",", 1)
    image_data = base64.b64decode(encoded)
    image = Image.open(io.BytesIO(image_data)).convert("RGB")
    pixels = image.load()
    width, height = image.size
    obstacles, visited = [], set()

    for x in range(0, width, 10):
        for y in range(0, height, 10):
            if (x, y) in visited:
                continue
            r, g, b = pixels[x, y]
            if r + g + b < 200:
                blob, q = [], [(x, y)]
                visited.add((x, y))
                while q:
                    px, py = q.pop()
                    blob.append((px, py))
                    for nx in range(px-10, px+11, 10):
                        for ny in range(py-10, py+11, 10):
                            if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                                nr, ng, nb = pixels[nx, ny]
                                if nr + ng + nb < 200:
                                    visited.add((nx, ny))
                                    q.append((nx, ny))
                if blob:
                    avg_x = sum(p[0] for p in blob) / len(blob)
                    avg_y = sum(p[1] for p in blob) / len(blob)
                    obstacles.append({'x': avg_x, 'y': avg_y})

    print(f"👁️ Detected {len(obstacles)} obstacles.")
    return obstacles


# --- Main Controller ---
def run_controller():
    api = APIClient(API_BASE_URL)
    print("🤖 AO* Controller Initializing...")

    initial_state = None
    while not initial_state:
        initial_state = api.get_current_state()
        if not initial_state:
            print("Waiting for simulator...")
            time.sleep(1)

    obstacles = detect_obstacles_from_image(initial_state['image_data'])
    grid = create_grid(obstacles)

    obstacles = api.set_random_obstacles(15)
    if obstacles:
        grid = create_grid(obstacles)

    last_goal, last_collisions = None, api.get_collision_count()

    while True:
        state = api.get_current_state()
        if not state or state.get('status') != 'success':
            print("Waiting for simulator...")
            time.sleep(2)
            continue

        robot_pos = state['robot_position']
        goal_pos = state['goal_position']
        collisions = api.get_collision_count()

        if goal_pos != last_goal or collisions > last_collisions:
            print(f"🎯 New planning request. Goal: {goal_pos}")
            last_goal, last_collisions = goal_pos, collisions

            start_grid, end_grid = pixel_to_grid(robot_pos), pixel_to_grid(goal_pos)
            path = ao_star_search(grid, start_grid, end_grid)

            if path:
                print(f"✅ Path with {len(path)} steps found.")
                for waypoint in path[1:]:
                    tx, ty = grid_to_pixel(waypoint)
                    api.move_robot_to(tx, ty)
                    time.sleep(0.5)
            else:
                print("❌ No path found.")
        else:
            time.sleep(0.5)


if __name__ == "__main__":
    run_controller()
