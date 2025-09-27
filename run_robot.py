import time
import requests
import websocket
import json
import threading
import math
import heapq
from typing import List, Tuple, Optional

# --- CONFIG ---
FLASK_URL = "http://localhost:5000"
WS_URL = "ws://localhost:8080"
GOALS = ["NE", "SE", "SW", "NW"]  # sequence of corners to visit
MOVE_DELAY = 2.5  # seconds to wait for movement completion
GRID_SIZE = 3.0  # pathfinding grid resolution
ROBOT_RADIUS = 1.5  # robot collision radius
OBSTACLE_RADIUS = 3.5  # obstacle avoidance radius

# --- GLOBAL VARIABLES ---
ws_msgs = []
robot_pos = {"x": 0, "y": 0, "z": 0}
goal_reached = False
collision_detected = False
ws_connected = False

# --- WEBSOCKET HANDLER ---
def on_message(ws, message):
    global robot_pos, goal_reached, collision_detected
    try:
        data = json.loads(message)
        ws_msgs.append(data)
        
        if data.get("type") == "collision":
            print(f"🔴 [COLLISION] Detected at: ({data['position']['x']:.1f}, {data['position']['z']:.1f})")
            collision_detected = True
            robot_pos = data["position"]
            
        elif data.get("type") == "goal_reached":
            print(f"🎯 [GOAL REACHED] at: ({data['position']['x']:.1f}, {data['position']['z']:.1f})")
            goal_reached = True
            robot_pos = data["position"]
            
        elif data.get("type") == "confirmation":
            if "Arrived at target" in data.get("message", ""):
                robot_pos = data.get("position", robot_pos)
                print(f"📍 Robot position: ({robot_pos['x']:.1f}, {robot_pos['z']:.1f})")
                
    except Exception as e:
        print(f"Error parsing WebSocket message: {e}")

def on_error(ws, error):
    print(f"❌ WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    global ws_connected
    ws_connected = False
    print("🔌 WebSocket connection closed")

def on_open(ws):
    global ws_connected
    ws_connected = True
    print("✅ WebSocket connected to simulator")

# Initialize WebSocket
ws = websocket.WebSocketApp(
    WS_URL,
    on_open=on_open,
    on_message=on_message,
    on_error=on_error,
    on_close=on_close
)

# Start WebSocket in background thread
ws_thread = threading.Thread(target=ws.run_forever)
ws_thread.daemon = True
ws_thread.start()

# Wait for connection
print("🔄 Connecting to WebSocket...")
for i in range(10):
    if ws_connected:
        break
    time.sleep(0.5)

if not ws_connected:
    print("❌ Failed to connect to WebSocket")
    exit(1)

# --- NODE CLASS FOR A* ---
class Node:
    def __init__(self, x: float, z: float):
        self.x = x
        self.z = z
        self.g = 0  # Cost from start
        self.h = 0  # Heuristic cost to goal
        self.f = 0  # Total cost
        self.parent = None
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return abs(self.x - other.x) < 1.0 and abs(self.z - other.z) < 1.0
    
    def __hash__(self):
        return hash((round(self.x), round(self.z)))

# --- PATHFINDING FUNCTIONS ---
def get_obstacles():
    """Get default obstacle positions"""
    return [
        {"x": 10, "z": 0}, {"x": -10, "z": -10}, {"x": 0, "z": 10},
        {"x": 15, "z": 5}, {"x": -12, "z": 12}, {"x": 5, "z": -15},
        {"x": -8, "z": -5}, {"x": 20, "z": 20}, {"x": -18, "z": -3},
        {"x": 13, "z": -7}, {"x": -7, "z": 8}, {"x": 18, "z": -10},
        {"x": -5, "z": 17}, {"x": 12, "z": 13}, {"x": -16, "z": -14},
        {"x": 3, "z": -12}, {"x": -14, "z": 0}, {"x": 7, "z": 16},
        {"x": -20, "z": 10}, {"x": 0, "z": -20}, {"x": 4, "z": 4}
    ]

def is_collision(x: float, z: float, obstacles: List[dict]) -> bool:
    """Check if position collides with obstacles"""
    for obs in obstacles:
        dist = math.sqrt((x - obs["x"])**2 + (z - obs["z"])**2)
        if dist < (ROBOT_RADIUS + OBSTACLE_RADIUS):
            return True
    return False

def is_within_bounds(x: float, z: float) -> bool:
    """Check if position is within floor bounds"""
    return -45 <= x <= 45 and -45 <= z <= 45

def heuristic(node1: Node, node2: Node) -> float:
    """Calculate heuristic distance"""
    return math.sqrt((node1.x - node2.x)**2 + (node1.z - node2.z)**2)

def get_neighbors(node: Node, obstacles: List[dict]) -> List[Node]:
    """Get valid neighbor nodes"""
    neighbors = []
    # 8-directional movement
    directions = [
        (0, GRID_SIZE), (0, -GRID_SIZE),
        (GRID_SIZE, 0), (-GRID_SIZE, 0),
        (GRID_SIZE, GRID_SIZE), (GRID_SIZE, -GRID_SIZE),
        (-GRID_SIZE, GRID_SIZE), (-GRID_SIZE, -GRID_SIZE)
    ]
    
    for dx, dz in directions:
        new_x = node.x + dx
        new_z = node.z + dz
        
        if (is_within_bounds(new_x, new_z) and 
            not is_collision(new_x, new_z, obstacles)):
            neighbors.append(Node(new_x, new_z))
    
    return neighbors

def a_star_pathfind(start_pos: dict, goal_pos: dict, obstacles: List[dict]) -> List[Node]:
    """A* pathfinding algorithm"""
    start = Node(start_pos["x"], start_pos["z"])
    goal = Node(goal_pos["x"], goal_pos["z"])
    
    # Snap to grid
    start.x = round(start.x / GRID_SIZE) * GRID_SIZE
    start.z = round(start.z / GRID_SIZE) * GRID_SIZE
    goal.x = round(goal.x / GRID_SIZE) * GRID_SIZE
    goal.z = round(goal.z / GRID_SIZE) * GRID_SIZE
    
    open_set = []
    closed_set = set()
    
    heapq.heappush(open_set, start)
    
    while open_set:
        current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct path
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]  # Reverse to get start-to-goal path
        
        closed_set.add(current)
        
        for neighbor in get_neighbors(current, obstacles):
            if neighbor in closed_set:
                continue
            
            tentative_g = current.g + heuristic(current, neighbor)
            
            # Check if neighbor is in open_set
            neighbor_in_open = None
            for i, node in enumerate(open_set):
                if node == neighbor:
                    neighbor_in_open = node
                    break
            
            if neighbor_in_open is None:
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current
                heapq.heappush(open_set, neighbor)
            elif tentative_g < neighbor_in_open.g:
                neighbor_in_open.g = tentative_g
                neighbor_in_open.f = neighbor_in_open.g + neighbor_in_open.h
                neighbor_in_open.parent = current
    
    return []  # No path found

# --- API FUNCTIONS ---
def set_goal(corner: str):
    """Set goal using corner name"""
    try:
        resp = requests.post(f"{FLASK_URL}/goal", json={"corner": corner}, timeout=5)
        if resp.status_code == 200:
            result = resp.json()
            print(f"🎯 Goal set to {corner}: ({result['goal']['x']}, {result['goal']['z']})")
            return result["goal"]
        else:
            print(f"❌ Failed to set goal: {resp.text}")
    except Exception as e:
        print(f"❌ API error setting goal: {e}")
    return None

def move_robot(x: float, z: float):
    """Move robot to specific coordinates"""
    try:
        resp = requests.post(f"{FLASK_URL}/move", json={"x": x, "z": z}, timeout=5)
        if resp.status_code == 200:
            print(f"➡️  Moving to ({x:.1f}, {z:.1f})")
            return True
        else:
            print(f"❌ Move command failed: {resp.text}")
    except Exception as e:
        print(f"❌ API error moving robot: {e}")
    return False

def reset_robot():
    """Reset robot position and collision counter"""
    try:
        resp = requests.post(f"{FLASK_URL}/reset", timeout=5)
        if resp.status_code == 200:
            global robot_pos, collision_detected, goal_reached
            robot_pos = {"x": 0, "y": 0, "z": 0}
            collision_detected = False
            goal_reached = False
            print("🔄 Robot reset to origin")
            return True
    except Exception as e:
        print(f"❌ API error resetting robot: {e}")
    return False

def get_collision_count():
    """Get current collision count"""
    try:
        resp = requests.get(f"{FLASK_URL}/collisions", timeout=5)
        if resp.status_code == 200:
            return resp.json().get("count", 0)
    except Exception as e:
        print(f"❌ Error getting collision count: {e}")
    return 0

# --- NAVIGATION FUNCTIONS ---
def navigate_with_astar(goal_pos: dict, obstacles: List[dict]) -> bool:
    """Navigate to goal using A* pathfinding"""
    global robot_pos, collision_detected, goal_reached
    
    print(f"🗺️  Planning path from ({robot_pos['x']:.1f}, {robot_pos['z']:.1f}) to ({goal_pos['x']:.1f}, {goal_pos['z']:.1f})")
    
    # Find path using A*
    path = a_star_pathfind(robot_pos, goal_pos, obstacles)
    
    if not path:
        print("❌ No path found!")
        return False
    
    print(f"✅ Path found with {len(path)} waypoints")
    
    # Execute path
    for i, waypoint in enumerate(path[1:], 1):  # Skip start position
        if goal_reached:
            print("🎯 Goal reached during navigation!")
            return True
            
        print(f"📍 Waypoint {i}/{len(path)-1}: ({waypoint.x:.1f}, {waypoint.z:.1f})")
        
        if not move_robot(waypoint.x, waypoint.z):
            print("❌ Failed to send move command")
            return False
        
        # Wait for movement completion
        time.sleep(MOVE_DELAY)
        
        # Check for collisions
        if collision_detected:
            print("🔴 Collision detected during navigation!")
            collision_detected = False  # Reset flag
            return False
    
    # Final check if we reached the goal
    final_dist = math.sqrt((robot_pos["x"] - goal_pos["x"])**2 + (robot_pos["z"] - goal_pos["z"])**2)
    if final_dist < 5.0 or goal_reached:
        return True
    
    return False

def navigate_to_corner(corner: str, obstacles: List[dict], max_retries: int = 3) -> bool:
    """Navigate to a corner with retry logic"""
    global goal_reached, collision_detected
    
    # Set goal
    goal_pos = set_goal(corner)
    if not goal_pos:
        return False
    
    # Try navigation with retries
    for attempt in range(max_retries):
        print(f"🚀 Navigation attempt {attempt + 1}/{max_retries} to {corner}")
        
        # Reset flags
        goal_reached = False
        collision_detected = False
        
        # Try A* navigation
        if navigate_with_astar(goal_pos, obstacles):
            print(f"✅ Successfully reached {corner}!")
            return True
        
        if attempt < max_retries - 1:
            print(f"⚠️  Attempt {attempt + 1} failed, retrying...")
            time.sleep(1)
    
    print(f"❌ Failed to reach {corner} after {max_retries} attempts")
    return False

# --- MAIN EXECUTION ---
def main():
    print("🤖 Robot Navigator with A* Pathfinding")
    print("=" * 60)
    
    # Reset robot
    print("🔄 Resetting robot...")
    reset_robot()
    time.sleep(2)
    
    # Get obstacles
    obstacles = get_obstacles()
    print(f"📦 Loaded {len(obstacles)} obstacles")
    
    # Navigate to each goal
    success_count = 0
    start_time = time.time()
    
    for i, corner in enumerate(GOALS, 1):
        print(f"\n{'='*20} GOAL {i}/{len(GOALS)}: {corner} {'='*20}")
        
        if navigate_to_corner(corner, obstacles):
            success_count += 1
            print(f"✅ Goal {i} completed successfully!")
        else:
            print(f"❌ Goal {i} failed!")
            
        # Brief pause between goals
        if i < len(GOALS):
            print("⏸️  Brief pause before next goal...")
            time.sleep(2)
    
    # Final statistics
    total_time = time.time() - start_time
    final_collisions = get_collision_count()
    
    print(f"\n{'='*60}")
    print("🏁 NAVIGATION COMPLETE!")
    print(f"✅ Successfully reached: {success_count}/{len(GOALS)} goals")
    print(f"🔴 Total collisions: {final_collisions}")
    print(f"⏱️  Total time: {total_time:.1f} seconds")
    
    if success_count == len(GOALS):
        print("🎉 All goals completed successfully!")
    else:
        print("⚠️  Some goals were not reached")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️  Navigation interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        print("👋 Goodbye!")
