# run_robot.py
import time
import requests
import websocket
import json

# --- CONFIG ---
FLASK_URL = "http://localhost:5000"
GOALS = ["NE", "NW", "SE", "SW"]  # sequence of corners to visit
RETRY_DELAY = 1  # seconds

# --- WEBSOCKET HANDLER ---
ws_msgs = []

def on_message(ws, message):
    data = json.loads(message)
    ws_msgs.append(data)
    if data.get("type") == "collision":
        print("[WebSocket] Collision detected at:", data["position"])
    if data.get("type") == "goal_reached":
        print("[WebSocket] Goal reached at:", data["position"])

def on_error(ws, error):
    print("WebSocket error:", error)

def on_close(ws, close_status_code, close_msg):
    print("WebSocket closed")

def on_open(ws):
    print("WebSocket connected")

# Connect WebSocket
ws = websocket.WebSocketApp(
    "ws://localhost:8080",
    on_open=on_open,
    on_message=on_message,
    on_error=on_error,
    on_close=on_close
)
import threading
ws_thread = threading.Thread(target=ws.run_forever)
ws_thread.daemon = True
ws_thread.start()
time.sleep(1)  # give some time to connect

# --- UTILS ---
def set_goal(goal):
    resp = requests.post(f"{FLASK_URL}/goal", json={"corner": goal})
    if resp.status_code == 200:
        print(f"[API] Goal set to {goal}")
    else:
        print("[API] Failed to set goal:", resp.json())

def move_to(x, z):
    resp = requests.post(f"{FLASK_URL}/move", json={"x": x, "z": z})
    if resp.status_code != 200:
        print("[API] Move command failed:", resp.json())

# --- MAIN PATH ---
try:
    for g in GOALS:
        set_goal(g)
        reached = False
        while not reached:
            # Ask simulator for goal coords
            # Here we just send move command to center of goal box as placeholder
            # For simplicity, using corner coordinates like in server
            coords_map = {"NE": (45, -45), "NW": (-45, -45), "SE": (45, 45), "SW": (-45, 45)}
            x, z = coords_map[g]
            move_to(x, z)
            
            # Wait for goal_reached message
            time.sleep(RETRY_DELAY)
            for msg in ws_msgs:
                if msg.get("type") == "goal_reached":
                    reached = True
                    ws_msgs.clear()
                    break
except KeyboardInterrupt:
    print("Interrupted by user")

print("✅ Auto-drive script finished successfully")
