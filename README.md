# RoboSphere_SIM_13
Repository for solving maze problem for robot to overcome obstacles (dynamic and static) and goal point (dynamic and static).


# 🤖 AO* Robot Pathfinding Controller

This project implements an **AO\*** (And-Or graph search) based **dynamic pathfinding controller**  
for an auto-driving robot in a simulator.  

The robot:
- Connects to a Python/Flask simulation server
- Uses vision (image capture) to detect obstacles
- Builds a collision grid with clearance rules
- Plans a path using the **AO\*** search algorithm
- Smooths the path (line-of-sight optimization)
- Moves step-by-step towards the goal while avoiding collisions
- Re-plans dynamically if obstacles or goal change

# 🤖 AO* Maze Solver Robot

This project implements an **AO\*** (And-Or A*) algorithm for solving a 3D maze and controlling a robot inside a simulator.  
The robot automatically computes the path from a given **start** to a **goal** while avoiding obstacles.

---

## 📂 Project Structure

repo-root/
├── run_robot.py # Main entry-point (AO* algorithm + simulator control)
├── README.md # Instructions and usage guide
└── requirements.txt # Python dependencies



## ⚙️ Requirements

- Python **≥ 3.8**
- Dependencies (install from `requirements.txt`):
  ```bash
  pip install -r requirements.txt
▶️ Running the Robot
Start your simulator (must expose REST APIs at http://localhost:5001).

Run the controller script:

bash
Copy code
python run_robot.py
The script will:

Fetch maze/obstacle data from the simulator

Plan a valid path using AO* algorithm

Send sequential move commands (/move) to the simulator

Continue until the robot reaches the goal or no path exists

🧠 How It Works
State Capture → Uses /capture API to read the robot and goal state

Path Planning → AO* algorithm finds an optimal path in 3D grid

Move Execution → Sends /move commands step-by-step

Collision Handling → Monitors /collisions and replans if needed

📡 API Endpoints Used
GET /capture → Get current state (robot, goal, obstacles)

POST /move → Send movement command

GET /collisions → Get collision count

POST /obstacles/random → Generate random obstacles (optional)

📝 Example Run Log
text
Copy code
[INFO] Starting AO* path planning...
[INFO] Path found with 12 steps
[INFO] Moving to (x=2, y=0, z=1)
[INFO] Moving to (x=3, y=0, z=1)
[INFO] Collision detected, replanning...
[INFO] Path updated, continuing...
[INFO] Goal reached successfully! 🎉
✅ Expected Output
The robot moves through the maze

Avoids obstacles dynamically

Prints logs for each step

Exits with success when goal is reached

yaml
Copy code
