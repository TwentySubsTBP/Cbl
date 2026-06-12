# LAB RUN — Ubuntu Lab PC guide (NO Docker)

This guide runs the 3 acts on the lab computer (Ubuntu, ROS 2 Jazzy installed
natively). **Follow it step by step, in order. Do not skip steps.**

> **For Windows/Docker:** see `RUN.md`.
> This file is ONLY for the lab PC.

---

## RULES (READ BEFORE STARTING)

1. **Every time you open a new terminal**, run the source commands below — otherwise you get `ros2: command not found`
2. Only the `my_tb3_world` folder goes into `src/` — put nothing else there
3. **Do NOT clone the whole repo into `src/`** — copy only the `my_tb3_world` folder
4. Make sure `setup.py` says `package_name = 'my_tb3_world'` (NO trailing S)

---

## A) CLEANUP — start from scratch (5 minutes)

If you tried anything before, clean up FIRST. If this is your first time, run
it anyway — it does no harm:

```bash
# 1. Delete old build artifacts
rm -rf /home/team06/turtlebot3_ws/build
rm -rf /home/team06/turtlebot3_ws/install
rm -rf /home/team06/turtlebot3_ws/log

# 2. Delete WRONG files inside src (repo leftovers)
rm -rf "/home/team06/turtlebot3_ws/src/Simulation Files (1)"
rm -rf /home/team06/turtlebot3_ws/src/Cbl
rm -rf /home/team06/turtlebot3_ws/src/docs
rm -f  /home/team06/turtlebot3_ws/src/README.md
rm -f  /home/team06/turtlebot3_ws/src/RUN.md
rm -f  /home/team06/turtlebot3_ws/src/LAB_RUN.md

# 3. Delete the old my_tb3_world too (we will copy a fresh one)
rm -rf /home/team06/turtlebot3_ws/src/my_tb3_world

# 4. Delete the old clone
rm -rf /home/team06/Cbl

# 5. Clear broken environment variables
unset COLCON_PREFIX_PATH
```

---

## B) CLONE AND COPY (2 minutes)

```bash
# 1. Clone the repo into the HOME folder (NOT into src!)
cd /home/team06
git clone https://github.com/TwentySubsTBP/Cbl.git
```

> If it asks for a password: NOT your GitHub password — you need a Personal
> Access Token. github.com > Settings > Developer settings >
> Personal access tokens > Generate

```bash
# 2. Copy ONLY the my_tb3_world folder into the workspace
cp -r /home/team06/Cbl/my_tb3_world /home/team06/turtlebot3_ws/src/
```

**CHECK — src must contain only this:**
```bash
ls /home/team06/turtlebot3_ws/src/
```
Expected output:
```
my_tb3_world
```
If anything else is there (docs, README.md, Simulation Files, Cbl, etc.) it is
WRONG. Run the cleanup steps above again.

---

## C) SETUP.PY CHECK (30 seconds)

Do NOT skip this step. This is where the error was last time.

```bash
head -5 /home/team06/turtlebot3_ws/src/my_tb3_world/setup.py
```

Expected output:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_tb3_world'
```

> **IF IT SAYS `my_tb3_worlds` (trailing S)**, fix it:
> ```bash
> sed -i "s/package_name = 'my_tb3_worlds'/package_name = 'my_tb3_world'/" /home/team06/turtlebot3_ws/src/my_tb3_world/setup.py
> ```

---

## D) BUILD (2 minutes)

```bash
# 1. Source ROS
source /opt/ros/jazzy/setup.bash

# 2. Go to the workspace
cd /home/team06/turtlebot3_ws

# 3. Build
colcon build
```

**Expected output:**
```
Starting >>> my_tb3_world
Finished <<< my_tb3_world [X.XXs]
Summary: 1 package finished [X.XXs]
```

> **IF YOU GET AN ERROR:**
> - `Duplicate package names` → Go back to cleanup, there are extra folders in `src/`
> - `package directory does not exist` → Go back to the setup.py check
> - `colcon: command not found` → You forgot `source /opt/ros/jazzy/setup.bash`

```bash
# 4. Source the build output
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# 5. Check the package is visible
ros2 pkg list | grep my_tb3
```

Expected output:
```
my_tb3_world
```

---

## E) RUNNING — you need 3 terminals

### IMPORTANT: Run these 3 lines FIRST in every terminal

```bash
source /opt/ros/jazzy/setup.bash
cd /home/team06/turtlebot3_ws && source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Without these lines you get `ros2: command not found`.

---

### T1 — Gazebo world

```bash
ros2 launch my_tb3_world new_world.launch.py
```
Wait ~40 seconds, the Gazebo window will open.

---

### T2 — Twin nodes (open a new terminal)

Run the source commands first (above), then:

```bash
# check odom is alive (must print numbers):
ros2 topic echo --once --qos-reliability best_effort /odom
```

If numbers came through:

```bash
ros2 launch my_tb3_world waverider.launch.py start_world:=false hazard_x:=0.8 hazard_y:=0.0
```

---

### T3 — Control terminal (open a new terminal)

Run the source commands first (above), then run the acts.

---

## ACT 1 — State sync (pH leak) [RED zone in Gazebo]

```bash
# 1) Open the alert listener FIRST (in the background). /alerts publishes ONE
#    message per event — if you subscribe AFTER the trigger you MISS it and
#    the echo waits forever:
ros2 topic echo --once /alerts &
sleep 1

# 2) trigger the leak:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: true}"
# the background echo now prints the RAISED message

# 3) extra evidence (/mode publishes continuously, works any time):
ros2 topic echo --once --field data /mode

# 4) reset:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"
```

Expected: `RAISED` on alerts, `ALERT` in mode, red zone in Gazebo.

---

## ACT 2 — Hazard re-route [BLUE zone in Gazebo, robot curves around it]

**Make sure Act 1 is reset FIRST** (otherwise the robot will not move):

```bash
# 0) check the leak is off:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"

# 1) PARK THE ROBOT AT HOME FIRST (NO hazard yet). The robot drives itself to
#    (1.5, 0) right after launch — meaning it has already PASSED the hazard
#    spot (0.8). Sending it to 1.8 from there SHOWS NO DETOUR. Go home first:
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"
#    ... WAIT until the robot STOPS at (0,0).

# 2) spawn the hazard (x=0.8, now BETWEEN the robot and the goal):
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: true}"

# 3) send the robot beyond the hazard (QoS flags REQUIRED) -> it curves around:
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 1.8, y: 0.0}, orientation: {w: 1.0}}}"

# 4) reset (once the robot reaches the goal):
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: false}"
```

> Sending the SAME goal twice DOES NOTHING (go_to_goal ignores duplicate
> coordinates). To show it again, alternate goals: home (0,0) -> far (1.8,0).

---

## ACT 3 — Comms safety halt (twin dies, robot stops)

```bash
# make the robot drive:
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"

# check the twin is alive:
ros2 topic echo --once --field data /twin_alive
# -> must be True

# kill the twin:
pkill -9 -f lib/my_tb3_world/dt_supervisor

# wait 5 seconds, the robot STOPS, then:
ros2 topic echo --once --field data /twin_alive
# -> must be False
```

**IMPORTANT — bring the twin BACK after Act 3:**
```bash
ros2 run my_tb3_world dt_supervisor --ros-args -p cmd_vel_topic:=cmd_vel_override
```
> Run this in a separate terminal (if you run it in T3, T3 stays busy).
> The robot will NOT move until the twin is back.

---

## AUTONOMOUS PATROL (optional)

If you want the robot to survey the map automatically:

```bash
# the twin must be alive FIRST (twin_alive = True)
# In a new terminal (don't forget the source commands):
ros2 run my_tb3_world sector_nav
```

The robot patrols a 2x2 grid automatically. You can trigger anomalies from
another terminal while it patrols.

---

## ERROR GUIDE

| Error | Cause | Fix |
|------|-------|-------|
| `ros2: command not found` | ROS not sourced | `source /opt/ros/jazzy/setup.bash` |
| `Package 'my_tb3_world' not found` | Install not sourced or wrong folder | `cd /home/team06/turtlebot3_ws && source install/setup.bash` |
| `Duplicate package names` | Two copies of `my_tb3_world` in `src/` | Delete the old copy: `rm -rf "src/Simulation Files (1)"` |
| `package directory 'my_tb3_worlds' does not exist` | Typo in setup.py | `sed -i "s/my_tb3_worlds/my_tb3_world/" src/my_tb3_world/setup.py` |
| `git clone` password error | GitHub does not accept passwords | Use a Personal Access Token |
| Robot does not move (cmd_vel = 0) | twin_alive False or mode ALERT | 1) Check `ros2 topic echo --once --field data /twin_alive`. If False, restart the supervisor. 2) Check `ros2 topic echo --once --field data /mode`. If ALERT, reset the pH |
| `/odom` empty / Gazebo frozen | Gazebo crash | Close T1, run `ros2 launch my_tb3_world new_world.launch.py` again |
| `COLCON_PREFIX_PATH` warning | Old broken environment variable | `unset COLCON_PREFIX_PATH` or open a new terminal |

---

## FOLDER STRUCTURE (MUST LOOK LIKE THIS)

```
/home/team06/
├── Cbl/                          <-- repo cloned HERE (NOT into src)
│   ├── my_tb3_world/
│   ├── docs/
│   ├── RUN.md
│   ├── LAB_RUN.md
│   └── README.md
│
└── turtlebot3_ws/                <-- workspace
    ├── src/
    │   └── my_tb3_world/         <-- ONLY this folder lives here
    │       ├── launch/
    │       ├── my_tb3_world/     <-- Python node files (.py)
    │       ├── worlds/
    │       ├── resource/
    │       ├── test/
    │       ├── package.xml
    │       ├── setup.py          <-- package_name = 'my_tb3_world' (NO S!)
    │       └── setup.cfg
    ├── build/                    <-- created by colcon build
    ├── install/                  <-- created by colcon build
    └── log/                      <-- created by colcon build
```

> **WRONG structures:**
> - `src/Cbl/my_tb3_world/` — whole repo cloned into src
> - `src/Simulation Files (1)/` — old copy
> - `src/README.md` — repo files do not belong in src
