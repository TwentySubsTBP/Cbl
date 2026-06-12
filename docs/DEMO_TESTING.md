# WaveRider — 3-Act demo: how to run each test (step by step)

The three graded capabilities, each as a runnable test. Verified in the Gazebo
sim. Use 3 terminals: **T1 = world**, **T2 = system nodes**, **T3 = triggers/evidence**.

---

## 0. One-time setup (on the lab/school computer)

```bash
# (host WSL) get the code onto the lab machine and into the workspace
cd /home/c2irr10/turtlebot3_ws/src
git clone https://github.com/TwentySubsTBP/Cbl.git        # or: cd Cbl && git pull
cp -r Cbl/my_tb3_world /home/c2irr10/turtlebot3_ws/src/    # overwrite the package

# open Docker Desktop, then start the container:
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=:0 -e HOME=/tmp \
  -e TURTLEBOT3_MODEL=burger -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
# inside the container:
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Helper to open more terminals INTO the same container (use for T2, T3):
```bash
docker exec -it turtlebot3_container bash
cd /ws && source install/setup.bash && export TURTLEBOT3_MODEL=burger
```

---

## 1. Start the world and the system

**T1 — Gazebo world** (inside the container from setup):
```bash
ros2 launch my_tb3_world new_world.launch.py
```
Wait ~40 s for Gazebo to open.

**Check /odom is live** (the one gotcha — if dead, restart T1):
```bash
ros2 topic echo --once --qos-reliability best_effort /odom   # should print a pose
```

**T2 — all the twin nodes** (one command, hazard placed at x=0.8 for Act 2):
```bash
ros2 launch my_tb3_world waverider.launch.py start_world:=false hazard_x:=0.8 hazard_y:=0.0
```
Confirm 10 nodes are up: `ros2 node list`

---

## 2. ACT 1 — State sync (pH leak)  [FR1.1 + FR4 + NFR3]

**What it shows:** a sudden pH drop is detected, mirrored across the twin
(mode → ALERT), drawn in Gazebo (red zone), and emitted as a structured alert.

**T3:**
```bash
# watch the evidence (leave running):
ros2 topic echo /alerts &
ros2 topic echo --field data /mode &

# >>> trigger the leak:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: true}"
```
**Watch:**
- Gazebo: a **red zone** appears at the robot.
- `/mode` → `"mode": "ALERT"`, `"ph": 6.0`.
- `/alerts` → `{"state": "RAISED", ... "reason": "sudden pH drop ..."}`.

**Reset:**
```bash
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"   # red zone disappears, /alerts CLEARED
```

---

## 3. ACT 2 — Environmental interaction (hazard re-route)  [FR2 + FR3 + NFR3]

**What it shows:** a current/storm exists ONLY in the twin (blue zone, no physical
collision), yet the robot re-routes around it = the twin changes the robot's path.

**T3:**
```bash
# 1) put the leak away first if Act 1 is still on:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"

# 2) park the robot at home FIRST (no hazard yet). The robot auto-drives to
#    (1.5, 0) right after startup, which is already past the hazard spot, so
#    a goal at 1.8 from there would show no detour:
#    (/goal_pose is transient_local -> the -w 1 + --qos-durability flags are REQUIRED)
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"
#    ... wait until the robot stops at (0, 0).

# 3) spawn the hazard (at x=0.8, now between the robot and the goal):
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: true}"

# 4) send the robot to a goal BEYOND the hazard:
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.8, y: 0.0}, orientation: {w: 1.0}}}"
```
**Watch:**
- Gazebo: a **blue zone** at (0.8, 0); the robot drives, **curves around it**, and
  still reaches the goal (it does not go straight through).

**Reset:**
```bash
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: false}"   # blue zone disappears
```
*Tip:* re-sending the SAME goal does nothing (go_to_goal ignores duplicate
coordinates) — to drive the pass again, alternate goals: home (0,0) then far (1.8,0).

---

## 4. ACT 3 — Comms safety halt (NFR2)

**What it shows:** if the robot loses the twin for >5 s, it stops (fail-safe).

**T3:**
```bash
# make the robot drive first (transient_local QoS flags REQUIRED):
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo --field data /twin_alive &     # watch: True while twin is up

# >>> cut the twin: kill dt_supervisor (the heartbeat source)
#     simplest from T3:
pkill -9 -f lib/my_tb3_world/dt_supervisor
```
**Watch (within ~5 s):**
- `/twin_alive` → `False`.
- The **robot stops** (comms_watchdog asserts a stop on /cmd_vel via cmd_mux).
- T2 log: `comms_watchdog ... COMMS LOST -- HALTING robot`.

**Reset:** restart the twin in T2 (or re-run `ros2 run my_tb3_world dt_supervisor`) →
`/twin_alive` → True, `COMMS OK`.

---

## 5. Record evidence (optional but recommended)

In a spare terminal, before running the acts:
```bash
ros2 bag record /water_quality /mode /alerts /latency_ms /sync_status /sync_error_m /hazard_zone /sector /obstacle_info /twin_alive /cmd_vel /odom /scan
```
This bag is your NFR1/FR4 evidence and a backup demo.

---

## 6. Troubleshooting (from the course decks)

| Symptom | Fix |
|---------|-----|
| `/odom` empty / nodes idle | Sim stalled — restart T1 (`new_world.launch.py`). |
| `docker: command not found` | Docker Desktop crashed — reopen it, clear stale sockets, retry. |
| Robot does not move | Is `/cmd_vel` arriving? Is a navigator (go_to_goal) running? |
| Two things fight over motion | Only `cmd_mux` should publish `/cmd_vel` (check `ros2 topic info /cmd_vel` → 1 publisher). |
| Nothing on `/water_quality` | `ph_sensor` waits for first `/odom` — fix `/odom` first. |
