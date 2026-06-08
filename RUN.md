# RUN — copy-paste demo guide (WaveRider digital twin)

Run the 3 graded acts. **Do the blocks in order, top to bottom.** Each block says
which terminal it belongs to. You need 3 terminals.

> ⚠️ The Act 1/2/3 trigger commands alone are NOT enough — you must run
> **A) setup** and **B) start** first, then the acts work.

---

## A) SETUP — one time on the lab computer

**T1 (host WSL terminal):**
```bash
# 1. get the latest code into the workspace
cd /home/c2irr10
git clone https://github.com/TwentySubsTBP/Cbl.git 2>/dev/null || (cd Cbl && git pull)
cp -r /home/c2irr10/Cbl/my_tb3_world /home/c2irr10/turtlebot3_ws/src/

# 2. open Docker Desktop (the app), then start the container:
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=:0 -e HOME=/tmp \
  -e TURTLEBOT3_MODEL=burger -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
```
**Now you are INSIDE the container (T1). Build once:**
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## B) START — every time you demo

**T1 (still inside the container) — start Gazebo world:**
```bash
ros2 launch my_tb3_world new_world.launch.py
```
Wait ~40s for Gazebo to open.

**T2 (new host WSL terminal) — enter the container and start all twin nodes:**
```bash
docker exec -it turtlebot3_container bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
# check odom is alive (must print a number):
ros2 topic echo --once --qos-reliability best_effort /odom
# start the whole system (hazard placed at x=0.8 for Act 2):
ros2 launch my_tb3_world waverider.launch.py start_world:=false hazard_x:=0.8 hazard_y:=0.0
```

**T3 (new host WSL terminal) — your control/trigger terminal:**
```bash
docker exec -it turtlebot3_container bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```
Run the acts below from **T3**.

---

## ACT 1 — State sync (pH leak)   [watch Gazebo: RED zone]
```bash
# trigger the leak:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: true}"
# evidence:
ros2 topic echo --once /alerts                 # -> {"state":"RAISED", "ph":6.0, ...}
ros2 topic echo --once --field data /mode      # -> "mode":"ALERT"
# reset:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"
```

## ACT 2 — Environmental interaction (hazard re-route)   [watch Gazebo: BLUE zone, robot curves around]
```bash
# make sure no leak is active:
ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"
# spawn the twin-only hazard (at x=0.8):
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: true}"
# send the robot to a goal BEYOND the hazard -> it re-routes around it.
# NOTE: /goal_pose uses transient_local QoS, so the flags below are REQUIRED:
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 1.8, y: 0.0}, orientation: {w: 1.0}}}"
# reset:
ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: false}"
```

## ACT 3 — Comms safety halt (lose the twin -> robot stops in 5s)
```bash
# make the robot drive (transient_local QoS flags are REQUIRED):
ros2 topic pub --once -w 1 --qos-durability transient_local /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo --once --field data /twin_alive   # -> True
# cut the twin (kill the heartbeat):
pkill -9 -f lib/my_tb3_world/dt_supervisor
# within ~5s: robot STOPS, and:
ros2 topic echo --once --field data /twin_alive   # -> False
# IMPORTANT: bring the twin back BEFORE doing anything else (Acts, patrol, etc.)
# The robot will NOT move while twin_alive is False.
ros2 run my_tb3_world dt_supervisor --ros-args -p cmd_vel_topic:=cmd_vel_override
```

> **After Act 3:** Always restart the supervisor above before continuing.
> Run it in a **separate terminal** (e.g. T3) so it stays alive in the
> background while you use other terminals for patrol or further testing.

---

## Autonomous patrol (optional)

Instead of sending manual goals, you can let the robot patrol the map
autonomously using sector_nav. Run in **T3** (after setup + start):
```bash
ros2 run my_tb3_world sector_nav
```
The robot will patrol a 2x2 grid in snake order, looping forever. You can
trigger anomalies from **T4** while it patrols.

> **Important:** If you ran Act 3 before this, `twin_alive` will be `False` and
> the robot will refuse to move. Restart the supervisor first:
> ```bash
> ros2 run my_tb3_world dt_supervisor --ros-args -p cmd_vel_topic:=cmd_vel_override
> ```

---

## Record evidence (optional, run in a spare T4 before the acts)
```bash
docker exec -it turtlebot3_container bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
ros2 bag record /water_quality /mode /alerts /latency_ms /hazard_zone /sector /obstacle_info /twin_alive /cmd_vel /odom /scan
```

## If something breaks

- **`ros2: command not found`** → You forgot to source ROS. Run this in every new terminal inside the container:
  ```bash
  cd /ws
  source /opt/ros/jazzy/setup.bash
  source /opt/turtlebot3_ws/install/setup.bash
  source install/setup.bash
  export TURTLEBOT3_MODEL=burger
  ```
- **`git clone` fails with password prompt** → GitHub no longer accepts passwords. Use a [Personal Access Token](https://github.com/settings/tokens) (repo scope) as the password, or use `gh auth login` + `gh repo clone`.
- **Robot won't move / cmd_vel is all zeros** → Check these in order:
  1. `ros2 topic echo --once --field data /twin_alive` — if `False`, the supervisor was killed (Act 3). Restart it:
     ```bash
     ros2 run my_tb3_world dt_supervisor --ros-args -p cmd_vel_topic:=cmd_vel_override
     ```
  2. `ros2 topic echo --once --field data /mode` — if `ALERT`, reset the pH anomaly first:
     ```bash
     ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"
     ```
  3. `ros2 topic info /cmd_vel` — should show **1 publisher** (cmd_mux).
- **`/odom` empty / nothing happens** → Gazebo stalled: restart **T1** (`new_world.launch.py`).
- **`docker: command not found`** → Docker Desktop crashed: reopen the app and retry.
- **Act 2: robot doesn't move toward goal** → Make sure mode is not ALERT and twin_alive is True before sending the goal.

Full explanation: see `docs/DEMO_TESTING.md`.
