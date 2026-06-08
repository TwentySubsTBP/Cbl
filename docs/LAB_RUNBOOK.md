# WaveRider — Lab session runbook (avoid errors on the day)

Sourced from the course decks (Week 1/2/6 + lab overview) and our verified sim
setup. Goal: walk into the lab prepared, with a guaranteed fallback.

## 0. The big picture (from Slide 38 — DDS discovery)

The course's intended Option-A architecture is **same ROS domain, DDS auto-discovery** —
NOT a custom bridge:

```
Robot SBC (Raspberry Pi):  bringup -> publishes /scan, /odom   (LiDAR, OpenCR)
Lab Laptop / container:    our nodes (dt_supervisor, ph_sensor, ...) + Gazebo
                           -> publishes /cmd_vel back to the robot
Discovery: same ROS_DOMAIN_ID + same Wi-Fi  => nodes find each other automatically
```

So our nodes use the standard topics (`/scan`, `/odom`, `/cmd_vel`) and should work
against the real robot **with no new code**, *provided* domain + network + bringup
are correct. No `twin_safety_node` / custom bridge is strictly required for Option A.

## 1. MUST DO BEFORE THE LAB (tonight)

- [ ] **Read the Canvas doc "Connecting lab laptop to lab robot.pdf"** — it is
      required reading and has the *exact* connection steps (ROS_DOMAIN_ID value,
      Wi-Fi, the robot bringup command). We do NOT have this file; get it from Canvas.
- [ ] **Transfer our code to the lab laptop** (Canvas has a "create packages +
      transfer" instruction). Our branch: `feat/digital-twin-sim-nodes`.
      `git clone`/`pull`, copy `src/my_tb3_world` into the workspace, `colcon build`.
- [ ] **Smoke-test the all-sim demo on the lab laptop** so you KNOW it runs there:
      `ros2 launch my_tb3_world waverider.launch.py`, then run the 3 acts once.
- [ ] Confirm `/odom` is live before starting nodes (our known stall gotcha; restart
      the world if dead).

## 2. Decide the path (Slide 25)

- **Option A — physical robot + Gazebo twin.** Highest rubric, needs the live
  connection to work. This is what going to the lab is for.
- **Option B — no physical robot.** Two software entities mirror each other.
  **Our current system already satisfies Option B** (bidirectional pub/sub + state
  sync via `/mode` + environmental interaction via hazards). Option B requires a
  written email to the lecturer and means not attending the lab.

**Plan: attempt Option A in the lab, but keep the all-sim run as the guaranteed backup.**

## 3. Connect to the real robot (Option A) — order matters

1. **Robot bringup first** (on the SBC, per the Canvas PDF) — without it there is no
   `/scan` or `/odom`.
2. **Same ROS_DOMAIN_ID** on the laptop and the robot (set via env var — allowed; do
   NOT change Wi-Fi/system settings, which the lab rules forbid):
   `export ROS_DOMAIN_ID=<value from the PDF>`
3. **Verify discovery from the laptop:**
   ```
   ros2 topic list            # expect /scan /odom /cmd_vel /tf from the robot
   ros2 topic echo /scan      # data arriving?
   ros2 topic hz /odom
   ```
4. **AVOID THE TOPIC CLASH (critical).** Do NOT run the Gazebo sim robot publishing
   the SAME `/odom` and subscribing the SAME `/cmd_vel` as the real robot — they will
   fight. For Option A, the **real robot is the source of `/scan` `/odom`** and the
   destination of `/cmd_vel`; Gazebo is the *mirror* (initialise its start pose from
   the real robot — Gazebo Tutorial). If you must run both, separate with namespaces.
5. **use_sim_time:** our nodes default to real time (`use_sim_time=false`) — correct
   when the data source is the real robot. Only the Gazebo side uses sim time.

## 4. If something breaks — the course's own first checks (Slide 12/13)

| Symptom | First checks |
|---------|--------------|
| Robot does not move | Is `/cmd_vel` arriving? Battery ok? `turtlebot3_node`/bringup running on SBC? |
| LiDAR missing on `/scan` | LDS driver launched? `TURTLEBOT3_MODEL` set? |
| Sim moves, real robot does NOT | **Wrong ROS_DOMAIN_ID? Missing robot bringup? Topic namespace clash?** |
| Twin desynchronizes | QoS mismatch? Topic namespace collision? Bridge not running? |
| RViz no data | Correct Fixed Frame? `/tf` publishing? `use_sim_time` consistent? |

Diagnostic order: node running (`ros2 node list`) → topic exists (`ros2 topic list`)
→ data arriving (`ros2 topic echo/hz`) → frames (`ros2 run tf2_tools view_frames`).

## 5. Map our 3 acts onto the real robot

- **Act 1 — state sync (FR1.1/FR4):** `ph_sensor` runs on the laptop reading the real
  robot's `/odom`; trigger `/ph_anomaly` → `/alerts` RAISED + `mode=ALERT` + red zone
  in the Gazebo mirror. (No physical pH sensor — dummy data is explicitly allowed.)
- **Act 2 — bidirectional + env (FR2/FR3):** spawn a hazard in the twin → `go_to_goal`
  publishes `/cmd_vel` → **the real robot re-routes**. Communication B→A shown.
- **Act 3 — NFR2:** kill `dt_supervisor` (or the link) → `comms_watchdog` stops the
  real robot within 5 s.

## 6. Evidence to capture (cheap, and a safe fallback)

Rosbag is the course-recommended evidence + backup (Slide 37):

```
ros2 bag record /scan /odom /tf /cmd_vel /water_quality /mode /alerts /latency_ms /hazard_zone /sector
```

- **State-sync error (Slide 15):** log `/odom` from BOTH the real robot and the Gazebo
  mirror at the same time — the difference IS your measured sync error (great evidence).
- Even if live control is flaky, **recording the real robot's `/scan` `/odom` is easy
  and is valid evidence** — then replay in Gazebo to show the twin loop.

## 7. Guaranteed fallback (do this no matter what)

Record the **all-sim 3-act run** (already verified working) + a clean rosbag. The
recording plan's "Option B all-sim version still shows all 3 acts" is an accepted path.
That way you leave the lab with a complete demo even if the hardware link misbehaves.
