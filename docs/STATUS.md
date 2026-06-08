# WaveRider — Coding Status

Branch: `feat/digital-twin-sim-nodes`. Every node below was built and **verified
live in the Gazebo sim**. Run the whole system with one command:

```bash
ros2 launch my_tb3_world waverider.launch.py            # also starts Gazebo
ros2 launch my_tb3_world waverider.launch.py start_world:=false   # world already up
```

## To-do list — status

**1. pH sensor node (state sync) — (done)**
- subscribe /odom (position), publish simulated pH + CO2 on /water_quality
- values change by position + a scripted sudden drop for the demo
- no hardware, just code ("simulated sensor")
- → `ph_sensor.py`; leak via `ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: true}"`

**2. DT supervisor node (bidirectional) — (done)**
- the 2-way hub
- robot → twin: subscribe /water_quality, /obstacle_info, /scan
- twin → robot: publish commands back (/cmd_vel override, /goal_pose, /mode)
- replaces the old one-way test_odom.py
- → `dt_supervisor.py`; priority state machine AVOID > ALERT > NORMAL

**3. Hazard node (environmental interaction) — (done)**
- spawn currents/storms ONLY in the digital twin
- twin computes a detour and makes the robot re-route (extend go_to_goal)
- → `hazard_manager.py` (blue DT-only zone + /hazard_zone); `go_to_goal` re-routes
  with potential-field repulsion. Verified ~0.55 m detour around a 0.45 m hazard,
  still reaches the goal.

**4. Comms watchdog (NFR2) — (done)**
- robot halts if it loses the twin for 5s
- checked for twin_safety_node — not present in the repo, so no duplication
- → `comms_watchdog.py`; watches /mode heartbeat, asserts stop on /cmd_vel,
  publishes /twin_alive. Verified halt within 5 s and recovery.

**5. Anomaly alert (FR4) — (done)**
- detect sudden pH drop → send a structured alert + log it
- → `anomaly_alert.py`; structured JSON on /alerts + evidence log
  (/tmp/waverider_alerts.log), RAISED/CLEARED with hysteresis.

**6. Latency logger (NFR1) — (done)**
- measure robot↔twin delay, must be under 1000 ms, rosbag it for evidence
- → `latency_logger.py` (dt_supervisor echoes src_stamp); publishes /latency_ms.
  Verified avg 214 ms, max 415 ms, 0 over limit → NFR1 PASS.

**7. Extend obstacle_info (FR1.2) — (done)**
- flag unexpected objects vs known walls (the "illegal boat" case)
- → `obstacle_info.py` segments the scan; small/near/isolated cluster = unexpected
  object (walls ignored). Verified: walls→0, spawned box→flagged, removed→0.

**8. Show anomaly + hazard zones visually in gazebo (NFR3) — (done)**
- → `anomaly_zone_viz.py` draws a red leak zone; `hazard_manager` draws a blue
  hazard zone in the Gazebo scene via gz /world/create+remove.

**9. Divide map into sectors for nav (items 3.1/3.2) — (done)**
- → `sector_nav.py` patrols a rows×cols grid of sector centres in snake order via
  /goal_pose; `go_to_goal` now follows /goal_pose. Verified sector advance.

**10. Make it easy to add CO2/salinity sensors later (NFR4) — (done)**
- → `docs/SCALABILITY.md` (pattern + 3-step add-a-sensor guide). Verified 3 sensor
  instances auto-discovered & publishing with no reconfiguration.

## Also done (integration)

**11. One launch file + command arbitration — (done)**
- → `waverider.launch.py` runs the whole system; `cmd_mux.py` is the single
  arbitrated writer of /cmd_vel (HALT > OVERRIDE > NAV + fail-safe stop), fixing
  the multi-publisher contention found in the dry-run.

## Remaining (cross-cutting, from the WhatsApp list)

- **12.** Locate / merge the team's `twin_safety_node` / `tb3_safety_stop` — still
  not in the repo.
- **13.** Full system dry-run + **rosbag a clean run** for the video/report.
  Bag-ready topics: `/water_quality /mode /alerts /latency_ms /hazard_zone /sector`.

## Demo mapping (the 3 acts)

- Act 1 (state sync): trigger leak → /alerts RAISED + red zone + mode ALERT.
- Act 2 (bidirectional + env): spawn hazard → robot re-routes around it.
- Act 3 (NFR2): kill the twin → robot halts within 5 s (comms_watchdog).
