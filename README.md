# Cbl — WaveRider digital twin

## ▶ To run the demo:
- **Lab PC (Ubuntu, no Docker):** see **[LAB_RUN.md](LAB_RUN.md)** — step-by-step, foolproof
- **Windows + Docker:** see **[RUN.md](RUN.md)** — copy-paste, top to bottom
- 3 graded acts (state sync / hazard re-route / comms halt) with exact commands.
- Full status of every feature: [docs/STATUS.md](docs/STATUS.md)
- Lab-session prep (real robot): [docs/LAB_RUNBOOK.md](docs/LAB_RUNBOOK.md)
- One-command run: `ros2 launch my_tb3_world waverider.launch.py`

---

## Original setup notes
Create the package as instructed in the setting up workspace
paste the tb3world folder and use that.
when launching gazebo use that package
use  ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=src/../resource/map.yaml to get the correct map in nav2
get the pose 
make the python nav_test executeable by chomd -x ...
run the script but be careful with selecting hte goal
In theory if both robots are in the correct spot navigation should be smooth and fast
