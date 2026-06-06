# Scalability — adding more sensors (NFR4)

> NFR4: *the system must scale to more sensors (CO2, salinity, ...) without
> re-architecting.* This is satisfied by design: sensors are independent nodes
> that publish a self-describing JSON payload, and the twin consumes them
> generically. Adding a sensor is a copy-paste + one launch line, no changes to
> the supervisor, alerting, or visualisation.

## Why it already scales

1. **One sensor = one node = one topic.** `ph_sensor` reads `/odom` and publishes
   JSON on `/water_quality`. Nothing else owns its data. New sensors are new
   nodes; they never touch existing files (no merge conflicts, no shared state).
2. **Self-describing JSON payloads.** Readings are `std_msgs/String` JSON, so a
   new field (e.g. `salinity_psu`) just appears in the payload — consumers that
   don't care ignore it, consumers that do read it by key. No custom-message
   rebuilds, no interface package to bump.
3. **Generic twin consumers.** `dt_supervisor`, `anomaly_alert`, and
   `anomaly_zone_viz` parse the JSON by key and react to whichever fields are
   present. They are not hard-wired to pH.
4. **Docker / DDS discovery scales horizontally.** Every node joins the same ROS 2
   graph over `--net=host`; launching N sensor nodes needs no broker config and
   no central registry. More sensors = more containers/processes, discovered
   automatically.

## Add a new sensor in 3 steps (worked example: salinity)

**1. Copy the pattern.** Duplicate `my_tb3_world/ph_sensor.py` to
`salinity_sensor.py`; keep the `/odom` subscription and the position-dependent
field, and publish your reading:

```python
class SalinitySensor(Node):
    def __init__(self):
        super().__init__('salinity_sensor')
        # ... subscribe /odom (BEST_EFFORT), create a timer ...
        self.pub = self.create_publisher(String, 'salinity', sensor_qos)

    def _tick(self):
        x, y = self.pose
        salinity = 35.0 + 0.5 * math.sin(0.7 * x)   # PSU, position-dependent
        self.pub.publish(String(data=json.dumps({
            'salinity_psu': round(salinity, 2),
            'x': round(x, 3), 'y': round(y, 3),
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
        })))
```

**2. Register it** in `setup.py` (one line under `console_scripts`):

```python
'salinity_sensor = my_tb3_world.salinity_sensor:main',
```

**3. Add it to the launch** (`waverider.launch.py`, one line):

```python
n('salinity_sensor'),
```

That's it. `colcon build` and run. To have the twin act on salinity, add one
`elif` in `dt_supervisor` / a threshold in `anomaly_alert` reading the new key —
no structural change.

## Putting a second metric through the existing topic

If a metric is part of the same water sample, you can instead add a field to the
existing `/water_quality` payload (e.g. `ph_sensor` already publishes `ph`,
`co2_ppm`, `temp_c` together). Consumers pick it up by key with zero wiring.
Use a separate node + topic when the sensor is physically/temporally independent;
use an extra field when it travels with an existing reading.

## Demonstrating the scale (for the report/video)

```bash
# launch the system, then add more sensor instances live:
ros2 run my_tb3_world ph_sensor --ros-args -r __node:=ph_sensor_2 -r water_quality:=water_quality_2
ros2 run my_tb3_world salinity_sensor
ros2 topic list | grep -E 'water_quality|salinity'   # all discovered automatically
```

No reconfiguration was needed to bring the new sensors online — that is NFR4.
