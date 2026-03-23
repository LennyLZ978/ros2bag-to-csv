# ros2bag-to-csv

A ROS 2 subscriber node that records `/mpc_traj` and `/odom` topics into CSV files while playing back a rosbag. CSV files open natively in Excel with no third-party libraries needed.

## Prerequisites

- ROS 2 (Humble / Iron / Jazzy)
- Python 3.10+
- `nav_msgs` package (provides `Odometry` and `Path` message types)

## Usage

**Terminal 1** — start the recorder:

```bash
python3 bag_to_excel/bag_to_excel.py
```

**Terminal 2** — play the bag:

```bash
ros2 bag play <bag_path>
```

Press **Ctrl+C** in Terminal 1 when the bag finishes. Two CSV files are written to the output directory.

### Options

| Flag | Description | Default |
|------|-------------|---------|
| `--namespace`, `-n` | Topic namespace (e.g. `robot1` → `/robot1/mpc_traj`) | *(none)* |
| `--output-dir`, `-o` | Directory for output CSV files | Script directory |

### Example with namespace

```bash
python3 bag_to_excel/bag_to_excel.py --namespace robot1 --output-dir ./output
```

This subscribes to `/robot1/mpc_traj` and `/robot1/odom`.

## Output Format

### `mpc_traj.csv`

Each message produces a group of 3 rows (P, V, A) with pose indices as columns:

| timestamp | Poses[0] | Poses[1] | ... |
|-----------|----------|----------|-----|
| t1 | pos.x[0] | pos.x[1] | ... |
| | pos.y[0] | pos.y[1] | ... |
| | yaw[0] | yaw[1] | ... |

### `odom.csv`

Transposed layout with timestamps as columns:

| timestamp | t1 | t2 | ... |
|-----------|------|------|-----|
| P | pos.x | pos.x | ... |
| V | lin_x | lin_x | ... |
| A | ang_z | ang_z | ... |

Where **P** = `pose.pose.position.x`, **V** = `twist.twist.linear.x`, **A** = `twist.twist.angular.z`.
