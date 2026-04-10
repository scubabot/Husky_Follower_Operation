# Husky Multi-Camera Vision

ROS 2 package for the Clearpath Husky A2000 integrating a Stereolabs ZED-X for person-following and a FLIR Boson 320 for thermal object detection using YOLOv8.

## Hardware

- Clearpath Husky A2000
- Stereolabs ZED-X Camera
- FLIR Boson 320 Thermal Camera
- NVIDIA Jetson (for YOLOv8 inference and ZED SDK)

## Repository Structure

```
husky_multicam_vision/
├── README.md
├── requirements_robot.txt        # Dependencies for the ZED person follower
├── requirements_thermal.txt      # Dependencies for the thermal YOLO detector
├── zed_person_center_follower.py
└── thermal_yolo_detector.py
```

## The numpy Version Conflict

The ZED SDK Python bindings (`pyzed`) require `numpy < 2.0`, while newer versions of Ultralytics (YOLOv8) default to `numpy >= 2.0`. Since a single Python environment can only hold one numpy version, this project uses **separate virtual environments** for each script.

> **Try the simple path first:** Ultralytics *may* work fine with `numpy < 2.0` depending on your version. If so, you only need one virtual environment — see Option A below. If you get numpy-related errors in the thermal detector, move to Option B (two environments).

## Dependencies

- ROS 2 (Humble / Iron / Jazzy)
- Python 3.8+
- ZED SDK (installed system-wide on your Jetson)
- FLIR Boson 320 driver / ROS 2 topic publisher

## Installation

### 1. Clone into your ROS 2 workspace

```bash
cd ~/husky_vision_ws/src
git clone https://github.com/<your-username>/husky_multicam_vision.git
```

### 2. Set up Python environment(s)

Both options below use `--system-site-packages` so the virtual environment can see ROS 2 libraries and the ZED SDK that are installed system-wide on the Jetson.

---

#### Option A — Single environment (numpy < 2.0 for everything)

Use this if both scripts work under `numpy < 2.0`. This is simpler and worth trying first.

```bash
cd ~/husky_vision_ws/src/husky_multicam_vision

python3 -m venv venv --system-site-packages
source venv/bin/activate

pip install --upgrade pip
pip install -r requirements_robot.txt
pip install -r requirements_thermal.txt

# Pin numpy last to make sure nothing overrides it
pip install "numpy<2.0.0"
```

Verify it worked:

```bash
python3 -c "import numpy; print(numpy.__version__)"
# Should print 1.x.x
```

---

#### Option B — Two separate environments (if Option A causes errors)

Create one environment per script so each gets its own numpy version.

**Robot controller environment:**

```bash
cd ~/husky_vision_ws/src/husky_multicam_vision

python3 -m venv venv_robot --system-site-packages
source venv_robot/bin/activate

pip install --upgrade pip
pip install -r requirements_robot.txt
pip install "numpy<2.0.0"

deactivate
```

**Thermal detector environment:**

```bash
python3 -m venv venv_thermal --system-site-packages
source venv_thermal/bin/activate

pip install --upgrade pip
pip install -r requirements_thermal.txt
# numpy >= 2.0 will be pulled in automatically by ultralytics

deactivate
```

---

### 3. Build the ROS 2 workspace

```bash
cd ~/husky_vision_ws
colcon build --symlink-install
```

## Usage

Before running any node, you need to activate the correct virtual environment **and** source the ROS 2 workspace in that terminal.

### Running the ZED Person Follower

Make sure the ZED ROS 2 wrapper is running and publishing to `/zed/zed_node/obj_det/objects`.

```bash
# Terminal 1
cd ~/husky_vision_ws/src/husky_multicam_vision
source venv/bin/activate          # or venv_robot if using Option B
source ~/husky_vision_ws/install/setup.bash

ros2 run husky_multicam_vision zed_person_center_follower.py
```

### Running the Thermal YOLO Detector

Make sure your FLIR Boson camera is publishing image topics.

```bash
# Terminal 2
cd ~/husky_vision_ws/src/husky_multicam_vision
source venv/bin/activate          # or venv_thermal if using Option B
source ~/husky_vision_ws/install/setup.bash

ros2 run husky_multicam_vision thermal_yolo_detector.py
```

## Node Details

### `zed_person_center_follower`

| | Topic | Message Type |
|---|---|---|
| Subscribes | `/zed/zed_node/obj_det/objects` | `zed_msgs/ObjectsStamped` |
| Publishes | `/a200_1046/cmd_vel` | `geometry_msgs/Twist` |

Parameters:

- `target_dist` — Target distance to maintain from the detected person (meters).
- `mounting_offset` — Distance from robot center to the camera mount point.


## Troubleshooting

**"numpy has no attribute ..." or similar numpy errors:**
You likely have a numpy version mismatch. Check which version is active with `python3 -c "import numpy; print(numpy.__version__)"` and make sure you are in the correct virtual environment.

**ROS 2 packages not found after activating the venv:**
Make sure you created the venv with `--system-site-packages`. If you forgot, delete the venv folder and recreate it with that flag.

**`colcon build` can't find your scripts:**
Ensure your `setup.py` or `setup.cfg` lists both scripts as entry points or console scripts.

## License
