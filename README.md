## isaac_sim_orchard

This repository provides standalone scripts, USD assets, and Docker recipes to run an orchard simulation in NVIDIA Isaac Sim 5.x with a rover robot. It loads a custom orchard map and rover USD, attaches a camera and RTX LiDAR, and publishes ROS 2 topics via the built-in `isaacsim.ros2.bridge` writers.

### Features
- Load custom orchard map and rover USD
- Attach RTX LiDAR (3D PointCloud2, optional 2D LaserScan)
- Attach camera and publish `/image_raw` and `/camera_info`
- ROS 2 publishing via `isaacsim.ros2.bridge` (no direct `rclpy` in scripts)
- Config-driven startup via `orchard_standalone_scripts/config.yaml`

### Repository Structure
- `orchard_standalone_scripts/`
  - `compose_robot_map.py`: Composes the scene, spawns the robot, attaches sensors, and wires ROS 2 writers
  - `config.yaml`: Paths, spawn pose, sensor parameters, and app/simulation settings
  - `assets/`: USD scenes and rover assets (tracked via Git LFS)
- `dockerfiles/`
  - `Dockerfile`: Isaac Sim 5.0.0 + ROS 2 Humble base build (headless by default)
  - `docker-compose.yml`: Host-networked, GPU-enabled service with cache volume mounts
  - `README.md`: Notes for container usage
- `fastdds.xml`: Example Fast DDS profile (forces UDPv4 transport)

### Requirements
- NVIDIA GPU with drivers compatible with Isaac Sim 5.x images
- Docker, Docker Compose, and NVIDIA Container Toolkit
- (Optional) ROS 2 Humble on host for debugging tools

---

## Using Docker

The provided Docker setup builds on `nvcr.io/nvidia/isaac-sim:5.0.0` and adds ROS 2 Humble CLI tools. Volumes are preconfigured for cache directories and assets.

### 1) Prepare host cache directories (recommended)
```bash
mkdir -p /home/daniel/isaac_sim_cache/docker_isaac-sim/{kit/cache/Kit,cache/{ov,pip,glcache,computecache},logs,data,documents}
```

If you plan to run the GUI, allow X11 access once on the host:
```bash
xhost +local:docker
```

### 2) Build the image
From the repository root:
```bash
docker compose -f dockerfiles/docker-compose.yml build
```

This uses `dockerfiles/Dockerfile` with build context at the repository root (required for mounting assets and cache volumes).

### 3) Start the base container
```bash
docker compose -f dockerfiles/docker-compose.yml up -d
```

This starts a long-running, GPU-enabled container named `kimm_isaac_sim` that idles (`sleep infinity`). You can exec into it:
```bash
docker exec -it kimm_isaac_sim bash
```

### 4) Run Isaac Sim
- Headless (no window):
```bash
docker compose -f dockerfiles/docker-compose.yml run --rm --entrypoint \
  "/bin/bash -lc" run_isaac \
  "/isaac-sim/runapp.sh --no-window"
```

- GUI (requires X11): remove `--no-window` and ensure `DISPLAY` and `/tmp/.X11-unix` are exposed.

### 5) Run the standalone script inside the container
Assets and scripts are mounted at `/root/orchard_standalone_scripts`. From inside the container:
```bash
/isaac-sim/python.sh /root/orchard_standalone_scripts/compose_robot_map.py
```
Adjust `config.yaml` (path in the same directory) to change the map/robot USDs, spawn pose, sensor parameters, and renderer.

---

## Configuration
Edit `orchard_standalone_scripts/config.yaml`:
- `map_usd`, `robot_usd`: Relative paths to USD assets under `orchard_standalone_scripts/assets`
- `robot_prim`, `robot_xyz`, `robot_rpy`: Spawn path and pose
- `camera`: Link path and resolution
- `lidar`: Variant and 2D LaserScan toggle
- `app`, `sim`: Renderer, window size, headless/UI toggles

## ROS 2 Topics
- `/image_raw`, `/camera_info`
- `/point_cloud` (RTX LiDAR 3D)
- `/scan` (RTX LiDAR 2D, if enabled)

## Fast DDS (optional)
You can pass `fastdds.xml` via environment variable in your shell before launching Isaac Sim:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/daniel/aomr_project/isaac_sim_orchard/fastdds.xml
```

## License
TBD.
