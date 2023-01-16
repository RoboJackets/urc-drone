# PX4 Setup Guide

## 1. Install Dependencies

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

In `urc/drone-colon/src` run,

```bash
git submodule sync && git submodule update --init --recursive
```

```bash
bash external/PX4-Autopilot/Tools/setup/ubuntu.sh
```

## 2. Run Simulation Environment

In `urc/drone-colon/src/external/PX4-Autopilot`, run

```bash
make px4_sitl gazebo
```

On a second terminal, run

```bash
micro-ros-agent udp4 --port 8888
```

Now back in the first terminal, run

```bash
microdds_client status
```

You should see the following line: `INFO  [microdds_client] Running, connected`

In a **third** terminal, navigate to `urc/drone-colcon`. Run the following.

```bash
. install/setup.bash
```

```bash
ros2 topic list
```

You should see a list of topics that have the `/fmu/` prefix. Try running a `ros2 topic echo <topic_name>` to ensure that you're getting data! If you are, you're all set!
