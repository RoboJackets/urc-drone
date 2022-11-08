# PX4 Setup Guide

Make sure you have followed the installation instructions in the `urc-software` repo [here](https://github.com/RoboJackets/urc-software/blob/master/documents/installation/ubuntu_installation.md) **before** following these instructions. The following steps assume that your folder structure is set up as prescribed above.

If you use WSL, make sure to follow the [WSL prerequisite](wsl_px4_setup_prereq.md) before continuing. 

## 1. Install Dependencies

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

In `urc/drone-colon/src/urc-drone` run,

```bash
git submodule sync && git submodule update --init --recursive
```

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

```
sudo snap install micro-ros-agent
```

## 2. Run Simulation Environment

In `urc/drone-colon/src/urc-drone/external/PX4-Autopilot`, run

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

You should see a list of topics that have the `/fmu/` prefix. Try running a `ros2 topic echo <topic_name>` to ensure that you're getting data! If you are, you're all set :)




