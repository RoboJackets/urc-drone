# PX4 Setup Guide

## 1. Install Dependencies

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

In `urc/drone-colon/src` run,

```bash
git submodule sync && git submodule update --init --recursive
```
In `urc/drone-colon/src/external/micro-xrce-agent` run,

```bash
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

In `urc/drone-colon/src` run,

```bash
bash external/PX4-Autopilot/Tools/setup/ubuntu.sh
```

## 2. Run Simulation Environment

In `urc/drone-colon/src/external/PX4-Autopilot`, run

```bash
MicroXRCEAgent udp4 -p 8888
```

In a **third** terminal, navigate to `urc/drone-colcon`. Run the following.

```bash
. install/setup.bash
```

```bash
ros2 topic list
```

You should see a list of topics that have the `/fmu/` prefix. Try running a `ros2 topic echo <topic_name>` to ensure that you're getting data! If you are, you're all set!
