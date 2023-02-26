# PX4 Setup Guide

## Install Dependencies

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

## To run:
[Follow the PX4 run instructions here!](run_px4_instructions.md)
