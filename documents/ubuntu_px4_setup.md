# PX4 Setup Guide

**Currently only works on native Linux and WSL**

Make sure you have followed the installation instructions in the `urc-software` repo [here](https://github.com/RoboJackets/urc-software/blob/master/documents/installation/ubuntu_installation.md) **before** following these instructions. The following steps assume that your folder structure is set up as prescribed above.

## **WSL Users**
## 1. Set systemd flag in WSL distro settings

## 2. Enter the WSL config file
```bash
sudo nano /etc/wsl.conf
```
## 3. Inside the file, type
```
[boot]
systemd=true
```
Press CTRL+O and CTRL+X to save changes.

## 4. Close the terminal, open a Powershell window and type
```bash
wsl --shutdown
```

## 5. Open a new Ubuntu window and continue with the rest of the setup.

**If the sudo snap install still does not work, uninstall WSL and follow [URC Software Setup Instructions](https://github.com/RoboJackets/urc-software/blob/master/documents/installation/ubuntu_installation.md)**

---

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

```
sudo snap install micro-ros-agent
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

You should see a list of topics that have the `/fmu/` prefix. Try running a `ros2 topic echo <topic_name>` to ensure that you're getting data! If you are, you're all set :)
