# Running PX4

## Run Simulation Environment

In `urc/drone-colon/src/external/PX4-Autopilot`, run

```bash
make px4_sitl gazebo
```

In a **second** terminal, navigate to `urc/drone-colon/src/external/micro-xrce-agent` and run

```bash
MicroXRCEAgent udp4 -p 8888
```

In a **third** terminal, navigate to `urc/drone-colcon` and run

```bash
. install/setup.bash
```

```bash
ros2 topic list
```

You should see a list of topics that have the `/fmu/` prefix. Try running a `ros2 topic echo <topic_name>` to ensure that you're getting data! If you are, you're all set!

