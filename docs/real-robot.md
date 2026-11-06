# Running on a real UR3

> ⚠️ **Safety first.** This is research/teaching software, not a certified safety function.
> Validate every new command, prompt change or code change in simulation first
> ([simulation.md](simulation.md)), keep the workspace clear, keep the teach pendant's
> emergency stop within reach, and use reduced speed. The language model can misinterpret
> speech; the reachability check (node 06) only verifies kinematics and collisions with the
> modelled scene, not with people or unmodelled objects.

Setup used in the thesis: UR3 **CB3** series, PolyScope / URSoftware 3.15.8, External Control
URCap, Ubuntu 22.04 + ROS 2 Humble workstation on the same Ethernet subnet
(`192.168.1.0/24`, robot at `192.168.1.2`, workstation at `192.168.1.104`).

## Prerequisites

* Everything in [simulation.md](simulation.md), steps 1–3 (Gazebo packages are optional here).
* The robot's **calibration file**, generated once per physical robot:

  ```bash
  ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=<ROBOT_IP> target_filename:="$HOME/my_ur3_calibration.yaml"
  ```

  It is robot-specific and git-ignored (`*calibration*.yaml`).
* The **External Control** URCap installed on the pendant, configured with the workstation's
  IP address and port `50002` (Installation → URCaps → External Control).
* `HOME_JOINTS_GRADI` in
  [`07_esegui_movimento_cartesiano.py`](../tutorial_ws/src/tutorial_nodi_base/tutorial_nodi_base/07_esegui_movimento_cartesiano.py)
  was measured on the thesis robot; re-measure it on yours (teach pendant, degrees).

## The four terminals (order matters)

**Terminal 1 – robot driver**

```bash
source /opt/ros/humble/setup.bash
source tutorial_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=<ROBOT_IP> \
  kinematics_params_file:="$HOME/my_ur3_calibration.yaml"
```

**External Control step (between terminal 1 and terminal 2)** – the UR controller only runs
URScript programs, so it must hand motion control to the PC:

1. On the pendant open (or create) a program that contains only the *External Control* node
   (Program → URCaps).
2. Press **Play ▶**.
3. In terminal 1 wait for: `Robot connected to reverse interface. Ready to receive control commands.`
4. Only then start terminal 2.

Starting MoveIt before the driver has activated `scaled_joint_trajectory_controller` makes
trajectory execution fail with MoveIt error `-4`.

**Terminal 2 – MoveIt 2 with RViz** (no `use_sim_time` on the real robot)

```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true \
  kinematics_params_file:="$HOME/my_ur3_calibration.yaml"
```

**Terminal 3 – node 08 in voice mode**

```bash
source /opt/ros/humble/setup.bash
source tutorial_ws/install/setup.bash
ros2 run tutorial_nodi_base 08_interfaccia_continua --ros-args -p modalita:=vocale
```

**Terminal 4 – voice agent**

```bash
source /opt/ros/humble/setup.bash
source venv_agente_vocale/bin/activate
python3 voice_agent/09_agente_vocale.py
```

Press ENTER, speak the command, press ENTER again to stop recording.

## Differences between simulation and the real robot

| Aspect | Gazebo | Real UR3 |
|---|---|---|
| Trajectory controller | `joint_trajectory_controller` | `scaled_joint_trajectory_controller` (honours the pendant speed slider) |
| Joint-space nodes 04/05 | `-p topic_traiettoria:=/joint_trajectory_controller/joint_trajectory` | `-p topic_traiettoria:=/scaled_joint_trajectory_controller/joint_trajectory` |
| Clock | `use_sim_time:=true` for MoveIt | wall clock |
| Wrist turns | no accumulated turns | encoders keep multi-turn values; node 07 re-aligns trajectories by multiples of 2π, otherwise the controller rejects them ("state tolerance violation", error ≈ 6.283 rad) |
| Kinematics | nominal model | per-robot calibration file |
