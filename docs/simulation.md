# Reproduce it in simulation (no robot needed)

This is the same digital-twin procedure used in the thesis before every test on the real UR3
(thesis §2.2.2): Gazebo simulates the arm, MoveIt 2 plans, and the unchanged project nodes and
voice agent run on top. Nothing in the code distinguishes the simulated robot from the real one.

You can stop at whichever level you need:

| Level | What you see | Needs OpenAI key | Needs Gazebo/MoveIt | Time |
|---|---|---|---|---|
| **0 · Agent only** | Italian sentence → JSON command on a topic | yes | no | ~10 min |
| **1 · Robot side only** | JSON command → arm moves in Gazebo/RViz | no | yes | ~30–60 min (first build) |
| **2 · Full pipeline** | Typed or spoken Italian → arm moves | yes | yes | +5 min after level 1 |

## Requirements

* Ubuntu 22.04 (native install recommended; the authors did not test VMs or WSL2) with
  [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) desktop.
* Roughly 4 GB of free disk for the extra packages; a GPU is not required.
* For levels 0 and 2: an [OpenAI API key](https://platform.openai.com/api-keys) with credit.
  Each command costs one Whisper transcription (voice mode only) plus one or more GPT-4o calls.
* For voice mode: a microphone. Keyboard mode is identical apart from the input source.

## 1. Install system packages

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-venv \
  ros-humble-ur ros-humble-moveit \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
  libportaudio2            # only for voice mode (sounddevice)
```

`ros-humble-ur` brings the UR driver, description and MoveIt configuration; the Gazebo
simulation package is built from source in the next step, exactly as in the thesis.

## 2. Clone and build the workspace

```bash
git clone https://github.com/YOUR-GITHUB-USER/ur3-voice-control-rosa.git
cd ur3-voice-control-rosa/tutorial_ws

source /opt/ros/humble/setup.bash
vcs import src < dependencies.repos          # clones Universal_Robots_ROS2_GZ_Simulation (humble)

sudo rosdep init 2>/dev/null || true         # first time on this machine only
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble

colcon build
```

Every new terminal below starts with these two lines (run from the repository root):

```bash
source /opt/ros/humble/setup.bash
source tutorial_ws/install/setup.bash
```

## 3. Set up the voice agent (levels 0 and 2)

```bash
# from the repository root, with ROS 2 sourced
./scripts/setup_venv.sh          # creates venv_agente_vocale/ with --system-site-packages
nano .env                        # put your key: OPENAI_API_KEY="sk-..."
```

The virtualenv must be created with `--system-site-packages` (the script does it), otherwise
`rclpy` is not importable from inside it. `.env` is git-ignored; never commit it.

---

## Level 0 · Agent only (no Gazebo, no MoveIt)

Checks that the language layer works and shows exactly what the model produces.

**Terminal A** – watch the command topic:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /comando_vocale
```

**Terminal B** – keyboard agent:

```bash
source /opt/ros/humble/setup.bash
source venv_agente_vocale/bin/activate
python3 voice_agent/09_agente_vocale_testo.py
```

Type, for example, `alzati di 10 cm e poi vai a destra di 5` and you should see two messages in
terminal A: `{"mode": "relative", "dx": 0.0, "dy": 0.0, "dz": 0.1}` followed by
`{"mode": "relative", "dx": 0.05, "dy": 0.0, "dz": 0.0}`. Type `fine` to quit.

---

## Level 1 · Robot side only (no API key)

**Terminal 1** – Gazebo digital twin of the UR3 (thesis command C.12):

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur3 launch_rviz:=false
```

Wait until Gazebo shows the arm and the controllers are loaded
(`ros2 control list_controllers` should list `joint_trajectory_controller` as *active*).

**Terminal 2** – MoveIt 2 with RViz, on simulation time (thesis command C.13):

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 use_sim_time:=true launch_rviz:=true
```

**Terminal 3** – the project's command interface (node 08) listening on `/comando_vocale`:

```bash
ros2 run tutorial_nodi_base 08_interfaccia_continua --ros-args -p modalita:=vocale
```

Wait for `✅ Server MoveIt connessi.` and `In attesa di comandi...`.

**Terminal 4** – send commands by hand:

```bash
# 1) go to the HOME pose (do this first: Gazebo spawns the arm in a different pose)
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"home\"}'}"

# 2) rise 5 cm
ros2 topic pub --once /comando_vocale std_msgs/msg/String \
  "{data: '{\"mode\": \"relative\", \"dx\": 0.0, \"dy\": 0.0, \"dz\": 0.05}'}"

# 3) an absolute target in base_link (metres)
ros2 topic pub --once /comando_vocale std_msgs/msg/String \
  "{data: '{\"mode\": \"absolute\", \"x\": 0.13, \"y\": -0.28, \"z\": 0.42}'}"

# 4) end the session (node 08 returns HOME and exits)
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"fine\"}'}"
```

Terminal 3 prints the reachability check (node 06) and the execution result (node 07); the arm
moves in Gazebo and RViz. Try an unreachable target such as `"x": 0.0, "y": 0.0, "z": 0.9` to
see the safety check reject it.

Prefer typing coordinates? Start node 08 without the parameter (keyboard mode) and enter `X Y Z`:

```bash
ros2 run tutorial_nodi_base 08_interfaccia_continua
```

---

## Level 2 · Full pipeline

Keep terminals 1–3 of level 1 running (node 08 **before** the agent: the agent waits up to
5 s for a subscriber on `/comando_vocale`), then in terminal 4:

```bash
source /opt/ros/humble/setup.bash
source venv_agente_vocale/bin/activate

python3 voice_agent/09_agente_vocale_testo.py    # keyboard
# or
python3 voice_agent/09_agente_vocale.py          # microphone: ENTER to start, ENTER to stop
```

Try: `torna a casa`, `alzati di un po'`, `spostati a sinistra di 10 cm e poi scendi di 3`,
`vai a x 0.13 y -0.28 z 0.42`, `basta`.

Direction words follow the convention fixed in the prompt (operator standing in front of the
robot): right = +X, left = −X, back = +Y, forward = −Y, up = +Z, down = −Z.

---

## Other ways to simulate (not validated by the authors)

* **Mock hardware, no Gazebo** (lightest; arm "teleports"): follow the
  [UR driver documentation](https://docs.ros.org/en/humble/p/ur_robot_driver/doc/usage.html)
  and replace terminals 1–2 with

  ```bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy \
    use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
  ```

  If execution fails, MoveIt is probably targeting `scaled_joint_trajectory_controller`; check
  `ros2 launch ur_moveit_config ur_moveit.launch.py --show-args` for a `use_fake_hardware`
  argument, or follow the driver docs on switching the default controller.
* **URSim** (Universal Robots' official simulator in Docker): exercises the real-robot path,
  External Control included. See the driver documentation (`start_ursim.sh -m ur3`) and
  [real-robot.md](real-robot.md).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `⚠️ Nessun subscriber rilevato su /comando_vocale` | Node 08 not running (or started after the agent). Start it first with `-p modalita:=vocale`. |
| `ModuleNotFoundError: rclpy` inside the venv | The venv was created without `--system-site-packages`, or ROS 2 was not sourced before activating it. Recreate with `scripts/setup_venv.sh`. |
| `OPENAI_API_KEY non trovata` | No `.env` in the repository root and no exported variable. `cp .env.example .env` and edit it, or set `UR3_VOICE_ENV_FILE`. |
| `OSError: PortAudio library not found` | `sudo apt install libportaudio2` (voice mode only). |
| `ImportError` from `langchain` | A newer, incompatible LangChain got installed. Recreate the venv with `scripts/setup_venv.sh` (installs the exact versions from `voice_agent/requirements.lock`). |
| `❌ Server mancanti … compute_cartesian_path` | MoveIt (terminal 2) not running or not ready yet; start node 08 after MoveIt is up. |
| Execution fails with MoveIt code `-4` | Controller mismatch: check `ros2 control list_controllers`; in Gazebo `joint_trajectory_controller` must be active. |
| `Percorso rettilineo incompleto` then MoveJ fallback, or `posa non raggiungibile` | Expected for targets far away or near the base: the straight line leaves the workspace. Go HOME and use smaller steps. |
| Commands ignored with `movimento gia' in corso` | Node 08 drops commands received while a motion is running. Wait for the previous one to finish. |
