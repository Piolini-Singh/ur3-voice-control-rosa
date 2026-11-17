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

> **Tested from scratch.** Besides the lab PC used for the thesis, this guide was followed
> step by step on a freshly installed Ubuntu 22.04 computer: levels 0, 1 and 2 (keyboard mode)
> work.

## Before you start: two habits that avoid most problems

* **Run commands one at a time.** Pasting a whole block at once can make `sudo` read the next
  line as your password, or keep going after a failed command. The blocks below are short so
  you can paste them line by line.
* **Every terminal starts from the repository folder.** All commands below assume the
  repository is cloned in your home folder (`~/ur3-voice-control-rosa`) and use full paths, so
  they work from any directory.

## Requirements

* Ubuntu 22.04 (native install; the authors did not test VMs or WSL2) with
  [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) desktop.
* Roughly 4 GB of free disk for the extra packages; a dedicated GPU is not required.
* For levels 0 and 2: an [OpenAI API key](https://platform.openai.com/api-keys) with credit.
  Each command costs one Whisper transcription (voice mode only) plus one or more GPT-4o calls.
  Setting a monthly budget limit on the OpenAI account is a good idea.
* For voice mode: a microphone. Keyboard mode is identical apart from the input source.

## 1. Install system packages

```bash
sudo apt update
```
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool python3-venv ros-humble-ur ros-humble-moveit ros-humble-ros-gz ros-humble-gz-ros2-control libportaudio2
```

`ros-humble-ur` brings the UR driver, description and MoveIt configuration; the Gazebo
simulation package is built from source in the next step, exactly as in the thesis.
`libportaudio2` is only needed for voice mode.

## 2. Clone and build the workspace

```bash
cd ~
git clone https://github.com/Piolini-Singh/ur3-voice-control-rosa.git
```

Then, one line at a time:

```bash
cd ~/ur3-voice-control-rosa/tutorial_ws
source /opt/ros/humble/setup.bash
vcs import src < dependencies.repos
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build
```

* `vcs import` clones `Universal_Robots_ROS2_GZ_Simulation` (branch `humble`) into `src/`.
* `sudo rosdep init` is needed only the first time on a machine; if it says the file already
  exists, ignore it.
* `rosdep install` must end with `#All required rosdeps installed successfully`.
* `colcon build` must end with `Summary: N packages finished` and no `failed`.

**Every new terminal** in the rest of this guide starts with these three lines:

```bash
cd ~/ur3-voice-control-rosa
source /opt/ros/humble/setup.bash
source ~/ur3-voice-control-rosa/tutorial_ws/install/setup.bash
```

## 3. Set up the voice agent (levels 0 and 2)

```bash
cd ~/ur3-voice-control-rosa
source /opt/ros/humble/setup.bash
./scripts/setup_venv.sh
```

The script creates `venv_agente_vocale/` with `--system-site-packages` (otherwise `rclpy` is
not importable inside it), installs the exact package versions of the thesis
(`voice_agent/requirements.lock`) and creates `.env` from `.env.example`. It ends with
`Done. Activate with: ...`; it can take a few minutes.

Now store your key in `.env` **without showing it on screen or saving it in the shell history**:

```bash
cd ~/ur3-voice-control-rosa
read -s -p "Paste your OpenAI key and press Enter: " K; echo; printf 'OPENAI_API_KEY="%s"\n' "$K" > .env; unset K; wc -c .env
```

Paste the key **after** the prompt appears (in the Ubuntu terminal: Ctrl+Shift+V); nothing is
displayed while you paste. `wc -c` should print a number of roughly 60–200 depending on the key
type; a much smaller number (e.g. `19`) means the paste did not work. To check the format
without revealing the key:

```bash
sed -E 's/(sk-[A-Za-z0-9_-]{4})[A-Za-z0-9_-]*/\1***HIDDEN***/g' ~/ur3-voice-control-rosa/.env
```

It must print exactly one line: `OPENAI_API_KEY="sk-xxxx***HIDDEN***"`. `.env` is git-ignored;
never commit it, paste it in a chat or put it in a screenshot.

---

## Level 0 · Agent only (no Gazebo, no MoveIt)

Checks that the language layer works and shows exactly what the model produces.

**Terminal A** – watch the command topic (the message type must be given, because nobody is
publishing yet):

```bash
ros2 topic echo /comando_vocale std_msgs/msg/String
```

**Terminal B** – keyboard agent:

```bash
source ~/ur3-voice-control-rosa/venv_agente_vocale/bin/activate
python3 ~/ur3-voice-control-rosa/voice_agent/09_agente_vocale_testo.py
```

The prompt must start with `(venv_agente_vocale)` and the agent must print
`✅ Subscriber rilevato`. Type, for example, `vai a destra di 5 cm`: terminal A shows
`data: '{"mode": "relative", "dx": 0.05, "dy": 0.0, "dz": 0.0}'`. A compound sentence such as
`alzati di 10 cm e poi vai a destra di 5` produces two messages, in order. Type `fine` to quit.

---

## Level 1 · Robot side only (no API key)

Use four terminals, each starting with the three lines of step 2.

**Terminal 1** – Gazebo digital twin of the UR3 (thesis command C.12):

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur3 launch_rviz:=false
```

Wait until the arm is loaded and the controllers are active
(`ros2 control list_controllers` lists `joint_trajectory_controller` as *active*).

**Terminal 2** – MoveIt 2 with RViz, on simulation time (thesis command C.13):

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 use_sim_time:=true launch_rviz:=true
```

**Terminal 3** – the project's command interface (node 08) listening on `/comando_vocale`,
**also on simulation time**:

```bash
ros2 run tutorial_nodi_base 08_interfaccia_continua --ros-args -p modalita:=vocale -p use_sim_time:=true
```

Wait for `✅ Server MoveIt connessi.` and `In attesa di comandi...`.

> Why `use_sim_time:=true`: Gazebo and MoveIt run on simulated time, so node 08 must use the
> same clock. Otherwise it can read a stale or inconsistent pose of the tool, MoveIt is asked
> to rotate the tool as well as translate it, and feasible moves are rejected (e.g.
> "raggiunto solo il 67%"). On the real robot everything uses the wall clock, so the
> parameter is not needed there.

**Terminal 4** – send commands by hand, one at a time, waiting for the result in terminal 3:

```bash
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"home\"}'}"
```

Do this first: Gazebo spawns the arm in a different pose. Then:

```bash
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"relative\", \"dx\": 0.0, \"dy\": 0.0, \"dz\": 0.05}'}"
```
```bash
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"absolute\", \"x\": 0.13, \"y\": -0.28, \"z\": 0.42}'}"
```

An unreachable target, which node 06 must reject without moving the arm:

```bash
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"absolute\", \"x\": 0.0, \"y\": 0.0, \"z\": 0.9}'}"
```

End of the session (node 08 returns HOME and exits):

```bash
ros2 topic pub --once /comando_vocale std_msgs/msg/String "{data: '{\"mode\": \"fine\"}'}"
```

Terminal 3 prints the reachability check (node 06) and the result of each motion (node 07);
the arm moves in Gazebo and RViz.

Prefer typing coordinates? Start node 08 without `modalita` (keyboard mode) and enter `X Y Z`:

```bash
ros2 run tutorial_nodi_base 08_interfaccia_continua --ros-args -p use_sim_time:=true
```

---

## Level 2 · Full pipeline

Keep terminals 1–3 of level 1 running. If node 08 exited after `fine`, start it again: it must
be running **before** the agent, which waits up to 5 s for a subscriber on `/comando_vocale`.

**Terminal 4** – keyboard agent:

```bash
source ~/ur3-voice-control-rosa/venv_agente_vocale/bin/activate
python3 ~/ur3-voice-control-rosa/voice_agent/09_agente_vocale_testo.py
```

Try, one at a time and waiting for each motion to finish in terminal 3: `torna a casa`,
`alzati di un po'`, `spostati a sinistra di 10 cm e poi scendi di 3`,
`vai a x 0.13 y -0.28 z 0.42`, `basta`. If a number has no unit (e.g. `abbassati di 6`), the
agent asks for it instead of guessing: this is the prompt's safety rule at work.

**Voice mode** – once keyboard mode works, stop the keyboard agent (`fine`) and start the
voice one in the same terminal:

```bash
python3 ~/ur3-voice-control-rosa/voice_agent/09_agente_vocale.py
```

Press ENTER to start recording, speak, press ENTER again to stop.

Direction words follow the convention fixed in the prompt (operator standing in front of the
robot): right = +X, left = −X, back = +Y, forward = −Y, up = +Z, down = −Z.

> **The agent's reply is not the robot's result.** The agent only knows that it *sent* the
> command; it does not receive the outcome from node 08. It may answer "Mi sono spostato…"
> even when node 06 rejected the target and the arm did not move. The authoritative result
> is the output of node 08 in terminal 3.

---

## Other ways to simulate (not validated by the authors)

* **Mock hardware, no Gazebo** (lightest; arm "teleports"): follow the
  [UR driver documentation](https://docs.ros.org/en/humble/p/ur_robot_driver/doc/usage.html)
  and replace terminals 1–2 with

  ```bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
  ```
  ```bash
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
| `No such file or directory` for `scripts/...`, `voice_agent/...` or `install/setup.bash` | The terminal is not in the repository folder. Use the full paths shown above (`~/ur3-voice-control-rosa/...`). |
| `ros2 topic echo` exits with `Could not determine the type` | Give the type: `ros2 topic echo /comando_vocale std_msgs/msg/String`. |
| `⚠️ Nessun subscriber rilevato su /comando_vocale` | Node 08 not running (or started after the agent). Start it first with `-p modalita:=vocale`. |
| `ModuleNotFoundError: rclpy` inside the venv | The venv was created without `--system-site-packages`, or ROS 2 was not sourced before activating it. Recreate it with `scripts/setup_venv.sh`. |
| `OPENAI_API_KEY non trovata` | `.env` missing, empty or malformed. Recreate it with the `read -s` command of step 3 and check it with the `sed` command. |
| `invalid_api_key` / `insufficient_quota` from OpenAI | Wrong or revoked key / no credit left on the OpenAI account. |
| `OSError: PortAudio library not found` | `sudo apt install libportaudio2` (voice mode only). |
| `ImportError` from `langchain` | A newer, incompatible LangChain got installed. Recreate the venv with `scripts/setup_venv.sh` (installs the exact versions from `voice_agent/requirements.lock`). |
| `❌ Server mancanti … compute_cartesian_path` | MoveIt (terminal 2) not running or not ready yet; start node 08 after MoveIt is up. |
| `⏱️ Timeout: … non si e' concluso entro … s` | The motion took longer than the default 60 s; the arm may still be moving. Raise the limit when starting node 08: `-p timeout_movimento:=120.0`. |
| A short move is rejected (`raggiunto solo il 67%`) and MoveIt's log shows many points for a few centimetres | Node 08 started without `-p use_sim_time:=true` in simulation. |
| Execution fails with MoveIt code `-4` | Controller mismatch: check `ros2 control list_controllers`; in Gazebo `joint_trajectory_controller` must be active. |
| `Percorso rettilineo incompleto` then MoveJ fallback, or `posa non raggiungibile` | Expected for targets far away or near the base: the straight line leaves the workspace. Go HOME and use smaller steps. |
| Commands ignored with `movimento gia' in corso` | Node 08 drops commands received while a motion is running. Wait for the previous one to finish. |
