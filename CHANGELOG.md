# Changelog

## v1.0.0 — 2026-11-06

First public release of the code used for the Bachelor's thesis (University of Brescia,
A.Y. 2025/2026). Behaviour, system prompt and tools are those used in the experiments; the only
changes to the lab code are for portability and packaging:

* `voice_agent/agente_comune.py`: the `.env` path is no longer hard-coded to
  `/home/ur-robot/tirocinio_tesi/.env`; it defaults to the repository root and can be set with
  `UR3_VOICE_ENV_FILE`. The chat model can be overridden with `UR3_VOICE_LLM_MODEL`
  (default unchanged: `gpt-4o`).
* Nodes 04 and 05: the trajectory topic is a ROS parameter, `topic_traiettoria`, so switching
  between Gazebo and the real robot no longer requires editing code. Defaults are unchanged
  (04: `/joint_trajectory_controller/...`, 05: `/scaled_joint_trajectory_controller/...`).
* `package.xml` / `setup.py`: license (Apache-2.0), description and maintainers filled in;
  dependencies aligned with the imports (added `sensor_msgs`, `builtin_interfaces`; removed the
  unused `moveit_py`, and the unused `shape_msgs`).
* Run instructions in the agent scripts now use repository-relative paths.
* Added documentation, architecture diagram, simulation guide, demo media, citation metadata,
  `requirements.lock` (exact virtualenv of the experiments) and CI.
