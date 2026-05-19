# `r2d3_eval/`

Source of the `r2d3-eval` command-line tool.

Responsibilities:
- Load a task definition from `../tasks/<slug>.yaml`
- Launch the Isaac Sim scene with a deterministic seed
- Execute the agent under test (Python entry point or ROS2 action client)
- Apply the task's scoring rubric against the resulting world state
- Emit a structured report (JSON + console summary) suitable for upload to the leaderboard

To be packaged as a `pyproject.toml` project once stable.
