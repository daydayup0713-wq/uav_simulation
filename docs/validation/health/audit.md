# Health audit baseline

Source: 908b3655b957362b6e3e6cf3ca9985d41f5029cb (plus worktree ignore setup).

Tracked repository: 536 files / 4,311,092 bytes. Runtime 25 GiB; private dependencies 23 GiB (datasets 15 GiB); recordings 35 GiB. These categories are not unused source code.

Entrypoints checked: scripts/labctl dispatch; ROS setup.py console_scripts; lab.launch.py generated entrypoint; supervisor processes; ROS services/actions; native CMake; GitHub CI; React imports.

Static candidates are not deletion authority: localization evaluation/frames/quality and slam_config are compatibility exports; ROS subscription references keep objects alive; HTTP do_POST/log_message and launch generate_launch_description are framework callbacks. Native licenses, testing fixtures, alternative CLI/launch paths stay.

Confirmed candidates: frontend unused backends state, ordinary unused imports, unused result variables. Integration/evaluation/planner outputs require regression checks before consolidation.

UI findings: API/observation health conflated; FAILSAFE lacks immediate cause/action; static 8080 presents inactive control options; comparison API failures are swallowed and remain loading forever; request timeouts absent; unchanged paths rebuilt on every telemetry update; narrow layout hides navigation.

Baseline frontend: 13 passed; production build passed; existing act warnings and 808.51 kB JS chunk warning. Native baseline requires the new worktree's generated ROS interfaces and is being built; bare Python first attempt failed collection on missing uav_lab_interfaces (setup issue).
