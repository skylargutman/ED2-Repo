# tools

Run from the repo root (Git Bash on Windows). The SSH scripts reach the Pi through the server:
`ssh sciencelabtoyou.com` then `ssh pendulum` (reverse tunnel on the server's port 2223), using
`$SSH_DIR` (default `~/.ssh`: `config`, `id_ed25519`, `known_hosts`).

| Script | What it does |
|---|---|
| `deploy_models_to_pi.sh [Exp ...]` | Builds the ported UART models on the Pi (`build_pi.sh`) and links `~/picontrol/models/MATLAB_ws/R2025b/<Exp>.elf`, the file `dac_daemon` runs for a website experiment. Generate each model's code in MATLAB first. Building never moves the cart. |
| `rig_run.sh EXP [MAXSEC]` | **Moves the cart.** Starts EXP exactly as the website does (MQTT `pendulum/cmd`), follows the daemon log, logs the ESP32's USB output, and summarises the telemetry the website received. Sends Stop after MAXSEC (default 100). |
| `pi_motor_symmetry.py [--fric POS NEG] levels...` | **Moves the cart.** Runs on the Pi (`cat it \| ssh ... python3 -`). Drives each level right then left over the Simulink UART link and prints the steady speed each way. Without `--fric` it tests raw volts (offsets 0) and restores 0.3/0.3; with `--fric` the offsets stay set. |
| `export_pcb_upload.ps1` | Exports the PendulumV2 Gerbers, drill files, schematic PDF, DRC report, paper-fit print and completion statement with the lab's file names into `KiCad/PendulumV2/Upload_<term>_V<n>/`. |

ESP32 USB console on the Pi (`/dev/ttyUSB0`, 921600): open it with a plain
`serial.Serial(port, baud)`. Setting `dtr`/`rts` to False before opening resets the ESP32.
