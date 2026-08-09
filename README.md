# ED2 — Remote Inverted Pendulum Lab

Web-controlled inverted pendulum experiment. Students log in at
[sciencelabtoyou.com](https://sciencelabtoyou.com), select an experiment, tune
PID parameters, and watch the physical rig respond over a live video feed.

**📖 Full documentation is in [docs/](docs/)** — start with
[docs/README.md](docs/README.md) for a system overview, or
[docs/architecture.md](docs/architecture.md) for how the pieces connect.

## Files directory

1. **ESP32 code** — firmware for the 2 esp32's → [docs/esp32.md](docs/esp32.md)
2. **Feedback Documentation** — original files from Feedback for the pendulum, includes schematics
3. **KiCad** — files for the PCB
4. **MATLAB** — simulink models converted for the raspberry pi
5. **picamera** — files on the pi that is used to make the video sent to the server → [docs/raspberry-pis.md](docs/raspberry-pis.md)
6. **picontrol** — `dac_daemon.py` that monitors for button presses and executes the simulink models → [docs/raspberry-pis.md](docs/raspberry-pis.md)
7. **simulink-param-retrieval** — files for the simulink conversion to get the parameters from the `params.json` file into the simulink model running on the pi
8. **WebInterface** — django website interface for sciencelabtoyou.com → [docs/server-oracle.md](docs/server-oracle.md)
9. **deploy** — nginx / systemd / mosquitto / mediamtx configs and the server bootstrap script
10. **docs** — documentation for the whole ecosystem

## Deploying the server

```bash
ssh ubuntu@sciencelabtoyou.com
curl -O https://raw.githubusercontent.com/skylargutman/ED2-Repo/main/deploy/bootstrap.sh
chmod +x bootstrap.sh && ./bootstrap.sh
```

See [docs/server-oracle.md](docs/server-oracle.md) for the full runbook and
[docs/operations.md](docs/operations.md) for updates and troubleshooting.
