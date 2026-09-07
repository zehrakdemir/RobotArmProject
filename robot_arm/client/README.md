> **Not:** Bu dosya projenin ilk (macOS/Colima) kurulum aşamasından kalma, artık
> kullanılmıyor. Güncel WSL2 kurulumu ve gerçek robot ile çalışma şekli için bkz.
> `../SETUP.md`. Ana araç artık `movecobot.py` (bkz. `../SETUP.md` içindeki
> "Güncel Durum" bölümü).

# FAIRINO SimMachine Docker + Python demo (eski/tarihsel)

This workspace uses FAIRINO SimMachine `v3.9.3` and the matching official
Python SDK `v2.2.3_robot3.9.3`.

## Current local setup

- Colima profile: `fairino` (6 CPU, 6 GiB RAM, 60 GiB disk, Rosetta enabled)
- Docker context: `colima-fairino`
- Image: `fairino_simmachine:v3.9.3`
- Container: `fairino-simmachine`
- WebApp: <http://127.0.0.1> (`admin` / `123`)
- Live WebApp state: WebSocket ports `9998` and `9999`
- Python SDK ports: `127.0.0.1:20003` and `127.0.0.1:20004`

Start an existing environment after a reboot:

```bash
colima start fairino
docker context use colima-fairino
docker start fairino-simmachine
```

Use the WebApp without adding `:8080`. Its frontend appends the live-state
WebSocket port to the browser host, so the standard HTTP port is required.

Run the motion demonstration:

```bash
python3 move_robot.py
```

The demo switches the simulated controller to automatic mode, enables it,
reads the current joint pose, moves joint 1 by 10 degrees at low speed, and
returns to the original pose. The script refuses non-loopback controller
addresses unless `--allow-real-robot` is explicitly supplied.

Useful options:

```bash
python3 move_robot.py --degrees -5 --velocity 5
python3 move_robot.py --degrees 10 --stay
```

Run continuous random simulated motion every two seconds:

```bash
python3 random_motion.py
```

Stop it with `Ctrl+C`. For a finite test, use `python3 random_motion.py --count 10`.

Stop the simulator without deleting it:

```bash
docker stop fairino-simmachine
colima stop fairino
```
