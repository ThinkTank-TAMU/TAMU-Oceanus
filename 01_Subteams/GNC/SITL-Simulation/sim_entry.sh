#!/usr/bin/env bash
# Container entrypoint: launch ArduSub SITL headless, then a MAVProxy hub that
# fans the MAVLink stream out to two listening TCP ports so multiple clients
# (QGroundControl + pymavlink scripts) can connect at once.
set -euo pipefail
# Activate the ArduPilot python env the installer set up (venv / ~/.local/bin)
. "$HOME/.profile" 2>/dev/null || true
cd "$HOME/ardupilot"

echo ">> Starting ArduSub SITL (frame: vectored_6dof)…"
# Load the vehicle's own parameter file, then the SIM-only overrides, so the sim
# flies the configuration the real ROV gets. /sim/params is the mounted ./params.
PARAMS=()
for f in /sim/params/vectored_6dof.parm /sim/params/sitl.parm; do
    [ -f "$f" ] && PARAMS+=(--add-param-file="$f") && echo "   params: $f"
done
Tools/autotest/sim_vehicle.py -v ArduSub -f vectored_6dof \
    --no-rebuild --no-mavproxy -w -l 33.6,-118.0,0,0 "${PARAMS[@]}" \
    >/tmp/sitl.log 2>&1 &

echo ">> Waiting for firmware on tcp:5760…"
for _ in $(seq 1 90); do
    (echo > /dev/tcp/127.0.0.1/5760) 2>/dev/null && break
    sleep 1
done

# Note: the SITL binary itself occupies 5760-576x (its SERIALn TCP ports), so the
# MAVProxy fan-out ports must live outside that range.
echo ">> SITL up. Starting MAVProxy hub:"
echo "     tcp:localhost:5780  ->  GNC scripts / pymavlink"
echo "     tcp:localhost:5781  ->  QGroundControl (add a TCP comm link)"
exec mavproxy.py --master tcp:127.0.0.1:5760 \
    --out tcpin:0.0.0.0:5780 \
    --out tcpin:0.0.0.0:5781 \
    --non-interactive
