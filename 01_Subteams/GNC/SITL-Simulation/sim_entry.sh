#!/usr/bin/env bash
# Container entrypoint: launch ArduSub SITL headless, then a MAVProxy hub that
# fans the MAVLink stream out to two listening TCP ports so multiple clients
# (QGroundControl + pymavlink scripts) can connect at once.
set -euo pipefail
# Activate the ArduPilot python env the installer set up (venv / ~/.local/bin)
. "$HOME/.profile" 2>/dev/null || true
cd "$HOME/ardupilot"

# FRAME selects the simulated vehicle. The default is the real ROV. The only other
# value is the Practice Project 1 training frame, which needs the practice image
# (compose service ardusub-sitl-practice4).
FRAME="${FRAME:-vectored_6dof}"
case "$FRAME" in
    vectored_6dof)      VEHICLE_PARM=/sim/params/vectored_6dof.parm ;;
    vectored_practice4) VEHICLE_PARM=/sim/params/practice-4thruster.parm
        # Without the patch, FRAME_CONFIG 7 falls through to SimpleROV-3 and the
        # physics to the 6-thruster table: it would boot and fly wrong.
        if ! grep -q PRACTICE_4THRUSTER libraries/AP_Motors/AP_Motors6DOF.cpp; then
            echo "!! FRAME=vectored_practice4 needs the practice image:" >&2
            echo "!!   docker compose up --build ardusub-sitl-practice4" >&2
            exit 1
        fi ;;
    *) echo "!! Unknown FRAME '$FRAME' (vectored_6dof | vectored_practice4)" >&2; exit 1 ;;
esac

echo ">> Starting ArduSub SITL (frame: $FRAME)…"
# Load the vehicle's own parameter file, then the SIM-only overrides, so the sim
# flies the configuration the real ROV gets. /sim/params is the mounted ./params.
PARAMS=()
for f in "$VEHICLE_PARM" /sim/params/sitl.parm; do
    [ -f "$f" ] && PARAMS+=(--add-param-file="$f") && echo "   params: $f"
done
Tools/autotest/sim_vehicle.py -v ArduSub -f "$FRAME" \
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
