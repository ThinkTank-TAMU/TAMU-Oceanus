#!/usr/bin/env bash
# Make SIM_WIND_SPD / SIM_WIND_DIR behave as a horizontal WATER CURRENT in the
# ArduSub SITL submarine model.
#
# Why: stock ArduSub SITL computes the sub's wind vector but never applies it to
# the submarine's drag, so the default still-water sim can't exercise the
# ConOps "current compensation / drift" tasks. This patch makes the linear drag
# act on the vehicle's velocity *relative to the moving water*, so setting
# SIM_WIND_SPD/DIR pushes the ROV (it drifts toward the current at the current
# speed when unpowered).
#
# Usage: apply_current_model.sh <path to SIM_Submarine.cpp>
set -euo pipefail
F="${1:?path to SIM_Submarine.cpp required}"

if grep -q "fluid_rel_vel_bf" "$F"; then
    echo "current model already applied to $F"; exit 0
fi

sed -i "s|    calculate_drag_force(velocity_air_bf, frame_property.linear_drag_coefficient, linear_drag_forces);|    // Water current: apply SIM_WIND_SPD/DIR as a horizontal current (stock SITL\n    // does not feed wind to the sub). Drag acts on velocity relative to the water.\n    Vector3f current_ef(cosf(radians(sitl->wind_direction)) * sitl->wind_speed, sinf(radians(sitl->wind_direction)) * sitl->wind_speed, 0.0f);\n    Vector3f fluid_rel_vel_bf = dcm.transposed() * (velocity_ef - current_ef);\n    calculate_drag_force(fluid_rel_vel_bf, frame_property.linear_drag_coefficient, linear_drag_forces);|" "$F"

echo ">> Water-current model applied to $F:"
grep -n "fluid_rel_vel_bf" "$F" | sed 's/^/   /'
