#!/usr/bin/env bash
# Patch ArduPilot's stock SITL submarine model with Team Oceanus' ROV values so
# the simulated dynamics approximate OUR vehicle instead of the default BlueROV2.
#
# Edit the numbers below and rebuild (`docker compose build`) to update. Every
# value is tagged with its source / confidence — see OCEANUS-VEHICLE-MODEL.md.
#
# Usage: apply_oceanus_model.sh <path to SIM_Submarine.h>
set -euo pipefail
F="${1:?path to SIM_Submarine.h required}"

# ---- Team Oceanus ROV parameters -------------------------------------------
LENGTH=0.60                 # m  | ESTIMATE  (<1 m req; 400 mm enclosure/chassis) — update from CAD
WIDTH=0.50                  # m  | ESTIMATE  — update from CAD
HEIGHT=0.35                 # m  | ESTIMATE  — update from CAD
MASS=22.0                   # kg | DESIGN TARGET (reqs: <35 hard, <25, <18) — update when weighed
THRUST=24.5                 # N  | REAL: Diamond Dynamics TD1.2 max forward ~2.5 kgf (range 1.2-2.5 kgf)
MOUNT_RADIUS=0.28           # m  | ESTIMATE: thruster distance from CoM — update from CAD
SPHERE_RADIUS=0.25          # m  | ESTIMATE: drag/inertia scale — update from CAD
# Drag coefficients left at ArduPilot defaults (no measured hydrodynamics in repo).
# ----------------------------------------------------------------------------

sed -i \
  -e "s/float length = 0\.457;.*/float length = ${LENGTH}; \/\/ Oceanus (estimate)/" \
  -e "s/float width  = 0\.338;.*/float width  = ${WIDTH}; \/\/ Oceanus (estimate)/" \
  -e "s/float height = 0\.254;.*/float height = ${HEIGHT}; \/\/ Oceanus (estimate)/" \
  -e "s/float weight = 10\.5;.*/float weight = ${MASS}; \/\/ Oceanus (design target)/" \
  -e "s/float thrust = 51\.48;.*/float thrust = ${THRUST}; \/\/ Oceanus Diamond Dynamics TD1.2 (real)/" \
  -e "s/float thruster_mount_radius = 0\.25;.*/float thruster_mount_radius = ${MOUNT_RADIUS}; \/\/ Oceanus (estimate)/" \
  -e "s/float equivalent_sphere_radius = 0\.2;.*/float equivalent_sphere_radius = ${SPHERE_RADIUS}; \/\/ Oceanus (estimate)/" \
  "$F"

echo ">> Applied Team Oceanus vehicle model to $F:"
grep -nE 'float (length|width|height|weight|thrust|thruster_mount_radius|equivalent_sphere_radius) =' "$F" | sed 's/^/   /'
