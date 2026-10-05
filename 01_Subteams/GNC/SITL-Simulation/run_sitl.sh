#!/usr/bin/env bash
# Build (first run only) and launch ArduSub SITL.
#   GNC scripts     ->  tcp:127.0.0.1:5780   (the scripts default to this)
#   QGroundControl -> add TCP link to  localhost:5781
# Each port takes one client at a time.
# Stop with Ctrl-C.
set -euo pipefail
cd "$(dirname "$0")"
exec docker compose up --build
