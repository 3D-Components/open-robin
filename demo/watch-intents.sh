#!/usr/bin/env bash
# watch-intents.sh — live monitor for pendingIntent changes in OrionLD
#
# Shows the pendingIntent attribute on urn:ngsi-ld:Process:ros_bridge as it
# changes, so you can see OrionLD acting as the broker before the subscription
# notification fires to welding_http_bridge → /intents ROS2 topic.
#
# Usage:
#   bash demo/watch-intents.sh
#   ORION_URL=http://localhost:1026 bash demo/watch-intents.sh
#   ENTITY_ID=urn:ngsi-ld:Process:my-process bash demo/watch-intents.sh

set -euo pipefail

ORION=${ORION_URL:-http://localhost:1026}
ENTITY=${ENTITY_ID:-urn:ngsi-ld:Process:ros_bridge}
POLL_INTERVAL=${POLL_INTERVAL:-0.8}

echo "============================================================"
echo "  OrionLD pendingIntent watcher"
echo "  Entity : $ENTITY"
echo "  Orion  : $ORION"
echo "  Ctrl-C to stop"
echo "============================================================"
echo ""

prev=""
while true; do
  val=$(curl -sf \
    "$ORION/ngsi-ld/v1/entities/$ENTITY?attrs=pendingIntent" \
    -H 'Accept: application/json' \
    | python3 -c "
import sys, json
try:
    d = json.load(sys.stdin)
    pi = d.get('pendingIntent', {}).get('value', {})
    print(json.dumps(pi, indent=2) if pi else '(empty)')
except Exception as e:
    print(f'(parse error: {e})')
" 2>/dev/null \
  || echo "(orion unreachable)")

  if [ "$val" != "$prev" ]; then
    ts=$(date +"%H:%M:%S.%3N")
    echo "[$ts] pendingIntent updated:"
    echo "$val"
    echo ""
    prev="$val"
  fi

  sleep "$POLL_INTERVAL"
done
