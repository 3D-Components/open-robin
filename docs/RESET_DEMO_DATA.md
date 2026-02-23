# Reset Demo Data for Fresh Replay

Quick guide to clear all `ros_bridge` data before re-running the demo.

---

## Quick Reset (Copy-Paste)

Run this single command to clear everything:

```bash
# Clear TROE temporal data + restart Orion to reset entity
docker exec fiware-timescaledb psql -U orion -d orion -c \
  "DELETE FROM attributes WHERE entityid = 'urn:ngsi-ld:Process:ros_bridge';" && \
docker exec db-mongo mongo orion --quiet --eval \
  "db.entities.deleteOne({'_id.id': 'urn:ngsi-ld:Process:ros_bridge'})" && \
docker compose restart orion-ld && \
sleep 5 && \
echo "✅ Data cleared. Ready for fresh demo."
```

---

## What Gets Cleared

| Storage | Data | Command |
|---------|------|---------|
| **TimescaleDB (TROE)** | Historical measurements | `DELETE FROM attributes WHERE entityid = '...'` |
| **MongoDB** | Current entity state | `db.entities.deleteOne(...)` |
| **Orion Cache** | In-memory state | `docker compose restart orion-ld` |

---

## Step-by-Step (If You Want to Understand)

### 1. Clear Temporal Data (TROE/TimescaleDB)

```bash
docker exec fiware-timescaledb psql -U orion -d orion -c \
  "DELETE FROM attributes WHERE entityid = 'urn:ngsi-ld:Process:ros_bridge';"
```

This removes all historical measurements from the time-series database.

### 2. Delete Entity from MongoDB

```bash
docker exec db-mongo mongo orion --quiet --eval \
  "db.entities.deleteOne({'_id.id': 'urn:ngsi-ld:Process:ros_bridge'})"
```

This removes the current entity state.

### 3. Restart Orion (Clear Cache)

```bash
docker compose restart orion-ld
sleep 5  # Wait for Orion to be ready
```

### 4. Verify Clean State

```bash
# Should return 404 (entity doesn't exist)
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge" | jq '.title'

# Should return empty measurements
curl -s "http://localhost:8001/process/ros_bridge/measurements?last=5" | jq '.count'
```

Expected output:
```
"Entity Not Found"
0
```

---

## Now Run the Demo

```bash
# DDS telemetry demo mode
./demo/simulation-demo-rosbag.sh
```

The demo script will automatically create a fresh entity.

**Note**: the current baseline uses a DDS-first path with `ProcessTelemetry` aggregation.

---

## Troubleshooting

### "Entity already exists" but data looks old

The entity exists in MongoDB but might have stale attributes. Run the full reset above.

### Data still showing after reset

Mintaka might be caching. Wait 10-15 seconds or restart Mintaka:

```bash
docker compose restart mintaka
```

### Permission denied on docker commands

Run in a terminal with docker group access:

```bash
newgrp docker
# Then run the reset commands
```

---

## Create an Alias (Optional)

Add to your `~/.bashrc`:

```bash
alias reset-robin-demo='docker exec fiware-timescaledb psql -U orion -d orion -c "DELETE FROM attributes WHERE entityid = '\''urn:ngsi-ld:Process:ros_bridge'\'';" && docker exec db-mongo mongo orion --quiet --eval "db.entities.deleteOne({'\''_id.id'\'': '\''urn:ngsi-ld:Process:ros_bridge'\''})" && cd /path/to/open-robin && docker compose restart orion-ld && sleep 5 && echo "✅ Reset complete"'
```

Then just run:
```bash
reset-robin-demo
```
