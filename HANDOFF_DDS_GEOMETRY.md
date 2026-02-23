# Handoff: DDS Telemetry Aggregation Baseline

## Status

Resolved and standardized on **DDS-first telemetry ingestion**.

Date updated: **February 22, 2026**

## Current Approach

1. ROS topics are aggregated by `telemetry_aggregator_node.py`
2. Aggregated message type: `ProcessTelemetry.msg`
3. Orion DDS module maps `/robin/telemetry` to:
   - entity: `urn:ngsi-ld:Process:ros_bridge`
   - type: `urn:robin:Process`
   - attribute: `urn:robin:processTelemetry`

## Key Files

- `vulcanexus_ws/src/robin_interfaces/msg/ProcessTelemetry.msg`
- `vulcanexus_ws/src/robin_core_data/scripts/telemetry_aggregator_node.py`
- `config-dds.json`
- `demo/simulation-demo-rosbag.sh`

## Legacy Path

The HTTP NGSI bridge node is removed from the active baseline and should not be used for new demos.

## Verification Commands

```bash
# ROS telemetry stream
ros2 topic echo /robin/telemetry

# Orion entity check
curl -H "NGSILD-Tenant: robin" \
  "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge"

# Alert Engine measurement API
curl "http://localhost:8001/process/ros_bridge/measurements?last=5"
```

## Operational Notes

- Keep `ROS_DOMAIN_ID` aligned between Vulcanexus and Orion.
- Use host networking for DDS discovery in this deployment setup.
- Domain-specific topic names in source bags are acceptable; mapped storage remains process-generic.

