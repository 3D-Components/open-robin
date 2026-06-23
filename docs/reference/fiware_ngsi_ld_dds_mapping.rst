FIWARE / NGSI-LD and DDS Mapping Reference
==========================================

This page is the standalone interface reference for ROBIN's FIWARE,
NGSI-LD, and DDS integration.  It documents the current data model,
the DDS Enabler configuration, validation commands, and known mapping
limitations so reviewers do not need to infer the interface from source code.

Runtime Components
------------------

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Component
     - Local endpoint or file
     - Role
   * - Orion-LD
     - ``http://localhost:1026``
     - NGSI-LD context broker and DDS ingestion endpoint.
   * - MongoDB
     - ``localhost:27017``
     - Orion-LD document persistence.
   * - TimescaleDB / TROE
     - ``localhost:5433``
     - Temporal attribute store used by Orion-LD.
   * - Mintaka
     - ``http://localhost:9090``
     - Temporal query API over TROE history.
   * - Process Intelligence API
     - ``http://localhost:8001``
     - FastAPI layer for process lifecycle, measurements, deviation checks,
       alerts, recommendations, and dashboard-facing reads.
   * - ROBIN Dashboard
     - ``http://localhost:5174``
     - Browser UI for live monitoring, alerts, model controls, and history CSV
       export.
   * - DDS Enabler configuration
     - ``config-dds.json``
     - Orion-LD DDS mapping configuration mounted by ``docker-compose.yaml``.

Default local Compose configuration does not set ``NGSILD_TENANT`` for the API
or DDS path.  If a deployment enables tenants, use the same ``NGSILD-Tenant``
header consistently for Orion-LD, Mintaka, and API-side requests.

End-to-End Data Flow
--------------------

.. mermaid::

   graph LR
      SRC["ROS 2 robot / sensor topics"]
      AGG["telemetry_aggregator_node"]
      DDS["/robin/telemetry<br/>ProcessTelemetry"]
      ORION["Orion-LD<br/>DDS Enabler"]
      PROCESS["urn:ngsi-ld:Process:ros_bridge<br/>urn:robin:processTelemetry"]
      TSDB[("TimescaleDB / TROE")]
      MINTAKA["Mintaka temporal API"]
      API["Process Intelligence API"]
      UI["ROBIN Dashboard"]

      SRC --> AGG
      AGG --> DDS
      DDS --> ORION
      ORION --> PROCESS
      PROCESS --> TSDB
      TSDB --> MINTAKA
      MINTAKA --> API
      ORION --> API
      API --> UI

The primary DDS path is:

``ROS 2 topic -> DDS Enabler -> Orion-LD -> Mintaka/history -> API/dashboard``.

The API/demo path can also write directly to Orion-LD through REST.  That path
creates ``Measurement`` entities and patches scalar measurement attributes onto
the related ``Process`` entity so TROE/Mintaka can store history.

Entity Model Summary
--------------------

.. mermaid::

   erDiagram
      PROCESS ||--o{ MEASUREMENT : "processId"
      PROCESS ||--o| GEOMETRY_TARGET : "processId"
      PROCESS ||--o{ ALERT : "processId"
      PROCESS ||--o{ AI_RECOMMENDATION : "processId"

      PROCESS {
         string id
         string type
         string processStatus
         string operationMode
         object inputParams
         object pendingIntent
         object processTelemetry
      }

      MEASUREMENT {
         string id
         string type
         relationship processId
         number measuredHeight
         number measuredWidth
         number measuredSpeed
         number measuredCurrent
         number measuredVoltage
      }

      GEOMETRY_TARGET {
         string id
         string type
         relationship processId
         number targetHeight
         number targetWidth
      }

      ALERT {
         string id
         string type
         relationship processId
         string deviationType
         object expectedValue
         object measuredValue
         number deviationPercentage
      }

      AI_RECOMMENDATION {
         string id
         string type
         relationship processId
         object recommendedParams
         number confidence
         string modelVersion
      }

.. list-table::
   :header-rows: 1
   :widths: 18 28 27 27

   * - Entity
     - Entity ID pattern
     - Main properties / relationships
     - Producer and consumer
   * - ``Process``
     - ``urn:ngsi-ld:Process:PROCESS_ID``
     - ``processStatus``, ``operationMode``, ``startedAt``,
       ``stoppedAt``, ``stopReason``, ``inputParams``,
       ``pendingIntent``, ``simulationProgress``, ``processTelemetry`` or
       ``urn:robin:processTelemetry``, scalar measured attributes.
     - Created by API/CLI.  Read by API, dashboard, Orion-LD, Mintaka, and
       ROS/FIWARE intent bridge paths.
   * - ``Measurement``
     - ``urn:ngsi-ld:Measurement:MEASUREMENT_ID``
     - ``processId`` relationship, ``measuredHeight``, ``measuredWidth``,
       ``measuredSpeed``, ``measuredCurrent``, ``measuredVoltage``,
       ``inputParams``.  Measurement properties carry ``observedAt``.
     - Created by API/CLI demo path.  Used as Orion fallback when Mintaka
       history is not available.
   * - ``GeometryTarget``
     - ``urn:ngsi-ld:GeometryTarget:PROCESS_ID``
     - ``processId`` relationship, ``targetHeight``, ``targetWidth``.
     - Created or patched by ``POST /process/{process_id}/target`` and used
       by geometry-driven deviation checks.
   * - ``Alert``
     - ``urn:ngsi-ld:Alert:PROCESS_ID-TIMESTAMP``
     - ``processId`` relationship, ``deviationType``, ``expectedValue``,
       ``measuredValue``, ``deviationPercentage``, ``recommendedActions``,
       ``timestamp``.
     - Created by deviation checks.  Read by ``GET /process/{id}/alerts`` and
       the dashboard History / Reports export.
   * - ``AIRecommendation``
     - ``urn:ngsi-ld:AIRecommendation:PROCESS_ID-TIMESTAMP``
     - ``processId`` relationship, ``recommendedParams``, ``confidence``,
       ``modelVersion``, ``timestamp``.
     - Created by ``POST /ai-recommendation`` and stored as advisory evidence.

Type URNs used by the current implementation:

.. code-block:: text

   urn:robin:Process
   urn:robin:Measurement
   urn:robin:GeometryTarget
   urn:robin:Alert
   urn:robin:AIRecommendation

Implementation evidence:

* API endpoints and response shaping: ``robin/alert_engine.py``.
* FIWARE client and entity creation helpers: ``robin/cli.py``.
* Dashboard API types and fetch helpers:
  ``robin-dashboard/src/hooks/useRobinAPI.ts``.
* CSV export implementation:
  ``robin-dashboard/src/components/features/history/HistoryTab.tsx``.

Example NGSI-LD Payloads
------------------------

Process entity:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:Process:reviewer_demo_process",
     "type": "urn:robin:Process",
     "processStatus": {
       "type": "Property",
       "value": "active"
     },
     "operationMode": {
       "type": "Property",
       "value": "parameter_driven"
     },
     "inputParams": {
       "type": "Property",
       "value": {
         "wire_feed_speed_mpm_model_input": 10.5,
         "travel_speed_mps_model_input": 0.021,
         "arc_length_correction_mm_model_input": 1.5
       }
     }
   }

Measurement entity, created by the API/CLI demo path:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:Measurement:reviewer_demo_process-001",
     "type": "urn:robin:Measurement",
     "processId": {
       "type": "Relationship",
       "object": "urn:ngsi-ld:Process:reviewer_demo_process"
     },
     "measuredHeight": {
       "type": "Property",
       "value": 2.95,
       "unitCode": "MMT",
       "observedAt": "2026-06-16T12:01:05+00:00"
     },
     "measuredWidth": {
       "type": "Property",
       "value": 7.48,
       "unitCode": "MMT",
       "observedAt": "2026-06-16T12:01:05+00:00"
     },
     "measuredCurrent": {
       "type": "Property",
       "value": 234.9,
       "unitCode": "A",
       "observedAt": "2026-06-16T12:01:05+00:00"
     },
     "measuredVoltage": {
       "type": "Property",
       "value": 26.7,
       "unitCode": "V",
       "observedAt": "2026-06-16T12:01:05+00:00"
     }
   }

Geometry target entity:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:GeometryTarget:reviewer_demo_process",
     "type": "urn:robin:GeometryTarget",
     "processId": {
       "type": "Relationship",
       "object": "urn:ngsi-ld:Process:reviewer_demo_process"
     },
     "targetHeight": {
       "type": "Property",
       "value": 3.2,
       "unitCode": "MMT"
     },
     "targetWidth": {
       "type": "Property",
       "value": 7.0,
       "unitCode": "MMT"
     }
   }

Alert entity:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:Alert:reviewer_demo_process-1781611265000",
     "type": "urn:robin:Alert",
     "processId": {
       "type": "Relationship",
       "object": "urn:ngsi-ld:Process:reviewer_demo_process"
     },
     "deviationType": {
       "type": "Property",
       "value": "geometry"
     },
     "expectedValue": {
       "type": "Property",
       "value": {
         "height": 3.2,
         "width": 7.0
       }
     },
     "measuredValue": {
       "type": "Property",
       "value": {
         "height": 2.95,
         "width": 7.48
       }
     },
     "deviationPercentage": {
       "type": "Property",
       "value": 7.8125,
       "unitCode": "P1"
     }
   }

AI recommendation entity:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:AIRecommendation:reviewer_demo_process-1781611265",
     "type": "urn:robin:AIRecommendation",
     "processId": {
       "type": "Relationship",
       "object": "urn:ngsi-ld:Process:reviewer_demo_process"
     },
     "recommendedParams": {
       "type": "Property",
       "value": {
         "wire_feed_speed_mpm_model_input": 10.8,
         "travel_speed_mps_model_input": 0.019,
         "arc_length_correction_mm_model_input": 1.1,
         "confidence": 0.88
       }
     },
     "confidence": {
       "type": "Property",
       "value": 0.88
     },
     "modelVersion": {
       "type": "Property",
       "value": "v2.1"
     }
   }

DDS Enabler Configuration
-------------------------

The DDS Enabler mapping is defined in ``config-dds.json`` and mounted into
the Orion-LD container by ``docker-compose.yaml``:

.. code-block:: yaml

   volumes:
     - ./config-dds.json:/root/.orionld
   command: -dbhost 127.0.0.1 -logLevel DEBUG -troe -wip dds

Relevant ``config-dds.json`` excerpt:

.. code-block:: json

   {
     "dds": {
       "ddsmodule": {
         "dds": {
           "domain": 0,
           "allowlist": [
             {
               "name": "rt/robin/telemetry"
             }
           ]
         },
         "topics": {
           "name": "*",
           "qos": {
             "durability": "VOLATILE",
             "reliability": "BEST_EFFORT",
             "history-depth": 5000
           }
         }
       },
       "ngsild": {
         "topics": {
           "rt/robin/telemetry": {
             "entityType": "urn:robin:Process",
             "entityId": "urn:ngsi-ld:Process:ros_bridge",
             "attribute": "urn:robin:processTelemetry"
           }
         }
       }
     }
   }

Field meaning:

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Field
     - Meaning
   * - ``dds.ddsmodule.dds.domain``
     - DDS domain used by Orion-LD.  Must match the ROS 2 publisher domain.
       The Compose baseline uses ``ROS_DOMAIN_ID=0``.
   * - ``allowlist``
     - Limits Orion-LD DDS discovery to ``rt/robin/telemetry``.
   * - ``topics.qos``
     - DDS QoS used by the Orion-LD DDS module.  Current baseline is
       ``VOLATILE`` and ``BEST_EFFORT`` with history depth ``5000``.
   * - ``ngsild.topics.rt/robin/telemetry.entityType``
     - NGSI-LD entity type created or updated by the DDS topic.
   * - ``ngsild.topics.rt/robin/telemetry.entityId``
     - Fixed NGSI-LD entity receiving DDS telemetry in the current baseline.
   * - ``ngsild.topics.rt/robin/telemetry.attribute``
     - Compound NGSI-LD attribute that stores the ``ProcessTelemetry`` fields.

ROS 2 Message to NGSI-LD Attribute Mapping
------------------------------------------

The DDS publisher uses
``vulcanexus_ws/src/robin_interfaces/msg/ProcessTelemetry.msg``:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - ROS 2 field
     - NGSI-LD location
     - Notes
   * - ``current``
     - ``urn:robin:processTelemetry.value.current``
     - Welding current in ampere for the reference profile.
   * - ``voltage``
     - ``urn:robin:processTelemetry.value.voltage``
     - Arc voltage for the reference profile.
   * - ``speed``
     - ``urn:robin:processTelemetry.value.speed``
     - Wire feed speed in the reference profile.
   * - ``width``
     - ``urn:robin:processTelemetry.value.width``
     - Measured bead or process width.
   * - ``height``
     - ``urn:robin:processTelemetry.value.height``
     - Measured bead height or coating thickness equivalent.
   * - ``cross_sectional_area``
     - ``urn:robin:processTelemetry.value.cross_sectional_area``
     - Preserved in DDS payload when present.  Not all dashboard views use it.

API Endpoints and Expected Responses
------------------------------------

The API is served on host port ``8001`` in the local Compose stack.
Representative response fixtures are kept in ``media/api-responses/``.

.. list-table::
   :header-rows: 1
   :widths: 32 43 25

   * - Purpose
     - Command
     - Expected response marker
   * - Health check
     - ``curl http://localhost:8001/health``
     - ``"status": "healthy"``
   * - Create process
     - ``curl -X POST http://localhost:8001/create-process -H "Content-Type: application/json" -d @media/api-responses/create-process-request.json``
     - ``"status": "success"``
   * - Set geometry target
     - ``curl -X POST http://localhost:8001/process/reviewer_demo_process/target -H "Content-Type: application/json" -d "{\"height\":3.2,\"width\":7.0}"``
     - ``"mode_set": "geometry_driven"``
   * - Read measurements
     - ``curl "http://localhost:8001/process/reviewer_demo_process/measurements?last=5"``
     - ``"debug_info": {"source": "mintaka"}`` when Mintaka has data.
   * - Read alerts
     - ``curl "http://localhost:8001/process/reviewer_demo_process/alerts"``
     - ``"status": "success"``
   * - Check deviation
     - ``curl -X POST http://localhost:8001/check-deviation -H "Content-Type: application/json" -d @media/api-responses/check-deviation-request.json``
     - ``"deviation_percentage"`` and ``"recommended_actions"``.
   * - Request AI recommendation
     - ``curl -X POST http://localhost:8001/ai-recommendation -H "Content-Type: application/json" -d @media/api-responses/ai-recommendation-request.json``
     - ``"status": "success"``, ``"recommendation"``.

The response examples in ``media/api-responses/`` are illustrative evidence.
The reviewer quickstart should refresh or confirm them from a clean-clone run
before the final tag.

History and CSV Export
----------------------

History is API-backed:

* ``GET /process/{process_id}/measurements`` reads Mintaka first, then falls
  back to Orion ``Measurement`` entities, then TROE SQL fallback.
* ``GET /process/{process_id}/alerts`` reads persisted ``Alert`` entities from
  Orion-LD.
* The dashboard History / Reports tab combines those two responses into a CSV.

CSV export implementation:
``robin-dashboard/src/components/features/history/HistoryTab.tsx``.

Representative CSV evidence:
``media/exports/reviewer_demo_process-history-2026-06-16T12-02-00-000Z.csv``.

CSV rows use:

.. code-block:: text

   record_type,process_id,timestamp,alert_id,measurement_height_mm,
   measurement_width_mm,measurement_input_params_json,alert_severity,
   alert_deviation_type,alert_deviation_percentage,alert_expected_height_mm,
   alert_expected_width_mm,alert_measured_height_mm,alert_measured_width_mm,
   alert_recommended_actions,raw_payload_json,source

Validation Commands
-------------------

Validate the DDS configuration file:

.. code-block:: bash

   python -m json.tool config-dds.json >/dev/null && echo "config-dds.json OK"

Expected output:

.. code-block:: text

   config-dds.json OK

Validate local services:

.. code-block:: bash

   docker compose up -d
   docker compose ps
   curl http://localhost:1026/version
   curl http://localhost:9090/health
   curl http://localhost:8001/health

Expected output:

* Compose lists ``orion-ld``, ``mongo-db``, ``timescaledb``, ``mintaka``,
  ``alert-processor``, and ``robin-dashboard`` as running or healthy.
* Orion returns version metadata.
* Mintaka returns a health response.
* API health returns ``"status": "healthy"`` when Orion-LD and Mintaka are
  reachable.

Validate API/demo history path:

.. code-block:: bash

   PROCESS_ID=reviewer_demo_process
   python demo/profiles/welding_profile.py \
     --process-id "$PROCESS_ID" \
     --mode both \
     --duration 60 \
     --interval 1 \
     --no-prompt

   curl -s "http://localhost:8001/process/${PROCESS_ID}/measurements?last=5" \
     | jq '.status, .count, .debug_info.source'

Expected output after a successful run:

.. code-block:: text

   "success"
   5
   "mintaka"

If Mintaka is not ready, the API may return ``"orion"`` or ``"troe"`` as the
debug source.  That is a fallback path, not the preferred steady-state path.

Validate DDS topic and Orion-LD mapping on a Linux ROS 2/Vulcanexus setup:

.. code-block:: bash

   docker exec vulcanexus-bridge bash -lc \
     'source /opt/ros/jazzy/setup.bash 2>/dev/null || true;
      source /workspace/ros2_packages/install/setup.bash 2>/dev/null || true;
      ros2 topic list | grep /robin/telemetry'

   curl -s \
     "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge" \
     | jq '.id, .type, ."urn:robin:processTelemetry" // .processTelemetry'

Expected output:

* ``/robin/telemetry`` is listed after the telemetry aggregator is running.
* Orion returns ``urn:ngsi-ld:Process:ros_bridge`` with type
  ``urn:robin:Process``.
* The telemetry attribute contains the current ``ProcessTelemetry`` values.

Validate temporal history for DDS data:

.. code-block:: bash

   curl -s \
     "http://localhost:9090/temporal/entities/urn:ngsi-ld:Process:ros_bridge?attrs=urn:robin:processTelemetry,processTelemetry&timerel=between&timeAt=1970-01-01T00:00:00Z&endTimeAt=2035-01-01T00:00:00Z&timeproperty=observedAt&options=temporalValues&lastN=5" \
     | jq

Expected output:

* A JSON array with the ``ros_bridge`` process entity after DDS telemetry has
  been published and TROE has indexed it.
* An empty array means no DDS telemetry has been stored yet, or the time
  property/domain mapping does not match the running stack.

Known Limitations and Assumptions
---------------------------------

* ``config-dds.json`` maps ``rt/robin/telemetry`` to the fixed entity
  ``urn:ngsi-ld:Process:ros_bridge``.  Per-run or per-profile DDS entity IDs
  require changing or templating the DDS mapping.
* The DDS path expects ROS 2 publishers, Orion-LD, and Vulcanexus to use the
  same DDS domain.  The local Compose baseline uses domain ``0``.
* DDS QoS is ``BEST_EFFORT``.  This is suitable for telemetry visualization but
  should not be treated as guaranteed delivery for safety-critical control.
* The local macOS override is useful for API/FIWARE/dashboard checks, but full
  DDS discovery validation is best done on Linux with host networking.
* Tenant handling is disabled by default in the Compose baseline to keep DDS and
  TROE history aligned.  Deployments that enable ``NGSILD_TENANT`` must apply
  the same tenant consistently across Orion-LD, Mintaka, API calls, and tests.
* The ``Measurement`` entity path and scalar Process attributes support API/demo
  runs.  The DDS path stores a compound telemetry attribute on the Process
  entity instead.
* The committed API response and CSV files are representative examples.  Final
  release verification should refresh or confirm them from the approved
  quickstart/demo run.
