User Guide Overview
===================

Welcome to the ROBIN user guide.

What ROBIN Provides
-------------------

ROBIN ships two reusable modules and a documented integration pattern:

* **Module 1 - Process Intelligence API**: process lifecycle, deviation detection, AI recommendations
* **Module 2 - Monitoring Dashboard**: configurable operator UI with live KPIs and alerts
* **Integration pattern - ROS 2 to FIWARE**: telemetry aggregation and DDS bridge config for connecting any ROS 2 robot to the FIWARE data layer

The components are domain-agnostic and demonstrated across welding and spray coating
profiles.

Core Concepts
-------------

Entities in Orion-LD:

* **Process**: operation mode, parameters, tolerance, state
* **Measurement**: time-stamped measured values
* **GeometryTarget**: target dimensions for geometry-driven operation
* **AIRecommendation**: AI model output and guidance

Operation Modes
---------------

**Parameter-driven**

* operator sets process parameters
* AI predicts expected geometry from those parameters
* telemetry is streamed
* alert engine compares measured geometry against AI prediction
* warnings/alerts are generated on true deviation beyond tolerance

**Geometry-driven**

* operator sets target geometry
* AI suggests process parameters for that target
* telemetry is streamed
* alert engine checks measured geometry against expected geometry from AI-guided settings
  (fallback: explicit target geometry if needed)
* warnings/alerts are generated on true deviation beyond tolerance

Daily Workflow
--------------

.. mermaid::

   graph LR
       START["docker compose up -d"] --> CREATE["Create / select\na process"]
       CREATE --> STREAM["Stream measurements\n(CLI or ROS 2 DDS)"]
       STREAM --> MONITOR["Monitor in\nROBIN Dashboard"]
       MONITOR --> DECIDE{"Deviation\ndetected?"}
       DECIDE -->|yes| REC["Review AI\nrecommendation"]
       REC --> APPLY["Apply / reject\nparameter change"]
       APPLY --> STREAM
       DECIDE -->|no| STREAM

1. Start stack (``docker compose up -d``)
2. Create/select a process
3. Stream measurements (CLI or ROS 2 DDS path)
4. Monitor deviations and alerts in the ROBIN Dashboard
5. Apply recommendations and iterate

Interfaces
----------

* **ROBIN Dashboard** (`http://localhost:5174`): operator cockpit
* **Alert Engine API** (`http://localhost:8001/docs`): integration and automation
* **Orion-LD API** (`http://localhost:1026/ngsi-ld/v1`): direct NGSI-LD access
* **Mintaka API** (`http://localhost:9090/temporal`): temporal data queries

ROS 2 / DDS Path
----------------

The current ingestion baseline is DDS-first:

* ROS topics are aggregated into ``/robin/telemetry``
* Orion DDS mapping in ``config-dds.json`` writes into ``urn:robin:processTelemetry``

Legacy HTTP NGSI bridge is intentionally removed from the recommended workflow.

Profiles
--------

The core is domain-agnostic.  Domain-specific demos are provided as **profiles**:

* **Welding** (reference): ``python demo/profiles/welding_profile.py --mode both``
* **Spray Coating**: ``ROBIN_PROFILE=spray_coating docker compose up -d`` then
  ``python demo/profiles/spray_coating_profile.py --mode both``

Each demo waits for you to press **Start** from the dashboard before streaming.

Each profile maps domain terms onto the same five core measurement fields
(``measuredHeight``, ``measuredWidth``, ``measuredSpeed``, ``measuredCurrent``,
``measuredVoltage``).  See ``demo/profiles/README.md`` for the comparison table
and instructions for creating new profiles.

For publication details, see:

* ``ARISE_PUBLICATION_ROADMAP.md``
* ``arise/catalog-metadata.yaml``
