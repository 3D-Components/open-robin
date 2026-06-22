Troubleshooting Guide
=====================

This guide covers common issues when running the local reviewer/demo stack.
Start with the quick checks, then use the section matching the failing symptom.

Quick Checks
------------

.. code-block:: bash

   docker compose ps
   curl http://localhost:8001/health
   curl http://localhost:1026/version
   curl 'http://localhost:9090/temporal/entities?limit=1'

Expected local ports:

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Service
     - Host port
     - Notes
   * - ``orion-ld``
     - 1026
     - FIWARE Orion-LD context broker
   * - ``mongo-db``
     - 27017
     - Orion document store
   * - ``timescaledb``
     - 5433
     - TROE temporal store
   * - ``mintaka``
     - 9090
     - Temporal API, container port 8080
   * - ``alert-processor``
     - 8001
     - Process Intelligence API, container port 8000
   * - ``robin-dashboard``
     - 5174
     - Browser dashboard
   * - ``lichtblick``
     - 8080
     - 3D visualization UI

Docker Not Running
------------------

Symptoms:

* ``Cannot connect to the Docker daemon``
* ``docker compose ps`` fails before listing services

Checks:

.. code-block:: bash

   docker info
   docker compose version

Fix:

* Start Docker Desktop on macOS or the Docker daemon on Linux.
* Wait until ``docker info`` succeeds.
* Retry ``docker compose up -d``.

On Linux:

.. code-block:: bash

   sudo systemctl status docker
   sudo systemctl start docker

Ports Already In Use
--------------------

Symptoms:

* ``bind: address already in use``
* A service exits immediately after ``docker compose up -d``

Check which process owns a port:

.. code-block:: bash

   lsof -i :1026
   lsof -i :8001
   lsof -i :5174
   lsof -i :9090
   lsof -i :5433

Fix:

* Stop the local process using the conflicting port.
* Or edit the host port in ``docker-compose.yaml`` for local testing.
* Recreate the affected service:

.. code-block:: bash

   docker compose up -d --force-recreate SERVICE_NAME

UID, GID, or DISPLAY Not Set
----------------------------

The ``vulcanexus`` service runs as ``${UID}:${GID}`` and forwards
``${DISPLAY}`` for GUI/X11 tools.

Symptoms:

* ``The UID variable is not set``
* ``The GID variable is not set``
* GUI or RViz tools cannot open a display
* file permission errors in ``vulcanexus_ws/install``

Checks:

.. code-block:: bash

   echo "UID=${UID}"
   echo "GID=${GID}"
   echo "DISPLAY=${DISPLAY}"

Linux shell setup:

.. code-block:: bash

   env UID="$(id -u)" GID="$(id -g)" DISPLAY="${DISPLAY:-:0}" docker compose up -d

macOS note:

Docker Desktop does not provide Linux host-network and X11 behavior exactly like
native Linux.  Use ``docker-compose.macos.override.yaml`` for the bridged Orion
configuration, and prefer Linux for full ROS 2/Vulcanexus/RViz validation.

.. code-block:: bash

   docker compose -f docker-compose.yaml -f docker-compose.macos.override.yaml up -d

Orion Unavailable
-----------------

Symptoms:

* ``curl http://localhost:1026/version`` fails
* Alert Engine health reports Orion errors
* ``POST /create-process`` or intent publishing returns a broker error

Checks:

.. code-block:: bash

   docker compose ps orion-ld mongo-db timescaledb
   docker compose logs --tail=120 orion-ld
   curl http://localhost:1026/version

Common causes:

* Docker is not running.
* Port ``1026`` is already in use.
* MongoDB or TimescaleDB did not start.
* On macOS, host networking from the base compose file is not usable.

Fix:

.. code-block:: bash

   docker compose up -d mongo-db timescaledb
   docker compose up -d --force-recreate orion-ld

On macOS:

.. code-block:: bash

   docker compose -f docker-compose.yaml -f docker-compose.macos.override.yaml up -d --force-recreate orion-ld alert-processor mintaka

Mintaka Unavailable
-------------------

Symptoms:

* ``curl http://localhost:9090/temporal/entities?limit=1`` fails
* Dashboard has no history even though Orion contains entities
* API responses fall back to Orion instead of reporting Mintaka-backed data

Checks:

.. code-block:: bash

   docker compose ps mintaka timescaledb
   docker compose logs --tail=120 mintaka
   docker compose logs --tail=80 timescaledb
   curl 'http://localhost:9090/temporal/entities?limit=1'

Fix:

.. code-block:: bash

   docker compose up -d timescaledb
   docker compose up -d --force-recreate mintaka

If TimescaleDB was recreated with old state, use ``docker compose down -v`` only
when you intentionally want to delete local demo data.

Dashboard Cannot Reach API
--------------------------

Symptoms:

* Dashboard loads but service health shows the API as unavailable
* Browser console shows failed requests to ``localhost:8001``
* ``curl http://localhost:8001/health`` fails

Checks:

.. code-block:: bash

   docker compose ps alert-processor robin-dashboard
   docker compose logs --tail=120 alert-processor
   curl http://localhost:8001/health

The dashboard build uses ``VITE_ROBIN_API_URL``.  In the compose example it is
built with ``http://localhost:8001``.

Fix:

.. code-block:: bash

   docker compose build alert-processor robin-dashboard
   docker compose up -d --force-recreate alert-processor robin-dashboard

If you are using a custom host or port, rebuild the dashboard with the matching
``VITE_ROBIN_API_URL`` build argument.

No Telemetry Displayed
----------------------

Symptoms:

* Dashboard loads but charts stay empty
* API returns an empty measurement list
* Rosbag or profile demo appears to run but no process data appears

Checks:

.. code-block:: bash

   curl "http://localhost:8001/process/<process_id>/measurements?last=20"
   docker compose logs --tail=120 alert-processor
   docker compose logs --tail=120 orion-ld
   docker compose logs --tail=120 mintaka

For ROS 2/DDS runs, check the ROS topic inside the Vulcanexus container or on a
Linux ROS 2 host:

.. code-block:: bash

   ros2 topic list
   ros2 topic echo /robin/telemetry

Common causes:

* The process ID selected in the dashboard does not match the demo process ID.
* The profile demo is waiting for the dashboard Start button.
* Orion is receiving data but Mintaka has not indexed it yet.
* DDS mapping is not loaded or the ROS domain does not match.

Fix:

* Select the exact process ID printed by the demo script.
* Press **Start** in the dashboard, or run the demo with ``--no-prompt``.
* Wait 10 to 20 seconds for temporal indexing, then refresh the dashboard.
* Check the DDS mapping section below for ROS 2/DDS ingestion.

Model File Missing
------------------

Symptoms:

* API health or recommendation calls report a missing model/checkpoint
* ``POST /ai-recommendation`` fails
* logs mention ``model_path`` or scaler load failures

Checks:

.. code-block:: bash

   rg -n "model_path|scaler" config/profiles
   ls -lah data/models
   docker compose logs --tail=120 alert-processor

Fix:

* Confirm the active ``ROBIN_PROFILE`` value.
* Confirm the profile's ``ai.model_path`` exists under the mounted
  ``./data/models`` directory.
* Recreate the API container after changing files:

.. code-block:: bash

   docker compose up -d --force-recreate alert-processor

If a domain does not have a validated model, document that limitation and run
the process without relying on AI recommendations.

DDS Mapping Not Loading
-----------------------

Symptoms:

* ROS 2 ``/robin/telemetry`` exists but Orion does not show telemetry
* Orion starts without DDS mapping messages
* ``config-dds.json`` changes appear to have no effect

Checks:

.. code-block:: bash

   test -f config-dds.json
   python -m json.tool config-dds.json
   docker compose logs --tail=160 orion-ld | rg -i "dds|config|telemetry|error"
   docker inspect fiware-orion --format '{{ range .Mounts }}{{ .Source }} -> {{ .Destination }}{{ println }}{{ end }}'

Expected mapping:

* DDS topic: ``rt/robin/telemetry``
* entity: ``urn:ngsi-ld:Process:ros_bridge``
* attribute: ``urn:robin:processTelemetry``
* ROS domain: ``ROS_DOMAIN_ID=0`` in the compose example

Fix:

.. code-block:: bash

   docker compose up -d --force-recreate orion-ld vulcanexus

On macOS, the override disables the host-network DDS-oriented Orion setup.  Use
Linux for full DDS discovery validation, or document that the macOS run is
limited to API/FIWARE/dashboard checks.

Resetting Demo Data
-------------------

If old data appears in the dashboard, either use the demo cleanup script for one
process or reset local volumes when you intentionally want a clean slate.

.. code-block:: bash

   ./demo/cleanup-demo.sh PROCESS_ID
   docker compose down -v
   docker compose up -d --build

``docker compose down -v`` deletes local MongoDB and TimescaleDB demo data.

Getting Help
------------

When opening an issue, include:

1. operating system and CPU architecture;
2. exact command that failed;
3. ``docker compose ps`` output;
4. relevant logs from ``orion-ld``, ``mintaka``, ``alert-processor``, and
   ``robin-dashboard``;
5. active ``ROBIN_PROFILE`` and any compose override file used;
6. whether the run used API/demo data, ROS 2/DDS, rosbag replay, or hardware.

Do not include private datasets, credentials, site network details, or customer
configuration values in public issue reports.
