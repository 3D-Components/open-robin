Quick Start Guide
=================

Launch the full ROBIN stack and see live telemetry in under two minutes.

What You Will See
-----------------

By the end of this guide you will have:

* A complete FIWARE data layer (Orion-LD, TimescaleDB, Mintaka) running in Docker
* The **Alert Engine** (FastAPI) serving deviation detection and AI predictions
* The **ROBIN Dashboard** (React) showing live KPIs, time-series charts, robot
  control, 3D visualisation, and deviation monitoring
* A **3D Visualization** server (Viser) rendering a UR5 robot with live
  welding animation driven by simulation progress
* Simulated process telemetry flowing through the entire stack in real time

.. mermaid::

   graph LR
       subgraph stack ["docker compose up -d"]
           ORION["Orion-LD<br/>Context Broker"]
           TSDB[("TimescaleDB")]
           MINTAKA["Mintaka<br/>Temporal API"]
           ALERT["Alert Engine<br/>+ AI Model"]
           VISER["Viser<br/>3D Visualization"]
           DASH["ROBIN Dashboard"]
       end
       SIM["Simulation<br/>Script"] -->|NGSI-LD| ORION
       ORION --> TSDB
       TSDB --> MINTAKA
       ALERT --> ORION
       ALERT --> MINTAKA
       DASH --> ALERT
       DASH -->|WebSocket| VISER

Prerequisites
-------------

* **Docker** and **Docker Compose** installed and running
* **Python 3.12+** with `Poetry <https://python-poetry.org/>`_
* ROBIN dependencies installed (``poetry install``)

See :doc:`installation` for full setup instructions.

Step 1 - Launch the Stack
-------------------------

.. code-block:: bash

   docker compose up -d

Verify all services are healthy:

.. code-block:: bash

   docker compose ps

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Service
     - Port
     - Role
   * - ``orion-ld``
     - 1026
     - NGSI-LD context broker (FIWARE)
   * - ``mongo-db``
     - 27017
     - Orion-LD persistence
   * - ``timescaledb``
     - 5433
     - Temporal storage (TROE)
   * - ``mintaka``
     - 9090
     - Temporal query API
   * - ``alert-processor``
     - 8001
     - Process Intelligence API (Module 1)
   * - ``robin-dashboard``
     - 5174
     - Operator Dashboard (Module 2)
   * - ``robin-viser``
     - 8081, 8082
     - 3D Visualization (Viser + WebSocket bridge)
   * - ``vulcanexus``
     - -
     - ROS 2 / DDS bridge container

Quick health check:

.. code-block:: bash

   curl http://localhost:8001/health

Step 2 - Open the Dashboard
----------------------------

Open http://localhost:5174 in your browser.

.. image:: _static/screenshots/dashboard-live-ops.png
   :alt: ROBIN Dashboard - Live Ops cockpit
   :align: center
   :width: 100%

This is the **Live Ops** cockpit - the main operator view.  It's empty right
now because no data is flowing yet.  For a full walkthrough of every panel, see
:doc:`user_guide/dashboard`.

Step 3 - Start a Demo
----------------------

Run the canonical welding demo (parameter-driven mode). The script will
create the process, configure AI expectations, then **wait for you to
press Start from the dashboard UI** before streaming data:

.. code-block:: bash

   python demo/profiles/welding_profile.py \
       --process-id demo-quickstart \
       --mode parameter_driven \
       --duration 60 \
       --interval 0.3

The terminal will print::

   Waiting for Start from the dashboard UI for process "demo-quickstart"...
     Open http://localhost:5174, select process "demo-quickstart", and press Start.

Step 4 - Press Start
---------------------

1. In the dashboard, select **demo-quickstart** from the process selector
   dropdown in the top bar.
2. Click the **Start** button in the Robot Control panel.
3. The simulation script detects the start signal and begins streaming
   telemetry.

You will see:

* The **progress bar** advancing in real time with the actual simulation
  progress (elapsed time / total duration).
* **Telemetry charts** and **KPI cards** updating live.
* The **3D visualization** showing the UR5 robot performing a welding sweep
  along the seam, driven by the simulation progress.
* **Deviation alerts** firing during injected deviation windows.

.. tip::

   Use ``--no-prompt`` to skip the Start-button wait and stream data
   immediately (useful for CI or scripted verification runs).

.. image:: _static/screenshots/dashboard-with-data.png
   :alt: ROBIN Dashboard - process selector with multiple processes
   :align: center
   :width: 100%

Step 5 - Clean Up
------------------

Remove demo data without stopping the stack:

.. code-block:: bash

   ./demo/cleanup-demo.sh demo-quickstart

Or tear everything down:

.. code-block:: bash

   docker compose down -v

After Code Changes (Rebuild + Sanity Check)
-------------------------------------------

If you changed ``robin/``, ``robin-dashboard/``, or ``robin-ui/services/``, refresh
the running services:

.. code-block:: bash

   docker compose build alert-processor robin-dashboard robin-viser
   docker compose up -d --force-recreate alert-processor robin-dashboard robin-viser

Then run a short verification demo and confirm UI data source/cadence:

.. code-block:: bash

   BASE="verify-$(date +%s)"
   python demo/profiles/welding_profile.py \
       --process-id "$BASE" \
       --mode both \
       --duration 60 \
       --interval 1 \
       --no-prompt

In the dashboard (``http://localhost:5174``), select ``${BASE}-parameter`` and
``${BASE}-geometry`` and verify the Telemetry panel shows:

* **Mintaka stored data** source chip
* **Poll 1s** (Active Run) or **Poll 2s** (Demo Mode)

API confirmation:

.. code-block:: bash

   curl -s "http://localhost:8001/process/${BASE}-parameter/measurements?last=5" | jq '.debug_info.source'

Expected:

.. code-block:: text

   "mintaka"

What's Next
-----------

Now that the stack is running, explore ROBIN in depth:

* :doc:`user_guide/dashboard` - full walkthrough of every dashboard panel
* :doc:`user_guide/api` - REST API exploration and process lifecycle management
* :doc:`user_guide/ai_models` - AI model management, training, and trust
* :doc:`user_guide/profiles` - switch domain profiles (welding, spray coating, ...)
* :doc:`user_guide/demos` - canonical welding/spray dual-mode demos
