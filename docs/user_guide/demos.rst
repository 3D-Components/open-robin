Demo Scripts & Simulations
==========================

ROBIN now uses **two canonical demo scripts**:

* ``demo/profiles/welding_profile.py``
* ``demo/profiles/spray_coating_profile.py``

Each script validates both operational modes with real AI-backed deviation checks
and produces alert entities when simulation data deviates beyond tolerance.

Canonical Demos
---------------

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Script
     - What it validates
   * - ``python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2``
     - Welding profile, parameter-driven + geometry-driven in one run, telemetry updates, dashboard refresh, and real alert generation from ``POST /check-deviation``.
   * - ``python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2``
     - Spray coating profile, same dual-mode validation with domain-specific vocabulary and units.

Dual-Mode Validation Logic
--------------------------

**Parameter-driven**

1. Operator sets initial process parameters.
2. AI predicts expected geometry.
3. Telemetry is streamed.
4. Alert processor compares measured geometry to AI-predicted geometry.
5. Deviation alerts are generated when tolerance is exceeded.

**Geometry-driven**

1. Operator sets target geometry.
2. AI suggests process parameters for that target.
3. Telemetry is streamed.
4. Alert processor compares measured geometry against the expected geometry from
   AI-suggested parameters (falling back to target geometry if needed).
5. Deviation alerts are generated when tolerance is exceeded.

What to watch in the dashboard:

* **Measurement KPIs** update in real time.
* **Telemetry chart** continues streaming.
* **Deviation Monitor** toggles between OK/Deviation/ALERT with mode-aware source.
* **Alerts panel** receives warning/critical entries during injected deviation windows.

Rebuild + Verification Checklist
--------------------------------

Use this exact sequence when you need to ensure recent code changes are live and
the dashboard is plotting real Mintaka-stored data.

1. Rebuild and recreate backend + dashboard containers:

.. code-block:: bash

   docker compose build alert-processor robin-dashboard
   docker compose up -d --force-recreate alert-processor robin-dashboard
   docker compose ps alert-processor robin-dashboard

2. Start a non-interactive dual-mode welding demo:

.. code-block:: bash

   BASE="verify-$(date +%s)"
   python demo/profiles/welding_profile.py \
       --process-id "$BASE" \
       --mode both \
       --duration 60 \
       --interval 1 \
       --no-prompt

3. In the dashboard (`http://localhost:5174`), select ``${BASE}-parameter`` then
   ``${BASE}-geometry`` from the top bar process selector and confirm in **Live Ops -> Telemetry**:

* source chip shows **Mintaka stored data**
* poll chip shows **Poll 1s** (Active Run) or **Poll 2s** (Demo Mode)
* chart points keep advancing at that cadence

4. API verification (source must be Mintaka):

.. code-block:: bash

   curl -s "http://localhost:8001/process/${BASE}-parameter/measurements?last=5" | jq '.debug_info.source, .measurements'

Expected first output line:

.. code-block:: text

   "mintaka"

5. Optional raw Mintaka cross-check:

.. code-block:: bash

   curl -s "http://localhost:9090/temporal/entities/urn:ngsi-ld:Process:${BASE}-parameter?attrs=measuredHeight,measuredWidth,measuredSpeed,measuredCurrent,measuredVoltage&timeproperty=observedAt&timerel=between&timeAt=1970-01-01T00:00:00Z&endTimeAt=2035-01-01T00:00:00Z&options=temporalValues&lastN=5" | jq

Mode-specific runs
------------------

You can run one mode at a time:

.. code-block:: bash

   python demo/profiles/welding_profile.py --mode parameter_driven
   python demo/profiles/welding_profile.py --mode geometry_driven

   python demo/profiles/spray_coating_profile.py --mode parameter_driven
   python demo/profiles/spray_coating_profile.py --mode geometry_driven

Supporting Utilities (Non-canonical)
------------------------------------

The following scripts are still available for troubleshooting or specific
integration tasks, but they are not the primary demos:

* ``demo/validate-setup.sh`` - service health checks
* ``demo/cleanup-demo.sh`` - remove demo entities
* ``demo/simulation-demo-rosbag.sh`` - ROS 2 bag replay path
* ``demo/dashboard-simulation.py`` - headless dashboard data pump
* ``demo/interactive-demo.sh`` / ``demo/simulation-demo.sh`` / ``demo/smooth-simulation-demo.py`` - legacy walkthrough/sim variants

Cleanup
-------

.. code-block:: bash

   ./demo/cleanup-demo.sh weld-
   ./demo/cleanup-demo.sh coating-

For a full reset (Orion + Mintaka temporal history), see
:doc:`/RESET_DEMO_DATA`.
