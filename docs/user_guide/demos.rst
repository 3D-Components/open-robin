Demo Scripts & Simulations
==========================

Use :doc:`/quickstart` first when you only need the minimal no-hardware hello
world. This page is the basic simulated demo path: it shows meaningful behavior
beyond installation by exercising live telemetry, dashboard interaction,
deviation checks, alerts, and history verification.

ROBIN now uses **two canonical demo scripts**:

* ``demo/profiles/welding_profile.py``
* ``demo/profiles/spray_coating_profile.py``

Both scripts validate both operational modes and produce alert entities when simulation data
deviates beyond tolerance.  The **welding** script is AI-backed end to end (it ships with a
trained model).  The **spray-coating** script demonstrates configuration / vocabulary reuse and
deviation-against-target detection; the repository does not commit a spray-coating model, so its
deviation reference is the target geometry rather than an AI prediction.  For the minimal
no-hardware hello world to run first, see :doc:`/quickstart`.

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
     - Spray coating profile, same dual-mode flow with domain-specific vocabulary and units. Validates configuration reuse and deviation-against-target detection (no committed spray model; deviation reference is the target geometry).

Live ROS 2 Demo (intent-driven robot)
-------------------------------------

The canonical profile demos above stream telemetry straight into the FIWARE layer
over HTTP.  For an **intent-driven** demonstration - where a simulated UR10e welds a
bead and synthetic telemetry travels the full **ROS 2 -> DDS -> FIWARE** path - run:

.. code-block:: bash

   ./demo/simulation-demo-ros2-live.sh

By default this starts the **lite** simulation (no Gazebo, MoveIt, controllers, or GPU,
so it comes up in seconds): ``robot_state_publisher`` renders the full welding cell -
the UR10e arm with its **Fronius weld torch**, **Garmo laser profilometer**, and the
**welding table** - while pure-Python mock skills animate the arm and stream synthetic
telemetry.  One alternative mode is available:

* ``--legacy-python`` - the pure-Python HTTP telemetry path (``demo/profiles/welding_profile.py``),
  which streams telemetry straight into FIWARE over HTTP without the ROS 2 robot animation.

The script brings up the FIWARE stack and the intent pipeline
(``welding_http_bridge`` + ``welding_supervisor`` + skill action servers).  The dashboard
buttons then drive the robot through the ROS4HRI intent path
(see :doc:`/reference/ros4hri_ros4ri_alignment`):

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Button (intent)
     - Behavior
   * - **Start** (``START_PROCESS``)
     - The seam skill welds the bead: it animates the arm along the seam and streams
       synthetic ``ProcessTelemetry`` to ``/robin/telemetry`` -> DDS ->
       ``urn:ngsi-ld:Process:ros_bridge`` and back to the dashboard.  The **wire feed
       speed** and **travel speed** you set in the dashboard drive the seam goal
       (``wire_feed_rate`` and ``weld_speed``) and the reported telemetry, so the
       numbers you enter are the numbers the robot runs.
   * - **Pause** (``PAUSE_PROCESS``)
     - The arm **freezes in place** at the current waypoint and telemetry stops. It does
       not return home.
   * - **Resume** (``RESUME_PROCESS``)
     - The weld **continues** from where it froze.
   * - **Stop** (``STOP_PROCESS``)
     - The active goals are cancelled and the arm returns to its **home** position.
   * - **Abort** (``ESTOP``)
     - The arm returns **home** and an **error** appears in the Alerts panel (also
       published on the ``/weld_errors`` topic).
   * - **Launch new DOE** (``LAUNCH_NEW_DOE``)
     - A new design-of-experiments run supersedes the current one: the supervisor
       announces it on ``/doe/launch``, **stops the weld, and returns the arm home**.

Select the ``ros_bridge`` process in the dashboard and watch the cell in the embedded
**Lichtblick** 3D view (``http://localhost:8080``, fed by ``foxglove_bridge`` on
``ws://localhost:8765``).  Telemetry can be inspected directly:

.. code-block:: bash

   curl -s 'http://localhost:8001/process/ros_bridge/measurements?last=5' | jq
   docker exec vulcanexus-bridge bash -lc \
     'source /workspace/ros2_packages/install/setup.bash && ros2 topic echo /robin/telemetry'

.. note::

   **Intent delivery is de-duplicated.**  The dashboard publishes intents through
   Orion-LD, whose at-least-once notifications can repeat a single button press.  The
   ``welding_http_bridge`` drops duplicates (keyed on the intent's ``observedAt``
   timestamp) so a repeated or late ``START`` cannot restart a weld you have already
   stopped.

.. note::

   DDS discovery requires the ROS 2 nodes and Orion-LD to share a DDS domain - both
   Compose services and ``config-dds.json`` use ``ROS_DOMAIN_ID=0``.

Dual-Mode Validation Logic
--------------------------

**Parameter-driven**

1. Operator sets initial AI inputs. In the welding demo these are wire feed
   speed, travel speed, and arc length correction.
2. AI predicts expected geometry.
3. Telemetry is streamed.
4. Alert processor compares measured geometry to AI-predicted geometry.
5. Deviation alerts are generated when tolerance is exceeded.

**Geometry-driven**

1. Operator sets target geometry.
2. AI suggests AI input values for that target. In the welding demo, those are
   wire feed speed, travel speed, and arc length correction.
3. Telemetry is streamed.
4. Alert processor compares measured geometry against the expected geometry from
   AI-suggested inputs.
5. In geometry-driven mode, deviation checks use fixed AI-guided/setpoint
   inputs for the run (not per-sample live measured telemetry).
6. Reason: this keeps the deviation metric anchored to the planned operating
   point and avoids conflating geometric quality with control-loop jitter.
7. Deviation alerts are generated when tolerance is exceeded.

Press Start to Begin
--------------------

When you run a demo script (without ``--no-prompt``), it will create the
process and configure AI expectations, then **pause and wait for you to press
Start from the dashboard UI**. The terminal will print::

   Waiting for Start from the dashboard UI for process "<id>"...
     Open http://localhost:5174, select process "<id>", and press Start.

In the dashboard:

1. Select the process from the top-bar dropdown.
2. Click **Start** in the Robot Control panel.

The simulation detects the start signal and begins streaming telemetry. The
dashboard progress bar tracks the actual simulation elapsed time (not just
measurement count).

.. tip::

   Pass ``--no-prompt`` to skip the Start-button wait and stream data
   immediately. This is useful for CI, scripted verification, or when you
   want the demo to start unattended.

What to watch in the dashboard:

* **Progress bar** advances with actual simulation progress (elapsed / total).
* **Measurement KPIs** update in real time.
* **Telemetry chart** continues streaming.
* **3D Visualization** shows the UR10e cell (arm + torch + profilometer on the
  welding table) performing a welding sweep driven by simulation progress.
* **Deviation Monitor** toggles between OK/Deviation/ALERT with mode-aware source.
* **Alerts panel** receives warning/critical entries during injected deviation windows.

Rebuild After Code Changes
--------------------------

When you change ``robin/``, ``robin-dashboard/``, or ``robin-ui/services/``, rebuild and
recreate the affected containers before re-running a demo:

.. code-block:: bash

   docker compose build alert-processor robin-dashboard robin-viser
   docker compose up -d --force-recreate alert-processor robin-dashboard robin-viser
   docker compose ps alert-processor robin-dashboard robin-viser

For the minimal scripted no-hardware run (health, profile, persisted measurements,
persisted alerts, live deviation, and the Mintaka source check), see :doc:`/quickstart`.

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
* ``demo/simulation-demo-rosbag-wfs-alc-ts.sh`` - current ROS 2 bag replay path
* ``demo/simulation-demo-rosbag.sh`` - compatibility wrapper for the current replay path

Cleanup
-------

.. code-block:: bash

   ./demo/cleanup-demo.sh weld-
   ./demo/cleanup-demo.sh coating-

For a full reset (Orion + Mintaka temporal history), see
:doc:`/RESET_DEMO_DATA`.
