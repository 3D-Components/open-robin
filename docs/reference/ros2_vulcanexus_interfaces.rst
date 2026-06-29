ROS 2 and Vulcanexus Interface Reference
========================================

This page is the central ROS 2/Vulcanexus interface reference for
``open-robin``. The workspace is the **hardware-agnostic open module**: an
intent-driven welding demo that runs on synthetic data with no physical
hardware, MoveIt, or Gazebo. The package names below are the current source of
truth for the documentation and validation commands you will use when
integrating the modules.

Runtime Baseline
----------------

.. list-table::
   :header-rows: 1
   :widths: 24 36 40

   * - Item
     - Current value
     - Evidence
   * - ROS 2 distribution
     - Jazzy
     - ``vulcanexus_ws/ws_setup.sh`` sources
       ``/opt/vulcanexus/jazzy/setup.bash``.
   * - Vulcanexus base image
     - ``eprosima/vulcanexus:jazzy-desktop-4.3.1``
     - ``vulcanexus_ws/Dockerfile``.
   * - ROS workspace
     - ``/workspace/ros2_packages``
     - Mounted from ``vulcanexus_ws/src`` by ``docker-compose.yaml``.
   * - Compose service
     - ``vulcanexus`` / container ``vulcanexus-bridge``
     - ``docker-compose.yaml``.
   * - Primary setup entry point
     - ``source /workspace/ros2_packages/ws_setup.sh``
     - Used by the launch and validation commands below.
   * - FIWARE DDS mapping file
     - ``config-dds.json``
     - Mounted into Orion-LD as ``/root/.orionld``.

DDS discovery requires Orion-LD and the ROS 2 publishers to use the same DDS
domain. The checked-in DDS mapping file uses domain ``0``, and both Compose
services (``orion-ld`` and ``vulcanexus``) set ``ROS_DOMAIN_ID=0`` so Orion-LD
discovers ``/robin/telemetry`` out of the box. If you change one, change all
three. The FIWARE mapping details are documented in
``docs/reference/fiware_ngsi_ld_dds_mapping.rst``.

Package Inventory
-----------------

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Package
     - Role
   * - ``welding_msgs``
     - Intent message and skill action definitions for the intent pipeline.
   * - ``welding_http_bridge``
     - HTTP / NGSI-LD (Orion-LD) to ROS 2 bridge; publishes ``/intents``.
   * - ``welding_supervisor``
     - Intent-to-skill mission controller; routes ``/intents`` to skill actions.
   * - ``welding_home_skill``
     - Lifecycle skill: move the arm to home (simulation by default).
   * - ``welding_seam_skill``
     - Lifecycle skill: execute a weld seam and publish ``ProcessTelemetry``.
   * - ``welding_manual_skill``
     - Lifecycle skill: adjust process parameters (clamped to safe ranges).
   * - ``welding_recommendation_skill``
     - Lifecycle skill: request AI welding parameters over HTTP.
   * - ``welding_demo``
     - Launch orchestrator; ``welding_robin_sim.launch.py`` is the no-hardware demo.
   * - ``robin_interfaces``
     - Generic ROBIN ROS 2 messages, services, and actions (telemetry schema +
       the richer set used by hardware-mode integrations).
   * - ``robin_description``
     - Hardware-agnostic URDF/xacro and meshes for the cell (UR10e + Fronius
       torch + Garmo profilometer + table). No nodes.

Package Layer Diagram
---------------------

.. mermaid::

   graph TD
      UI["ROBIN Dashboard"]
      API["Process Intelligence API"]
      ORION["Orion-LD / NGSI-LD"]
      HTTP["welding_http_bridge"]
      INTENT["/intents<br/>welding_msgs/msg/Intent"]
      SUP["welding_supervisor"]
      SKILLS["welding_* skill action servers"]
      HW["hardware-mode backends<br/>(adopter-provided)"]
      TEL["/robin/telemetry<br/>robin_interfaces/msg/ProcessTelemetry"]
      MINTAKA["Mintaka / TROE history"]

      UI --> API
      API --> ORION
      ORION --> HTTP
      UI --> HTTP
      HTTP --> INTENT
      INTENT --> SUP
      SUP --> SKILLS
      SKILLS -. "use_simulation:=false" .-> HW
      SKILLS --> TEL
      TEL --> ORION
      ORION --> MINTAKA
      MINTAKA --> API

Nodes and Executables
---------------------

.. list-table::
   :header-rows: 1
   :widths: 24 24 52

   * - Package
     - Executable or node
     - Main interface / purpose
   * - ``welding_http_bridge``
     - ``welding_http_bridge``
     - HTTP server on ``:8766`` (``/intent``, Orion notifications); publishes
       ``/intents``.
   * - ``welding_supervisor``
     - ``welding_supervisor``
     - Subscribes ``/intents``; routes to skill action servers; publishes
       ``/doe/launch`` and ``/weld_errors``; calls ``weld/pause``.
   * - ``welding_home_skill``
     - ``welding_home_skill``
     - Action ``welding_home_skill/execute`` (``MoveToHome``); owns
       ``/joint_states`` in simulation.
   * - ``welding_seam_skill``
     - ``welding_seam_skill``
     - Action ``welding_seam_skill/execute`` (``ExecuteSeam``); publishes
       ``/robin/telemetry``; hosts ``weld/pause``.
   * - ``welding_manual_skill``
     - ``welding_manual_skill``
     - Action ``welding_manual_skill/execute`` (``ManualAdjust``); hardware mode
       calls ``/fronius/set_*``.
   * - ``welding_recommendation_skill``
     - ``welding_recommendation_skill``
     - Action ``welding_recommendation_skill/execute``
       (``RequestAIRecommendation``); HTTP to ``ROBIN_API_URL``.

The skills run in simulation by default. A ``use_simulation:=false`` hardware
mode delegates to generic interfaces (``/move_home``, ``/execute_bead``,
``/fronius/set_*``) that an adopter implements for their own robot and power
source.

Messages
--------

``welding_msgs/msg``
~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Message
     - Purpose
   * - ``Intent``
     - Intent bus message: type, JSON data, source, modality, priority,
       confidence.

``robin_interfaces/msg``
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Message
     - Purpose
   * - ``ProcessTelemetry``
     - DDS/FIWARE telemetry snapshot: bead ID, progression, height, width,
       speed, current, voltage, and cross-sectional area.
   * - ``BeadGeometry`` / ``WeldProgression`` / ``WelderData`` / ``ActiveBead``
     - Weld geometry, progression, and welder data.
   * - ``ExperimentBead`` / ``ExperimentBeadSpec`` / ``PlateLayout`` / ``WeldPlate``
     - Experiment and plate definitions.

Actions
-------

.. list-table::
   :header-rows: 1
   :widths: 30 26 44

   * - Action
     - Package
     - Goal
   * - ``ExecuteSeam``
     - ``welding_msgs``
     - Cartesian weld-seam goal (seam_id, weld_speed, wire_feed_rate).
   * - ``MoveToHome``
     - ``welding_msgs``
     - Home the arm (use_fast_speed).
   * - ``ManualAdjust``
     - ``welding_msgs``
     - Adjust a process parameter (name, value, unit).
   * - ``RequestAIRecommendation``
     - ``welding_msgs``
     - Request AI welding parameters (process_id, mode).
   * - ``ExecuteBead`` / ``WeldExperiment``
     - ``robin_interfaces``
     - Bead / experiment execution targets for hardware-mode integrations.

Services
--------

``robin_interfaces`` provides the generic typed setters and calibration/planning
services used by hardware-mode integrations, including ``SetFloat32`` /
``SetInt32`` / ``SetBool`` (typed setters, e.g. ``/fronius/set_*``), ``SetCtwd``,
``SetTcpMode``, ``FindSurface``, ``CalibrateWireTip``, ``CalibratePlatePlane``,
``StartWeld``, ``PlanExperiment``, ``ApproveExperimentPlan``, ``ReserveSlots``,
and ``ClearReservedSlots``. The supervisor and seam skill use ``weld/pause``
(``std_srvs/SetBool``).

Core Topics
-----------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Type
     - Role
   * - ``/intents``
     - ``welding_msgs/msg/Intent``
     - Operator/dashboard intents into the supervisor.
   * - ``/robin/telemetry``
     - ``robin_interfaces/msg/ProcessTelemetry``
     - Normalized telemetry to the Orion-LD DDS bridge.
   * - ``/joint_states`` / ``/joint_states_manual``
     - ``sensor_msgs/msg/JointState``
     - Arm pose (home skill owns ``/joint_states``; seam skill animates via
       ``/joint_states_manual``).
   * - ``/doe/launch`` / ``/weld_errors``
     - ``std_msgs/msg/String``
     - DOE launch notifications and operator-visible errors.

ROS 2 to FIWARE Data Path
-------------------------

The reusable telemetry integration path is:

``ROS 2 source -> /robin/telemetry -> Orion-LD DDS -> NGSI-LD Process entity -> TROE/Mintaka history -> API/dashboard``.

In the no-hardware demo ``welding_seam_skill`` publishes synthetic
``robin_interfaces/msg/ProcessTelemetry`` on ``/robin/telemetry``;
``config-dds.json`` maps DDS topic ``rt/robin/telemetry`` to NGSI-LD entity
``urn:ngsi-ld:Process:ros_bridge`` with attribute ``urn:robin:processTelemetry``.
For a real process, publish the same message from a node fed by your robot/sensor
topics.

Dashboard and API Actions Back to ROS 2
---------------------------------------

The human input path is:

``dashboard/API action -> pendingIntent in Orion-LD or direct HTTP -> welding_http_bridge -> /intents -> welding_supervisor -> welding_* skill action server``.

``welding_http_bridge`` supports two entry points:

* ``POST /intent`` for direct testing or dashboard fallback.
* Orion-LD subscription notifications when ``pendingIntent`` changes on Process
  entities.

``welding_supervisor`` consumes ``/intents`` and routes the message to the
appropriate skill action server. The ROS4HRI/ROS4RI role, intent, skill, task,
and mission mapping is documented in
``docs/reference/ros4hri_ros4ri_alignment.rst``.

Validation Commands
-------------------

These commands are intended for a Linux Docker environment with the
``vulcanexus`` container running.

Build the workspace:

.. code-block:: console

   $ docker compose up -d vulcanexus
   $ docker exec vulcanexus-bridge bash -lc \
       'cd /workspace/ros2_packages && source ws_setup.sh && colcon build --symlink-install'

List current packages:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 pkg list | grep -E "^(robin_|welding_)" | sort'

Expected package names:

.. code-block:: text

   robin_description
   robin_interfaces
   welding_demo
   welding_home_skill
   welding_http_bridge
   welding_manual_skill
   welding_msgs
   welding_recommendation_skill
   welding_seam_skill
   welding_supervisor

Inspect key interfaces:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 interface show robin_interfaces/msg/ProcessTelemetry'
   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 interface show welding_msgs/msg/Intent'

Run the no-hardware intent path:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 launch welding_demo welding_robin_demo.launch.py'

In a second terminal:

.. code-block:: console

   $ curl -s -X POST http://localhost:8766/intent \
       -H 'Content-Type: application/json' \
       -d '{"intent": "START_PROCESS", "data": {"seam_id": "seam_01"}}'
   {"status": "published", "intent": "START_PROCESS"}

Validate the DDS topic is visible after a telemetry publisher is active:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 topic list | grep /robin/telemetry'

Runtime Assumptions and Limits
------------------------------

* The source-level package and interface inventory can be inspected on any
  platform. Full ROS 2 runtime validation is expected on Linux with Docker and
  the Vulcanexus image.
* The intent/skill path uses simulation behavior by default; no physical
  hardware, MoveIt, or Gazebo is required.
* Driving real hardware requires adopter-provided nodes that advertise the
  skills' hardware-mode interfaces (``/move_home``, ``/execute_bead``,
  ``/fronius/set_*``), launched with ``use_simulation:=false``.
* DDS/FIWARE validation depends on aligned ROS/DDS domain IDs and host
  networking behavior. If Orion-LD does not discover ``/robin/telemetry``,
  check ``ROS_DOMAIN_ID`` in both Compose services and ``config-dds.json``.
