ROS 2 and Vulcanexus Interface Reference
========================================

This page is the central ROS 2/Vulcanexus interface reference for
``open-robin``. It uses the package layout currently present in
``vulcanexus_ws/src`` after the Vulcanexus architecture consolidation. The
package names below are the current source of truth for reviewer-facing
documentation and validation commands.

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
domain. The checked-in DDS mapping file uses domain ``0``. The current Compose
file sets Orion-LD to ``ROS_DOMAIN_ID=0`` and the Vulcanexus container to
``ROS_DOMAIN_ID=52`` for the broader ROS stack, so DDS validation must align
those values before expecting Orion-LD to discover ``/robin/telemetry``. The
FIWARE mapping details are documented in
``docs/reference/fiware_ngsi_ld_dds_mapping.rst``.

Package Inventory
-----------------

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Package
     - Role
   * - ``robin_interfaces``
     - Generic ROBIN ROS 2 messages, services, and actions used by planning,
       telemetry, hardware adapters, and experiment execution.
   * - ``welding_msgs``
     - Intent and skill action interfaces used by the human input and
       dashboard-to-ROS path.
   * - ``robin_core``
     - Consolidated core runtime: MoveIt planner, experiment coordinator,
       calibration helpers, process data processing, weld data alignment,
       progression tracking, TCP/CTWD management, plate markers, and telemetry
       aggregation.
   * - ``robin_bringup``
     - Main system launch, URDF/xacro assets, RViz assets, and default plate
       and bead configuration.
   * - ``robin_moveit_config``
     - MoveIt 2 configuration for UR10e planning and servo operation.
   * - ``robin_hardware_ur``
     - UR10e driver launch wrapper and robot description integration.
   * - ``robin_hardware_opcua``
     - OPC UA bridge for WAGO PLC and Fronius signals.
   * - ``robin_hardware_fronius``
     - Welding coordinator for Fronius/WAGO start, stop, parameter, touch, and
       wire-feed services.
   * - ``robin_hardware_garmo``
     - Lifecycle node for the Garmo Garline profilometer control/data stream
       and ``PointCloud2`` publication.
   * - ``robin_simulation``, ``robin_sim_gz``
     - Simulation support, fake OPC UA bridge, Garmo simulation, and Gazebo
       integration.
   * - ``robin_rqt``
     - RQt operator tools, including the operator panel and process plots.
   * - ``welding_demo``
     - No-hardware launch path for the intent/skill pipeline.
   * - ``welding_http_bridge``
     - HTTP and Orion-LD notification bridge that publishes ROS intents.
   * - ``welding_supervisor``
     - Intent router that dispatches operator intent messages to skill action
       servers.
   * - ``welding_home_skill``, ``welding_seam_skill``,
       ``welding_manual_skill``, ``welding_recommendation_skill``
     - Skill action servers for home, seam execution, manual adjustment, and
       AI recommendation requests.

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
      CORE["robin_core"]
      HW["robin_hardware_*"]
      TEL["/robin/telemetry<br/>robin_interfaces/msg/ProcessTelemetry"]
      MINTAKA["Mintaka / TROE history"]

      UI --> API
      API --> ORION
      ORION --> HTTP
      UI --> HTTP
      HTTP --> INTENT
      INTENT --> SUP
      SUP --> SKILLS
      SKILLS --> CORE
      CORE --> HW
      HW --> CORE
      CORE --> TEL
      TEL --> ORION
      ORION --> MINTAKA
      MINTAKA --> API

Nodes and Executables
---------------------

.. list-table::
   :header-rows: 1
   :widths: 20 23 22 35

   * - Package
     - Executable or node
     - Main interface
     - Purpose
   * - ``robin_bringup``
     - ``lifecycle_manager``
     - lifecycle helper
     - Manages lifecycle nodes used by the bringup flow.
   * - ``robin_core``
     - ``robin_planner_node`` / ``robin_planner``
     - ``execute_bead`` and ``move_home`` actions
     - MoveItPy bead execution, home motion, welding service calls, scanning,
       and calibration support.
   * - ``robin_core``
     - ``experiment_node`` / ``robin_experiment``
     - ``weld_experiment`` action, experiment and plate services
     - Plans, approves, reserves, and executes multi-bead experiments.
   * - ``robin_core``
     - ``weld_data_node``
     - ``robin/data/fronius`` topic
     - Aligns Fronius display values with active bead progression.
   * - ``robin_core``
     - ``progression_node``
     - ``robin/data/progression`` topic
     - Projects TCP position onto the active bead path to compute normalized
       progression.
   * - ``robin_core``
     - ``process_data_node`` / ``weld_profile_processor``
     - ``robin/weld_dimensions`` topic
     - Converts Garmo point clouds into bead geometry measurements.
   * - ``robin_core``
     - ``telemetry_aggregator_node`` / ``telemetry_aggregator``
     - ``/robin/telemetry`` topic
     - Publishes the normalized DDS/FIWARE telemetry snapshot.
   * - ``robin_core``
     - ``tcp_manager_node`` / ``tcp_manager``
     - ``tcp/*`` topics and services
     - Manages active TCP mode and CTWD.
   * - ``robin_core``
     - ``plate_markers_node``
     - ``robin/plates/markers`` topic
     - Publishes RViz plate and bead visualization markers.
   * - ``robin_core``
     - ``view_profile_node``
     - point cloud and geometry subscriptions
     - Live profile viewer/debug tool.
   * - ``robin_hardware_opcua``
     - ``opcua_bridge_node``
     - Fronius and WAGO topics/services
     - C++ OPC UA to ROS 2 bridge.
   * - ``robin_hardware_fronius``
     - ``welding_coordinator``
     - ``welding/*`` services
     - Coordinates Fronius parameters, WAGO start/stop signals, touch probing,
       and wire movement.
   * - ``robin_hardware_garmo``
     - ``sensor_node`` / ``garmo_sensor_node``
     - ``robin/pointcloud`` topic and lifecycle services
     - Controls and reads the Garmo Garline profilometer.
   * - ``robin_hardware_ur``
     - launch-only wrapper
     - UR driver, controller manager, robot state
     - Starts the UR10e driver through ``robot.launch.py`` and ROBIN UR control
       launch files.
   * - ``robin_rqt``
     - RQt plugin classes
     - operator GUI tools
     - Provides the ROBIN Operator Panel and Process Plots plugin.
   * - ``welding_http_bridge``
     - ``welding_http_bridge_node``
     - HTTP ``/intent``, ``/orion-notify``, ``/health``; publishes ROS topic
       ``/intents``
     - Converts dashboard/API/Orion intent events into ROS 2 intent messages.
   * - ``welding_supervisor``
     - ``welding_supervisor_node``
     - ``/intents`` subscription and skill action clients
     - Routes operator intents to the corresponding skill action servers.
   * - ``welding_*_skill``
     - skill-specific node
     - ``welding_*_skill/execute`` actions
     - Implements mock or hardware-backed skills for home, seam execution,
       manual adjustment, and recommendation requests.

Messages
--------

``robin_interfaces/msg``
~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Message
     - Purpose
   * - ``ActiveBead``
     - Current bead ID, path, total length, target speed, wire feed speed,
       recommended current/voltage, and arc length correction.
   * - ``BeadGeometry``
     - Bead ID, progression, height, width, and toe angle from profile
       processing.
   * - ``ExperimentBead``
     - Dashboard-facing bead specification with plate assignment and process
       parameters.
   * - ``ExperimentBeadSpec``
     - System-agnostic bead input for experiment planning.
   * - ``PlateLayout``
     - Pitch, margins, spacing, and stagger configuration for bead placement.
   * - ``ProcessTelemetry``
     - DDS/FIWARE telemetry snapshot: bead ID, progression, height, width,
       speed, current, voltage, and cross-sectional area.
   * - ``WeldPlate``
     - Plate pose, dimensions, corner ID, calibration status, and plane model.
   * - ``WeldProgression``
     - Active bead progression, pass state, and TCP position.
   * - ``WelderData``
     - Fronius current, voltage, wire feed speed, power, energy, bead ID, and
       progression.

``welding_msgs/msg``
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Message
     - Purpose
   * - ``Intent``
     - Operator/system intent message with intent constant, JSON payload,
       source, modality, priority, and confidence. Current constants include
       ``MOVE_TO_HOME``, ``EXECUTE_SEAM``, ``ESTOP``, ``START_PROCESS``,
       ``REQUEST_AI_RECOMMENDATION``, ``MANUAL_ADJUST``, ``LAUNCH_NEW_DOE``,
       ``PAUSE_PROCESS``, ``RESUME_PROCESS``, and ``STOP_PROCESS``.

Services
--------

.. list-table::
   :header-rows: 1
   :widths: 30 24 46

   * - Service name
     - Type
     - Provider / purpose
   * - ``experiment/plan``
     - ``robin_interfaces/srv/PlanExperiment``
     - ``robin_experiment`` computes bead placement from bead specs and plate
       definitions.
   * - ``experiment/approve``
     - ``robin_interfaces/srv/ApproveExperimentPlan``
     - ``robin_experiment`` records operator approval for a planned experiment.
   * - ``experiment/terminate``
     - ``std_srvs/srv/Trigger``
     - ``robin_experiment`` requests cancellation of the active experiment.
   * - ``experiment/continue``
     - ``std_srvs/srv/Trigger``
     - ``robin_experiment`` resumes after an operator/nozzle-cleaning pause.
   * - ``plate/reserve_slots``
     - ``robin_interfaces/srv/ReserveSlots``
     - Reserves physical plate regions so later plans avoid used slots.
   * - ``plate/clear_reserved_slots``
     - ``robin_interfaces/srv/ClearReservedSlots``
     - Removes reservation records.
   * - ``plate/clear_all_beads``
     - ``std_srvs/srv/Trigger``
     - Clears bead records from the current plate configuration store.
   * - ``calibration/find_surface``
     - ``robin_interfaces/srv/FindSurface``
     - Touch-probes the workpiece surface.
   * - ``calibration/calibrate_wire_tip``
     - ``robin_interfaces/srv/CalibrateWireTip``
     - Uses touch sensing to calibrate CTWD/wire tip distance.
   * - ``calibration/calibrate_plate_plane``
     - ``robin_interfaces/srv/CalibratePlatePlane``
     - Probes four points and fits a plate plane.
   * - ``calibration/abort``
     - ``std_srvs/srv/Trigger``
     - Aborts an active calibration routine.
   * - ``tcp/set_mode``
     - ``robin_interfaces/srv/SetTcpMode``
     - Switches between ``welding`` and ``scanning`` TCP modes.
   * - ``tcp/set_ctwd``
     - ``robin_interfaces/srv/SetCtwd``
     - Updates the contact-tip-to-work distance and calibration flag.
   * - ``welding/start``
     - ``robin_interfaces/srv/StartWeld``
     - ``welding_coordinator`` starts welding with wire speed, travel speed,
       and arc correction parameters.
   * - ``welding/stop``
     - ``std_srvs/srv/Trigger``
     - Stops welding and waits for completion of the stop sequence.
   * - ``welding/set_params``
     - ``robin_interfaces/srv/StartWeld``
     - Applies Fronius/WAGO welding parameters without starting the arc.
   * - ``welding/touch_probe``
     - ``std_srvs/srv/Trigger``
     - Enables touch-probing mode.
   * - ``welding/wire_feed_until_touch``
     - ``std_srvs/srv/Trigger``
     - Feeds wire until the touch signal fires.
   * - ``welding/wire_retract``
     - ``robin_interfaces/srv/SetFloat32``
     - Retracts wire by a requested distance.
   * - ``fronius/set_current``, ``fronius/set_voltage``,
       ``fronius/set_wire_speed``, ``fronius/set_arc_length_correction``
     - ``robin_interfaces/srv/SetFloat32``
     - OPC UA writes for Fronius command variables.
   * - ``wago/in/*``
     - ``std_srvs/srv/SetBool``, ``robin_interfaces/srv/SetFloat32``, or
       ``robin_interfaces/srv/SetInt32``
     - OPC UA writes for WAGO/Fronius control signals such as robot ready,
       welding start, touch sensing, wire motion, working mode, and welding
       simulation.

Actions
-------

.. list-table::
   :header-rows: 1
   :widths: 27 25 48

   * - Action name
     - Type
     - Provider / consumer
   * - ``execute_bead``
     - ``robin_interfaces/action/ExecuteBead``
     - Provided by ``robin_planner``. Called by ``robin_experiment`` and the
       seam skill path for bead execution.
   * - ``move_home``
     - ``welding_msgs/action/MoveToHome``
     - Provided by ``robin_planner``. Called by ``welding_home_skill``.
   * - ``weld_experiment``
     - ``robin_interfaces/action/WeldExperiment``
     - Provided by ``robin_experiment`` for full experiment execution.
   * - ``welding_home_skill/execute``
     - ``welding_msgs/action/MoveToHome``
     - Skill action server for home motions.
   * - ``welding_seam_skill/execute``
     - ``welding_msgs/action/ExecuteSeam``
     - Skill action server for seam execution and ``START_PROCESS``.
   * - ``welding_manual_skill/execute``
     - ``welding_msgs/action/ManualAdjust``
     - Skill action server for manual parameter adjustments.
   * - ``welding_recommendation_skill/execute``
     - ``welding_msgs/action/RequestAIRecommendation``
     - Skill action server for advisory recommendation requests.

Core Topics
-----------

.. list-table::
   :header-rows: 1
   :widths: 29 29 42

   * - Topic
     - Type
     - Producer / consumer
   * - ``/intents``
     - ``welding_msgs/msg/Intent``
     - Published by ``welding_http_bridge`` and subscribed by
       ``welding_supervisor``.
   * - ``/doe/launch``
     - ``std_msgs/msg/String``
     - Published by ``welding_supervisor`` for the launch-new-DOE intent.
   * - ``/robin/telemetry``
     - ``robin_interfaces/msg/ProcessTelemetry``
     - Published by ``telemetry_aggregator`` and consumed by Orion-LD DDS.
   * - ``robin/data/active_bead``
     - ``robin_interfaces/msg/ActiveBead``
     - Published by ``robin_planner`` and consumed by progression, data, and
       profile-processing nodes.
   * - ``robin/data/is_welding``, ``robin/data/is_scanning``
     - ``std_msgs/msg/Bool``
     - Latched pass-state topics published by ``robin_planner``.
   * - ``robin/data/progression``
     - ``robin_interfaces/msg/WeldProgression``
     - Published by ``progression_node`` and consumed by weld-data and
       telemetry nodes.
   * - ``robin/data/fronius``
     - ``robin_interfaces/msg/WelderData``
     - Published by ``weld_data_node`` and consumed by telemetry aggregation.
   * - ``robin/weld_dimensions``
     - ``robin_interfaces/msg/BeadGeometry``
     - Published by ``weld_profile_processor`` and consumed by telemetry
       aggregation.
   * - ``robin/pointcloud``
     - ``sensor_msgs/msg/PointCloud2``
     - Published by ``garmo_sensor_node`` or simulation and consumed by profile
       processing and planner diagnostics.
   * - ``robin/plates/markers``
     - ``visualization_msgs/msg/MarkerArray``
     - Published by ``plate_markers`` and ``robin_experiment`` previews.
   * - ``robin/experiment/preview_path``
     - ``nav_msgs/msg/Path``
     - Published by ``robin_experiment`` during planning previews.
   * - ``tcp/active_frame``
     - ``std_msgs/msg/String``
     - Published by ``tcp_manager`` and consumed by planner/progression.
   * - ``tcp/ctwd``
     - ``std_msgs/msg/Float32``
     - Published by ``tcp_manager`` and consumed by planner TCP helpers.
   * - ``tcp/ctwd_calibrated``
     - ``std_msgs/msg/Bool``
     - Published by ``tcp_manager``.
   * - ``fronius/display_current``, ``fronius/display_voltage``,
       ``fronius/display_wfs``, ``fronius/display_power``,
       ``fronius/display_energy``
     - ``std_msgs/msg/Float32``
     - Published by ``opcua_bridge_node`` and consumed by ``weld_data_node``.
   * - ``wago/out/*``
     - ``std_msgs/msg/Bool`` or ``std_msgs/msg/Float32``
     - Published by ``opcua_bridge_node`` for PLC/Fronius feedback.

QoS Assumptions
---------------

* ``/intents`` uses reliable QoS with keep-last depth 10 in
  ``welding_http_bridge``.
* Plate marker and TCP state topics use transient-local/latched semantics where
  late subscribers need the latest state.
* Garmo point clouds and some weld-data streams use sensor-data/best-effort QoS
  because occasional dropped samples are acceptable for high-rate data.
* ``/robin/telemetry`` is published with keep-last depth 10. Orion-LD DDS uses
  the QoS configured in ``config-dds.json`` for the DDS side of the mapping.
* DDS validation requires matching DDS domains between ROS 2 publishers and
  Orion-LD DDS configuration.

Launch Files
------------

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Launch file
     - Purpose
   * - ``robin_bringup/launch/robin_main.launch.py``
     - Main bringup for UR, OPC UA, MoveIt, Garmo, Fronius, weld data, process
       data, RViz, RQt, and optional rosbag recording.
   * - ``robin_core/launch/moveit_planner.launch.py``
     - Starts the planner and MoveIt-related runtime.
   * - ``robin_core/launch/experiment.launch.xml``
     - Starts ``robin_experiment``.
   * - ``robin_core/launch/experiment_record.launch.py``
     - Starts experiment flow with rosbag recording support.
   * - ``robin_core/launch/weld_data.launch.xml``
     - Starts ``weld_data_node``.
   * - ``robin_core/launch/progression.launch.xml``
     - Starts ``progression_node``.
   * - ``robin_core/launch/process_data.launch.xml``
     - Starts ``weld_profile_processor``.
   * - ``robin_core/launch/tcp_manager.launch.xml``
     - Starts ``tcp_manager``.
   * - ``robin_core/launch/telemetry_aggregator.launch.xml``
     - Starts ``telemetry_aggregator``.
   * - ``robin_hardware_ur/launch/robot.launch.py``
     - ROBIN wrapper for UR10e control launch.
   * - ``robin_hardware_ur/launch/robin_ur_control.launch.py``
     - ROBIN UR control launch file.
   * - ``robin_hardware_opcua/launch/opcua_bridge.launch.xml``
     - Starts the OPC UA bridge.
   * - ``robin_hardware_fronius/launch/welding_coordinator.launch.xml``
     - Starts the welding coordinator.
   * - ``robin_hardware_garmo/launch/sensor.launch.xml``
     - Starts the Garmo lifecycle sensor node.
   * - ``robin_simulation/launch/fake_opcua_bridge.launch.xml``
     - Starts fake OPC UA services for local/simulation use.
   * - ``robin_simulation/launch/sim.launch.xml``
     - Starts the simulation path.
   * - ``welding_demo/launch/welding_robin_demo.launch.py``
     - Starts the no-hardware intent and skill demonstration path.
   * - ``welding_home_skill/launch/welding_home_skill.launch.py``
     - Starts the home skill action server.
   * - ``welding_seam_skill/launch/welding_seam_skill.launch.py``
     - Starts the seam skill action server.

ROS 2 to FIWARE Data Path
-------------------------

The reusable telemetry integration path is:

``ROS 2 source topics -> telemetry_aggregator -> /robin/telemetry -> Orion-LD DDS -> NGSI-LD Process entity -> TROE/Mintaka history -> API/dashboard``.

The ROS 2 side normalizes process data into
``robin_interfaces/msg/ProcessTelemetry``. ``config-dds.json`` maps DDS topic
``rt/robin/telemetry`` to NGSI-LD entity
``urn:ngsi-ld:Process:ros_bridge`` with attribute
``urn:robin:processTelemetry``. The Process Intelligence API and ROBIN
Dashboard can then read current and historical telemetry through Orion-LD and
Mintaka.

Dashboard and API Actions Back to ROS 2
---------------------------------------

The current human input path is:

``dashboard/API action -> pendingIntent in Orion-LD or direct HTTP -> welding_http_bridge -> /intents -> welding_supervisor -> welding_* skill action server -> robin_core/hardware``.

``welding_http_bridge`` supports two entry points:

* ``POST /intent`` for direct testing or dashboard fallback.
* ``POST /orion-notify`` for Orion-LD subscription notifications when
  ``pendingIntent`` changes on Process entities.

``welding_supervisor`` consumes ``/intents`` and routes the message to the
appropriate skill action server. The ROS4HRI/ROS4RI role, intent, skill, task,
and mission mapping is documented in
``docs/reference/ros4hri_ros4ri_alignment.rst``.

Validation Commands
-------------------

These commands are intended for a Linux Docker environment with the
``vulcanexus`` container running. They inspect the current package and interface
surface without requiring the industrial hardware.

Build the workspace:

.. code-block:: console

   $ docker compose up -d vulcanexus
   $ docker exec vulcanexus-bridge bash -lc \
       'cd /workspace/ros2_packages && source ws_setup.sh && colcon build --symlink-install'

Expected output includes a successful ``colcon`` summary for the current
``robin_*`` and ``welding_*`` packages.

List current packages:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 pkg list | grep -E "^(robin_|welding_)" | sort'

Expected package names:

.. code-block:: text

   robin_bringup
   robin_core
   robin_hardware_fronius
   robin_hardware_garmo
   robin_hardware_opcua
   robin_hardware_ur
   robin_interfaces
   robin_moveit_config
   robin_rqt
   robin_sim_gz
   robin_simulation
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

Expected output includes ``height``, ``width``, ``speed``, ``current``, and
``voltage`` for ``ProcessTelemetry`` and the current operator intent constants
for ``Intent``.

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

Then inspect ``/intents`` from the container:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 topic echo /intents'

Expected output includes an ``Intent`` message with
``intent: START_PROCESS`` and JSON data for ``seam_id``.

Validate the DDS topic is visible after a telemetry publisher is active:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       'source /workspace/ros2_packages/ws_setup.sh && ros2 topic list | grep /robin/telemetry'

Expected output:

.. code-block:: text

   /robin/telemetry

Runtime Assumptions and Limits
------------------------------

* The source-level package and interface inventory can be inspected on any
  platform. Full ROS 2 runtime validation is expected on Linux with Docker and
  the Vulcanexus image.
* The no-hardware intent/skill path uses simulation behavior by default.
* Physical operation depends on the UR10e, Fronius TPS320i, WAGO OPC UA, Garmo
  Garline, and cell network configuration.
* DDS/FIWARE validation depends on aligned ROS/DDS domain IDs and host
  networking behavior. If Orion-LD does not discover ``/robin/telemetry``,
  check ``ROS_DOMAIN_ID`` in both Compose services and ``config-dds.json``.
* Some hardware nodes expose welding-specific services and topics. The reusable
  integration pattern is the normalized telemetry message, DDS mapping, intent
  bridge, supervisor pattern, and dashboard/API control loop.
