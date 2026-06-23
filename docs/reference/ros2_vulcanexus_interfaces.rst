ROS 2 and Vulcanexus Interface Reference
========================================

This page is the central reference for the ROS 2/Vulcanexus interfaces in
``open-robin``. It documents the current repository names, generated interfaces,
runtime nodes, topics, services, actions, launch files, QoS assumptions, and the
validated ROS 2 to FIWARE path so reviewers do not need to read source code to
inspect the interface surface.

Scope and Source of Truth
-------------------------

The ROS 2 workspace lives in ``vulcanexus_ws/src`` and is mounted into the
Compose container as ``/workspace/ros2_packages/src``. The current top-level
repository names used by this reference are:

* ``open-robin`` for this repository.
* ``robin/`` for the Process Intelligence API module.
* ``robin-dashboard/`` for the React dashboard module.
* ``vulcanexus_ws/`` for the ROS 2/Vulcanexus workspace and integration
  examples.

This page uses the package names currently present in ``vulcanexus_ws/src``:
``robin_*`` and ``welding_*``. It does not use older D3 package names.

Version and Runtime Baseline
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 28 32 40

   * - Item
     - Current value
     - Evidence
   * - Vulcanexus base image
     - ``eprosima/vulcanexus:jazzy-desktop-4.3.1``
     - ``vulcanexus_ws/Dockerfile``
   * - ROS 2 distribution
     - Jazzy
     - ``/opt/ros/jazzy/setup.bash`` is sourced by ``ws_setup.sh``.
   * - Vulcanexus distribution
     - Jazzy
     - ``/opt/vulcanexus/jazzy/setup.bash`` is sourced by ``ws_setup.sh``.
   * - Compose ROS container
     - ``vulcanexus`` service, container ``vulcanexus-bridge``
     - ``docker-compose.yaml``
   * - ROS domain
     - ``ROS_DOMAIN_ID=0`` for Orion-LD DDS discovery
     - ``docker-compose.yaml``
   * - DDS/NGSI-LD bridge
     - Orion-LD ``-wip dds`` with ``config-dds.json``
     - ``docker-compose.yaml`` and ``config-dds.json``

Workspace Packages
------------------

The ROS workspace build validated these packages:

.. code-block:: text

   robin_calibration
   robin_core_bringup
   robin_core_data
   robin_core_planner
   robin_core_sensor
   robin_experiment
   robin_hardware_fronius
   robin_hardware_garmo
   robin_hardware_opcua
   robin_hardware_ur
   robin_interfaces
   robin_moveit_config
   robin_rqt
   robin_simulation
   welding_demo
   welding_home_skill
   welding_http_bridge
   welding_manual_skill
   welding_msgs
   welding_recommendation_skill
   welding_seam_skill
   welding_supervisor

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Package group
     - Role
   * - ``robin_interfaces``
     - ROBIN ROS 2 messages, services, and actions.
   * - ``welding_msgs``
     - Intent and dashboard/HRI skill action interfaces.
   * - ``robin_core_data``, ``robin_core_sensor``
     - Sensor processing, weld data alignment, plate marker publication, and
       telemetry aggregation for DDS/FIWARE.
   * - ``robin_core_planner``, ``robin_experiment``, ``robin_calibration``
     - Motion planning, bead execution, experiment planning, and touch-sensing
       calibration services.
   * - ``robin_core_bringup``, ``robin_moveit_config``
     - System launch and MoveIt configuration.
   * - ``robin_hardware_ur``, ``robin_hardware_garmo``,
       ``robin_hardware_fronius``, ``robin_hardware_opcua``
     - UR robot, Garmo profilometer, Fronius/WAGO welding, and OPC UA hardware
       integration.
   * - ``robin_rqt``, ``robin_simulation``
     - Operator panel and simulation/visualization support.
   * - ``welding_demo``, ``welding_home_skill``, ``welding_seam_skill``,
       ``welding_recommendation_skill``, ``welding_manual_skill``,
       ``welding_http_bridge``, ``welding_supervisor``
     - Dashboard/HRI intent pipeline and mock skill servers.

Generated Interfaces
--------------------

``robin_interfaces``
~~~~~~~~~~~~~~~~~~~~

``robin_interfaces/CMakeLists.txt`` generates 11 messages, 11 services, and 2
actions.

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Message
     - Purpose
   * - ``msg/ActiveBead``
     - Active bead ID, start/end points, bead length, and target process
       parameters used by weld data progression tracking.
   * - ``msg/BeadGeometry``
     - Bead height, width, and cross-sectional area from processed profilometer
       point clouds.
   * - ``msg/ExperimentBead``
     - Dashboard-facing planned bead with plate ID, travel speed, synergetic
       primary parameter, and optional voltage/current/wire-feed overrides.
   * - ``msg/ExperimentBeadSpec``
     - System-agnostic bead input for experiment planning: target current,
       weld speed, stickout, and optional scan speed.
   * - ``msg/FroniusSample``
     - Fronius measurement sample with bead ID, progression, current, voltage,
       wire-feed speed, and power.
   * - ``msg/PlateLayout``
     - Plate layout parameters for bead placement: pitch, margins, spacing, and
       stagger flag.
   * - ``msg/ProcessTelemetry``
     - Normalized DDS/FIWARE telemetry on ``/robin/telemetry`` with
       ``current``, ``voltage``, ``speed``, ``width``, ``height``, and
       ``cross_sectional_area``.
   * - ``msg/WeldBead``
     - Bead ID, start/end points, target speed, voltage, current, wire-feed
       speed, and stickout.
   * - ``msg/WeldPlate``
     - Plate ID, origin, size, orientation, surface calibration state, probing
       margins, plane coefficients, and probe points.
   * - ``msg/WeldProgression``
     - Active bead ID, normalized progression, welding state, and TCP position.
   * - ``msg/WelderData``
     - Simulated Fronius welder data with voltage, current, and wire-feed
       speed.

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Service
     - Contract summary
   * - ``srv/ApproveExperimentPlan``
     - Request ``plan_id`` and ``approve``; returns ``success``, ``approved``,
       and ``message``.
   * - ``srv/CalibratePlatePlane``
     - Request plate corner, size, margins, yaw, and probe settings; returns
       surface Z, plane coefficients, and probe points.
   * - ``srv/CalibrateStickout``
     - Request XY location, surface Z, and retract length; returns measured
       stickout and status.
   * - ``srv/FindSurface``
     - Request XY, starting Z, lower Z limit, and probe speed; returns detected
       surface Z and status.
   * - ``srv/PlanExperiment``
     - Request bead specs, calibrated plates, spacing, margins, stagger flag,
       and preview flag; returns plan ID, planned beads, plates, and layout.
   * - ``srv/SensorCommand``
     - Request sensor IP, ports, command, and FPS; returns success and message.
   * - ``srv/SetFloat32``
     - Request one ``float32``; returns success and message.
   * - ``srv/SetInt32``
     - Request one ``int32``; returns success and message.
   * - ``srv/SetTcpMode``
     - Request ``mode`` (``welding`` or ``scanning``); returns active TCP frame
       and status.
   * - ``srv/SetUInt8``
     - Request one ``uint8``; returns success and message.
   * - ``srv/StartWeld``
     - Request primary parameter selection plus current, voltage, and wire
       speed; returns success and message.

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Action
     - Goal/result/feedback summary
   * - ``action/ExecuteBead``
     - Goal identifies bead, plate, start/end points, process parameters,
       requested stickout, scan speed, and dry-run flag. Result reports success,
       message, calibrated stickout, and applied stickout. Feedback reports
       step and progress.
   * - ``action/WeldExperiment``
     - Goal contains ``WeldBead[]`` and dry-run flag. Result reports success
       and completion percentage. Feedback reports status and progress.

``welding_msgs``
~~~~~~~~~~~~~~~~

``welding_msgs/CMakeLists.txt`` generates 1 message and 4 actions.

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Interface
     - Purpose
   * - ``msg/Intent``
     - Dashboard/HRI intent message. Constants include ``MOVE_TO_HOME``,
       ``EXECUTE_SEAM``, ``ESTOP``, ``START_PROCESS``,
       ``REQUEST_AI_RECOMMENDATION``, ``MANUAL_ADJUST``, ``LAUNCH_NEW_DOE``,
       ``PAUSE_PROCESS``, ``RESUME_PROCESS``, and ``STOP_PROCESS``. Source
       constants are ``SOURCE_ROBOT``, ``SOURCE_REMOTE``, and
       ``SOURCE_UNKNOWN``. Modality constants are ``MODALITY_TOUCHSCREEN``,
       ``MODALITY_SPEECH``, ``MODALITY_GESTURE``, and
       ``MODALITY_INTERNAL``. Fields are ``intent``, JSON string ``data``,
       ``source``, ``modality``, ``priority``, and ``confidence``.
   * - ``action/ExecuteSeam``
     - Goal ``seam_id``, ``weld_speed`` in mm/s, and ``wire_feed_rate`` in
       m/min. Result success, message, and seam length. Feedback progress,
       current speed, and phase.
   * - ``action/ManualAdjust``
     - Goal parameter name, new value, and unit. Result success, message, and
       applied value. Feedback progress and phase.
   * - ``action/MoveToHome``
     - Goal ``use_fast_speed``. Result success and message. Feedback progress
       and current joint.
   * - ``action/RequestAIRecommendation``
     - Goal ``process_id`` and mode. Result success, message, and JSON
       recommendation. Feedback progress and phase.

Runtime Nodes
-------------

.. list-table::
   :header-rows: 1
   :widths: 24 24 52

   * - Node
     - Package/executable
     - Main interfaces
   * - ``welding_http_bridge``
     - ``welding_http_bridge`` / ``welding_http_bridge_node``
     - HTTP ``GET /health``, ``POST /intent``, and ``POST /orion-notify`` on
       port ``8766``. Publishes ``welding_msgs/msg/Intent`` on ``/intents``.
       Registers an Orion-LD subscription for ``pendingIntent`` changes.
   * - ``welding_supervisor``
     - ``welding_supervisor`` / ``welding_supervisor_node``
     - Subscribes ``/intents``. Publishes ``/doe/launch``. Routes intents to
       ``welding_home_skill/execute``, ``welding_seam_skill/execute``,
       ``welding_recommendation_skill/execute``, and
       ``welding_manual_skill/execute`` action servers.
   * - ``welding_home_skill``
     - ``welding_home_skill`` / ``welding_home_skill_node``
     - Provides ``welding_home_skill/execute``. In simulation mode publishes
       ``/joint_states`` and subscribes ``/joint_states_manual``.
   * - ``welding_seam_skill``
     - ``welding_seam_skill`` / ``welding_seam_skill_node``
     - Provides ``welding_seam_skill/execute``. In hardware mode sends
       ``robin_interfaces/action/WeldExperiment`` goals to ``weld_experiment``.
   * - ``welding_recommendation_skill``
     - ``welding_recommendation_skill`` /
       ``welding_recommendation_skill_node``
     - Provides ``welding_recommendation_skill/execute`` mock AI
       recommendation action.
   * - ``welding_manual_skill``
     - ``welding_manual_skill`` / ``welding_manual_skill_node``
     - Provides ``welding_manual_skill/execute``. In hardware mode maps
       ``current``, ``voltage``, and ``wire_speed`` to Fronius parameter
       services.
   * - ``telemetry_aggregator``
     - ``robin_core_data`` / ``telemetry_aggregator_node.py``
     - Subscribes ``/robin/weld_dimensions`` and ``/robin/data/fronius`` as raw
       CDR-compatible data. Publishes ``/robin/telemetry``.
   * - ``weld_data_node``
     - ``robin_core_data`` / ``weld_data_node.py``
     - Subscribes Fronius display topics plus ``/robin/data/active_bead`` and
       ``/robin/data/is_welding``. Publishes ``/robin/data/progression`` and
       ``/robin/data/fronius``.
   * - ``weld_profile_processor``
     - ``robin_core_sensor`` / ``process_data``
     - Subscribes ``/robin/pointcloud`` and publishes ``/robin/weld_dimensions``.
   * - ``robin_moveit_planner``
     - ``robin_core_planner`` / ``robin_planner.py``
     - Provides ``/execute_bead`` action. Uses welding, TCP, calibration, and
       profilometer services.
   * - ``robin_experiment``
     - ``robin_experiment`` / ``experiment_node.py``
     - Provides ``weld_experiment`` action and ``/experiment/*`` services.
       Publishes experiment preview markers and path.
   * - ``tcp_manager``
     - ``robin_hardware_fronius`` / ``tcp_manager``
     - Publishes TCP state and dynamic TF. Provides ``/tcp/*`` services.
   * - ``welding_coordinator``
     - ``robin_hardware_fronius`` / ``welding_coordinator``
     - Coordinates WAGO/Fronius control. Provides ``/welding/*`` services and
       uses WAGO/Fronius OPC UA bridge services/topics.
   * - ``opcua_bridge``
     - ``robin_hardware_opcua`` / ``opcua_bridge_node``
     - Uses ``opcua_bridge.yaml`` to publish OPC UA values as ROS topics and
       expose OPC UA writes as ROS services.
   * - ``garmo_command_node``
     - ``robin_hardware_garmo`` / ``sensor_cmd``
     - Exposes profilometer activation/deactivation services.
   * - ``robin_sensor_publisher``
     - ``robin_hardware_garmo`` / ``sensor_data``
     - Publishes Garmo point cloud data on ``/robin/pointcloud``.

Topics
------

.. list-table::
   :header-rows: 1
   :widths: 26 25 24 25

   * - Topic
     - Type
     - Producer
     - Consumer
   * - ``/intents``
     - ``welding_msgs/msg/Intent``
     - ``welding_http_bridge`` or direct ROS publishers
     - ``welding_supervisor``
   * - ``/doe/launch``
     - ``std_msgs/msg/String``
     - ``welding_supervisor``
     - DOE/operator-panel launch observers
   * - ``/robin/telemetry``
     - ``robin_interfaces/msg/ProcessTelemetry``
     - ``telemetry_aggregator``
     - Orion-LD DDS bridge via DDS topic ``rt/robin/telemetry``
   * - ``/robin/weld_dimensions``
     - ``robin_interfaces/msg/BeadGeometry``
     - ``weld_profile_processor``
     - ``telemetry_aggregator``
   * - ``/robin/data/fronius``
     - ``robin_interfaces/msg/FroniusSample``
     - ``weld_data_node``
     - ``telemetry_aggregator``
   * - ``/robin/data/progression``
     - ``robin_interfaces/msg/WeldProgression``
     - ``weld_data_node``
     - Data consumers and visualization
   * - ``/robin/data/active_bead``
     - ``robin_interfaces/msg/ActiveBead``
     - Planner/welding client
     - ``weld_data_node``
   * - ``/robin/data/is_welding``
     - ``std_msgs/msg/Bool``
     - Planner/welding client
     - ``weld_data_node``
   * - ``/robin/pointcloud``
     - ``sensor_msgs/msg/PointCloud2``
     - ``robin_sensor_publisher``
     - ``weld_profile_processor``
   * - ``/fronius/display_current``,
       ``/fronius/display_voltage``,
       ``/fronius/display_wfs``,
       ``/fronius/display_power``
     - ``std_msgs/msg/Float32``
     - ``opcua_bridge``
     - ``weld_data_node`` and operator UI
   * - ``/wago/out/*``
     - ``std_msgs/msg/Bool`` or ``std_msgs/msg/Float32``
     - ``opcua_bridge``
     - ``welding_coordinator`` and operator UI
   * - ``/wago/in/*/state``
     - ``std_msgs/msg/Bool``
     - ``opcua_bridge``
     - Operator UI
   * - ``/tcp/active_frame``
     - ``std_msgs/msg/String``
     - ``tcp_manager``
     - Planner and operator UI
   * - ``/tcp/stickout``
     - ``std_msgs/msg/Float32``
     - ``tcp_manager``
     - Planner and operator UI
   * - ``/tcp/stickout_calibrated``
     - ``std_msgs/msg/Bool``
     - ``tcp_manager``
     - Operator UI
   * - ``/joint_states_manual``
     - ``sensor_msgs/msg/JointState``
     - ``joint_state_publisher_gui``
     - ``welding_home_skill``
   * - ``/joint_states``
     - ``sensor_msgs/msg/JointState``
     - ``welding_home_skill`` in simulation
     - Robot state publisher and visualization
   * - ``/robin/plates/markers``
     - ``visualization_msgs/msg/MarkerArray``
     - ``plate_markers`` or ``robin_experiment``
     - RViz/operator visualization
   * - ``/robin/experiment/preview_path``
     - ``nav_msgs/msg/Path``
     - ``robin_experiment``
     - RViz/operator visualization

Services
--------

.. list-table::
   :header-rows: 1
   :widths: 26 25 49

   * - Service
     - Type
     - Provider and purpose
   * - ``/experiment/plan``
     - ``robin_interfaces/srv/PlanExperiment``
     - ``robin_experiment`` plans bead layout from specs and plates.
   * - ``/experiment/approve``
     - ``robin_interfaces/srv/ApproveExperimentPlan``
     - ``robin_experiment`` operator approval gate.
   * - ``/experiment/terminate``
     - ``std_srvs/srv/Trigger``
     - ``robin_experiment`` cancels active experiment execution.
   * - ``/calibration/find_surface``
     - ``robin_interfaces/srv/FindSurface``
     - Hosted by ``robin_moveit_planner`` via ``robin_calibration``.
   * - ``/calibration/calibrate_stickout``
     - ``robin_interfaces/srv/CalibrateStickout``
     - Hosted by ``robin_moveit_planner`` via ``robin_calibration``.
   * - ``/calibration/calibrate_plate_plane``
     - ``robin_interfaces/srv/CalibratePlatePlane``
     - Hosted by ``robin_moveit_planner`` via ``robin_calibration``.
   * - ``/calibration/abort``
     - ``std_srvs/srv/Trigger``
     - Hosted by ``robin_moveit_planner`` via ``robin_calibration``.
   * - ``/tcp/set_mode``
     - ``robin_interfaces/srv/SetTcpMode``
     - ``tcp_manager`` switches active TCP mode.
   * - ``/tcp/set_stickout``
     - ``robin_interfaces/srv/SetFloat32``
     - ``tcp_manager`` updates wire stickout.
   * - ``/tcp/mark_stickout_calibrated``
     - ``std_srvs/srv/Trigger``
     - ``tcp_manager`` records calibrated stickout state.
   * - ``/welding/start``
     - ``robin_interfaces/srv/StartWeld``
     - ``welding_coordinator`` starts welding with process parameters.
   * - ``/welding/stop``
     - ``std_srvs/srv/Trigger``
     - ``welding_coordinator`` stops welding.
   * - ``/welding/set_params``
     - ``robin_interfaces/srv/StartWeld``
     - ``welding_coordinator`` updates welding parameters.
   * - ``/welding/touch_probe``
     - ``std_srvs/srv/Trigger``
     - ``welding_coordinator`` runs a touch probe sequence.
   * - ``/welding/wire_feed_until_touch``
     - ``std_srvs/srv/Trigger``
     - ``welding_coordinator`` feeds wire until touch signal.
   * - ``/welding/wire_retract``
     - ``robin_interfaces/srv/SetFloat32``
     - ``welding_coordinator`` retracts wire by a requested length.
   * - ``/fronius/set_current``,
       ``/fronius/set_voltage``,
       ``/fronius/set_wire_speed``
     - ``robin_interfaces/srv/SetFloat32``
     - ``opcua_bridge`` writes Fronius OPC UA recommendation values.
   * - ``/wago/in/*``
     - ``std_srvs/srv/SetBool``, ``robin_interfaces/srv/SetFloat32``, or
       ``robin_interfaces/srv/SetInt32``
     - ``opcua_bridge`` writes WAGO PLC control values from
       ``opcua_bridge.yaml``.
   * - ``/profilometer_activate``,
       ``/profilometer_deactivate``
     - Intended callers use ``robin_interfaces/srv/SensorCommand``.
     - ``robin_hardware_garmo/sensor_cmd.py`` currently advertises these
       services as ``std_srvs/srv/Trigger`` while planner, RQT, and launch
       callers expect ``SensorCommand``. Treat this as a current implementation
       mismatch until the service provider is aligned.

Actions
-------

.. list-table::
   :header-rows: 1
   :widths: 28 24 48

   * - Action name
     - Type
     - Provider/client relationship
   * - ``welding_home_skill/execute``
     - ``welding_msgs/action/MoveToHome``
     - Provided by ``welding_home_skill``. Called by ``welding_supervisor``.
   * - ``welding_seam_skill/execute``
     - ``welding_msgs/action/ExecuteSeam``
     - Provided by ``welding_seam_skill``. Called by ``welding_supervisor`` for
       ``EXECUTE_SEAM`` and ``START_PROCESS`` intents.
   * - ``welding_recommendation_skill/execute``
     - ``welding_msgs/action/RequestAIRecommendation``
     - Provided by ``welding_recommendation_skill``. Called by
       ``welding_supervisor`` for ``REQUEST_AI_RECOMMENDATION``.
   * - ``welding_manual_skill/execute``
     - ``welding_msgs/action/ManualAdjust``
     - Provided by ``welding_manual_skill``. Called by ``welding_supervisor``.
   * - ``weld_experiment``
     - ``robin_interfaces/action/WeldExperiment``
     - Provided by ``robin_experiment``. Called by ``welding_seam_skill`` in
       hardware mode and by the RQT experiment workflow.
   * - ``/execute_bead``
     - ``robin_interfaces/action/ExecuteBead``
     - Provided by ``robin_moveit_planner``. Called by ``robin_experiment`` for
       per-bead execution.

Launch Files
------------

.. list-table::
   :header-rows: 1
   :widths: 36 64

   * - Launch file
     - Main nodes or includes
   * - ``welding_demo/launch/welding_robin_demo.launch.py``
     - Starts ``welding_home_skill``, ``welding_seam_skill``,
       ``welding_recommendation_skill``, ``welding_manual_skill``,
       ``welding_http_bridge``, and delayed ``welding_supervisor``.
   * - ``welding_home_skill/launch/welding_home_skill.launch.py``
     - Starts ``welding_home_skill``.
   * - ``welding_seam_skill/launch/welding_seam_skill.launch.py``
     - Starts ``welding_seam_skill``.
   * - ``robin_simulation/launch/sim_visualization.launch.py``
     - Starts robot state publisher, joint state GUI, simulation/visualization
       nodes, RViz, Foxglove bridge, ``welding_http_bridge``,
       ``welding_home_skill``, and delayed ``welding_supervisor``.
   * - ``robin_core_bringup/launch/robin_main.launch.py``
     - Full robot bringup: UR launch, MoveIt planner, experiment node, plate
       markers, RViz, optional RQT panel, Garmo sensor, sensor processing, OPC
       UA bridge, welding coordinator, TCP manager, and weld data node.
   * - ``robin_core_planner/launch/moveit_planner.launch.py``
     - Starts ``robin_moveit_planner`` and optional MoveIt Servo node.
   * - ``robin_experiment/launch/experiment.launch.py``
     - Starts ``robin_experiment`` with default planning parameters.
   * - ``robin_core_data/launch/weld_data.launch.py``
     - Starts ``weld_data_node`` with ``weld_data.yaml``.
   * - ``robin_core_sensor/launch/process_data.launch.py``
     - Starts ``weld_profile_processor``.
   * - ``robin_hardware_fronius/launch/tcp_manager.launch.py``
     - Starts ``tcp_manager``.
   * - ``robin_hardware_fronius/launch/welding_coordinator.launch.py``
     - Starts ``welding_coordinator``.
   * - ``robin_hardware_garmo/launch/sensor.launch.py``
     - Starts ``garmo_command_node`` and ``robin_sensor_publisher`` and attempts
       a delayed ``/profilometer_activate`` service call.
   * - ``robin_hardware_opcua/launch/opcua_bridge.launch.py``
     - Starts ``opcua_bridge`` with ``opcua_bridge.yaml``.
   * - ``robin_hardware_ur/launch/robot.launch.py``
     - Includes UR driver launch for simulation or real robot modes.
   * - ``robin_moveit_config/launch/ur_moveit.launch.py``
     - Starts MoveIt ``move_group``, RViz, and optional Servo node.
   * - ``robin_rqt/launch/operator_panel.launch.py``
     - Starts the RQT operator panel plugin.

QoS Assumptions
---------------

* The Orion-LD DDS configuration allows only ``rt/robin/telemetry`` and sets
  DDS topic QoS to ``VOLATILE`` durability, ``BEST_EFFORT`` reliability, and
  history depth ``5000``.
* ``welding_http_bridge`` publishes ``/intents`` with explicit ROS QoS:
  reliable, keep-last depth ``10``.
* Most Python ROS publishers/subscribers pass an integer queue depth of ``10``.
  In ROS 2 this means keep-last depth 10 with default reliable/volatile QoS.
* ``robin_hardware_garmo/sensor_data.py`` publishes point clouds with
  ``qos_profile_sensor_data``.
* The telemetry aggregator uses raw subscriptions for geometry and Fronius data
  to tolerate older bag CDR layouts.

Runtime Assumptions
-------------------

* Full DDS confidence requires Linux or a validated Docker host. macOS with
  Docker Desktop is acceptable for source inspection and documentation editing,
  but DDS discovery and full ROS 2 launch behavior should not be treated as
  fully validated from macOS alone.
* The Compose stack uses host networking and ``ipc: host`` for Orion-LD and
  ``vulcanexus`` DDS discovery.
* Orion-LD, MongoDB, TimescaleDB, Mintaka, the Process Intelligence API, the
  dashboard, and ``vulcanexus`` are expected to run from ``docker-compose.yaml``.
* Local Compose disables NGSI-LD tenant headers for the DDS path; use a
  consistent tenant configuration if enabling tenants in a deployment.
* Hardware-mode defaults assume reachable devices on the configured lab
  addresses: UR robot ``192.168.1.101``, Garmo sensor ``192.168.1.212``,
  Fronius OPC UA ``192.168.1.104:4840``, and WAGO OPC UA
  ``192.168.0.17:4840``.
* The ``welding_*`` skill nodes default to simulation mode unless
  ``use_simulation`` is set false.
* The current launch files do not start ``telemetry_aggregator_node.py`` as part
  of ``welding_robin_demo.launch.py``. Start it directly with
  ``ros2 run robin_core_data telemetry_aggregator_node.py`` when validating the
  DDS/FIWARE telemetry path.

ROS 2 to FIWARE Path
--------------------

The normalized telemetry path is:

.. code-block:: text

   /robin/weld_dimensions      robin_interfaces/msg/BeadGeometry
   /robin/data/fronius         robin_interfaces/msg/FroniusSample or compatible raw CDR
          |
          v
   telemetry_aggregator_node.py
          |
          v
   ROS 2 topic: /robin/telemetry
   DDS topic:   rt/robin/telemetry
          |
          v
   Orion-LD DDS bridge
          |
          v
   NGSI-LD entity:    urn:ngsi-ld:Process:ros_bridge
   NGSI-LD attribute: urn:robin:processTelemetry

``config-dds.json`` is the DDS/NGSI-LD mapping source of truth:

.. code-block:: json

   {
     "rt/robin/telemetry": {
       "entityType": "urn:robin:Process",
       "entityId": "urn:ngsi-ld:Process:ros_bridge",
       "attribute": "urn:robin:processTelemetry"
     }
   }

Dashboard/API to ROS 2 Intent Path
----------------------------------

The dashboard/API intent path is:

.. code-block:: text

   ROBIN dashboard or API action
          |
          | direct test path: POST http://localhost:8766/intent
          | Orion path: Process.pendingIntent -> Orion-LD subscription
          v
   welding_http_bridge
          |
          v
   /intents                         welding_msgs/msg/Intent
          |
          v
   welding_supervisor
          |
          +--> welding_home_skill/execute
          +--> welding_seam_skill/execute
          +--> welding_recommendation_skill/execute
          +--> welding_manual_skill/execute
          +--> /doe/launch

``docs/LAUNCH_AND_VERIFY_INTENTS.md`` documents the operator-facing validation
flow. ``welding_http_bridge_node.py`` defines the HTTP and Orion notification
handlers. ``welding_supervisor_node.py`` defines the routing from intent
constants to action clients.

Runtime Validation Record
-------------------------

Validation was performed on 2026-06-23 on a Linux Docker host in simulation
mode. No UR, Garmo, Fronius, or WAGO hardware was connected or validated. Long
command outputs below are trimmed to the rows that confirm interface, graph, and
runtime behavior.

Commands run:

.. code-block:: console

   $ docker compose up -d orion-ld mongo-db timescaledb mintaka alert-processor robin-dashboard vulcanexus
   Container db-mongo  Started
   Container fiware-timescaledb  Started
   Container fiware-orion  Started
   Container vulcanexus-bridge  Started
   Container fiware-mintaka  Started
   Container robin-alert-processor  Started
   Container robin-dashboard  Started

.. code-block:: console

   $ docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
   NAMES                   STATUS
   robin-alert-processor   Up
   fiware-orion            Up
   vulcanexus-bridge       Up
   fiware-timescaledb      Up (healthy)
   robin-dashboard         Up
   fiware-mintaka          Up
   db-mongo                Up

The first launch attempt used an existing stale ``vulcanexus`` image and exposed
an environment issue: ``welding_http_bridge`` failed with
``ModuleNotFoundError: No module named 'aiohttp'``. The source requirement was
already present in ``vulcanexus_ws/requirements.txt`` as ``aiohttp>=3.9``. The
image was rebuilt and the container was recreated before final validation:

.. code-block:: console

   $ docker compose build vulcanexus
   Collecting aiohttp>=3.9 (from -r /tmp/requirements.txt (line 3))
   => => naming to docker.io/library/open-robin-vulcanexus
   [SUCCESS] vulcanexus Built

.. code-block:: console

   $ docker compose up -d vulcanexus
   Container vulcanexus-bridge  Recreated
   Container vulcanexus-bridge  Started

.. code-block:: console

   $ python -c 'import aiohttp; print(aiohttp.__version__)'
   3.14.1

ROS workspace build after recreating the container:

.. code-block:: console

   $ docker exec vulcanexus-bridge bash -lc \
       "cd /workspace/ros2_packages && source /opt/ros/jazzy/setup.bash && \
        source /opt/vulcanexus/jazzy/setup.bash && colcon build --symlink-install"
   Summary: 22 packages finished [22.7s]

Package inventory:

.. code-block:: console

   $ ros2 pkg list | grep -E '^(robin_|welding_)'
   robin_calibration
   robin_core_bringup
   robin_core_data
   robin_core_planner
   robin_core_sensor
   robin_experiment
   robin_hardware_fronius
   robin_hardware_garmo
   robin_hardware_opcua
   robin_hardware_ur
   robin_interfaces
   robin_moveit_config
   robin_rqt
   robin_simulation
   welding_demo
   welding_home_skill
   welding_http_bridge
   welding_manual_skill
   welding_msgs
   welding_recommendation_skill
   welding_seam_skill
   welding_supervisor

Interface inventory:

.. code-block:: console

   $ ros2 interface list | grep -E '^[[:space:]]*(robin_interfaces|welding_msgs)/'
       robin_interfaces/msg/ActiveBead
       robin_interfaces/msg/BeadGeometry
       robin_interfaces/msg/ExperimentBead
       robin_interfaces/msg/ExperimentBeadSpec
       robin_interfaces/msg/FroniusSample
       robin_interfaces/msg/PlateLayout
       robin_interfaces/msg/ProcessTelemetry
       robin_interfaces/msg/WeldBead
       robin_interfaces/msg/WeldPlate
       robin_interfaces/msg/WeldProgression
       robin_interfaces/msg/WelderData
       welding_msgs/msg/Intent
       robin_interfaces/srv/ApproveExperimentPlan
       robin_interfaces/srv/CalibratePlatePlane
       robin_interfaces/srv/CalibrateStickout
       robin_interfaces/srv/FindSurface
       robin_interfaces/srv/PlanExperiment
       robin_interfaces/srv/SensorCommand
       robin_interfaces/srv/SetFloat32
       robin_interfaces/srv/SetInt32
       robin_interfaces/srv/SetTcpMode
       robin_interfaces/srv/SetUInt8
       robin_interfaces/srv/StartWeld
       robin_interfaces/action/ExecuteBead
       robin_interfaces/action/WeldExperiment
       welding_msgs/action/ExecuteSeam
       welding_msgs/action/ManualAdjust
       welding_msgs/action/MoveToHome
       welding_msgs/action/RequestAIRecommendation

Interface inspection:

.. code-block:: console

   $ ros2 interface show robin_interfaces/msg/ProcessTelemetry
   std_msgs/Header header
   float32 current
   float32 voltage
   float32 speed
   float32 width
   float32 height
   float32 cross_sectional_area

.. code-block:: console

   $ ros2 interface show welding_msgs/msg/Intent
   string MOVE_TO_HOME  = "MOVE_TO_HOME"
   string EXECUTE_SEAM  = "EXECUTE_SEAM"
   string ESTOP         = "ESTOP"
   string START_PROCESS             = "START_PROCESS"
   string REQUEST_AI_RECOMMENDATION = "REQUEST_AI_RECOMMENDATION"
   string MANUAL_ADJUST             = "MANUAL_ADJUST"
   string LAUNCH_NEW_DOE            = "LAUNCH_NEW_DOE"
   string PAUSE_PROCESS             = "PAUSE_PROCESS"
   string RESUME_PROCESS            = "RESUME_PROCESS"
   string STOP_PROCESS              = "STOP_PROCESS"
   string SOURCE_ROBOT   = "ROBOT"
   string SOURCE_REMOTE  = "REMOTE"
   string SOURCE_UNKNOWN = "UNKNOWN"
   string MODALITY_TOUCHSCREEN = "TOUCHSCREEN"
   string MODALITY_SPEECH      = "SPEECH"
   string MODALITY_GESTURE     = "GESTURE"
   string MODALITY_INTERNAL    = "INTERNAL"
   string intent
   string data
   string source
   string modality
   float32 priority
   float32 confidence

Demo launch after rebuilding the image:

.. code-block:: console

   $ ros2 launch welding_demo welding_robin_demo.launch.py
   [INFO] [welding_home_skill_node-1]: process started
   [INFO] [welding_seam_skill_node-2]: process started
   [INFO] [welding_recommendation_skill_node-3]: process started
   [INFO] [welding_manual_skill_node-4]: process started
   [INFO] [welding_http_bridge_node-5]: process started
   [welding_http_bridge] WeldingHttpBridgeNode ready - publishing on /intents,
     HTTP server on port 8766
   [welding_http_bridge_node-5] ======== Running on http://0.0.0.0:8766 ========
   [welding_supervisor] WeldingSupervisorNode ready - listening on /intents

The final launch validated the HTTP bridge, skill nodes, and supervisor in
simulation mode.

ROS graph during the final demo launch:

.. code-block:: console

   $ ros2 topic list
   /doe/launch
   /intents
   /joint_states
   /joint_states_manual
   /parameter_events
   /robin/telemetry
   /rosout
   /welding_home_skill/transition_event
   /welding_manual_skill/transition_event
   /welding_recommendation_skill/transition_event
   /welding_seam_skill/transition_event

.. code-block:: console

   $ ros2 topic info /intents
   Type: welding_msgs/msg/Intent
   Publisher count: 1
   Subscription count: 1

.. code-block:: console

   $ ros2 topic info /robin/telemetry
   Type: robin_interfaces/msg/ProcessTelemetry
   Publisher count: 1
   Subscription count: 1

.. code-block:: console

   $ ros2 service list
   /welding_home_skill/change_state
   /welding_home_skill/get_state
   /welding_home_skill/list_parameters
   /welding_http_bridge/list_parameters
   /welding_manual_skill/change_state
   /welding_manual_skill/get_state
   /welding_manual_skill/list_parameters
   /welding_recommendation_skill/change_state
   /welding_recommendation_skill/get_state
   /welding_recommendation_skill/list_parameters
   /welding_seam_skill/change_state
   /welding_seam_skill/get_state
   /welding_seam_skill/list_parameters
   /welding_supervisor/list_parameters

.. code-block:: console

   $ ros2 action list
   /welding_home_skill/execute
   /welding_manual_skill/execute
   /welding_recommendation_skill/execute
   /welding_seam_skill/execute

Dashboard/API HTTP intent validation:

.. code-block:: console

   $ curl -sS -X POST http://localhost:8766/intent \
       -H "Content-Type: application/json" \
       -d '{"intent":"REQUEST_AI_RECOMMENDATION","data":{"process_id":"ros_bridge","mode":"geometry_driven"}}'
   {"status": "published", "intent": "REQUEST_AI_RECOMMENDATION"}

.. code-block:: console

   $ ros2 topic echo --once /intents
   intent: REQUEST_AI_RECOMMENDATION
   data: '{"process_id": "ros_bridge", "mode": "geometry_driven"}'
   source: REMOTE
   modality: TOUCHSCREEN
   priority: 0.5
   confidence: 1.0

Supervisor and skill logs confirmed routing:

.. code-block:: text

   [welding_http_bridge] Published intent: REQUEST_AI_RECOMMENDATION | data:
     {"process_id": "ros_bridge", "mode": "geometry_driven"}
   [welding_supervisor] Received intent: 'REQUEST_AI_RECOMMENDATION' |
     modality: TOUCHSCREEN | data: {"process_id": "ros_bridge", "mode": "geometry_driven"}
   [welding_supervisor] REQUEST_AI_RECOMMENDATION: goal sent
   [welding_recommendation_skill] REQUEST_AI_RECOMMENDATION goal received:
     process='ros_bridge' mode='geometry_driven'
   [welding_supervisor] REQUEST_AI_RECOMMENDATION: result -> SUCCESS |
     "Recommendation ready for process ros_bridge"

Telemetry and FIWARE validation:

.. code-block:: console

   $ ros2 run robin_core_data telemetry_aggregator_node.py
   [telemetry_aggregator] Telemetry aggregator started | geometry: /robin/weld_dimensions |
     fronius: /robin/data/fronius | output: /robin/telemetry | period: 1.0s
   [telemetry_aggregator] [AGG] Published telemetry #1 current=0.0A voltage=0.0V
     width=0.00mm height=0.00mm

.. code-block:: console

   $ ros2 topic info /robin/telemetry
   Type: robin_interfaces/msg/ProcessTelemetry
   Publisher count: 2
   Subscription count: 1

.. code-block:: console

   $ ros2 topic echo --once /robin/telemetry
   header:
     stamp:
       sec: 1782232770
       nanosec: 142227266
     frame_id: base_link
   current: 0.0
   voltage: 0.0
   speed: 0.0
   width: 0.0
   height: 0.0
   cross_sectional_area: 0.0

.. code-block:: console

   $ curl -sS http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:Process:ros_bridge \
       -H "Accept: application/json"
   {
     "id": "urn:ngsi-ld:Process:ros_bridge",
     "type": "urn:robin:Process",
     "urn:robin:processTelemetry": {
       "type": "Property",
       "value": {
         "current": 0,
         "voltage": 0,
         "speed": 0,
         "width": 0,
         "height": 0,
         "cross_sectional_area": 0,
         "header": {"frame_id": "base_link", "stamp": {"sec": 1782232768}}
       },
       "ddsDataType": {
         "type": "Property",
         "value": "robin_interfaces::msg::dds_::ProcessTelemetry_"
       }
     }
   }

The Orion-LD entity also contained existing non-interface attributes from prior
application state; they are omitted above because the validation target was the
DDS-mapped ``urn:robin:processTelemetry`` property.

Validation Limitations
~~~~~~~~~~~~~~~~~~~~~~

* Hardware mode was not validated. The successful action validation used the
  default simulation behavior of the ``welding_*`` skills.
* A stale local ``vulcanexus`` image failed the first launch attempt because
  ``aiohttp`` was missing from its active Python environment. Rebuilding the
  image installed ``aiohttp`` 3.14.1, and the final HTTP bridge validation
  passed. Treat stale local images as a setup risk.
* ``/robin/telemetry`` was validated by running the telemetry aggregator
  directly with default zero values. Live hardware source data from Garmo,
  Fronius, WAGO, or UR was not validated.
* ``/profilometer_activate`` and ``/profilometer_deactivate`` have a current
  provider/caller service type mismatch: callers expect
  ``robin_interfaces/srv/SensorCommand``, but ``sensor_cmd.py`` advertises
  ``std_srvs/srv/Trigger``.
