Configuration Reference
=======================

This page documents the active configuration model for the ROBIN process-intelligence core.

Service Configuration
---------------------

Main runtime composition is defined in ``docker-compose.yaml``.

Core services:

* **Orion-LD** (`1026`)
* **MongoDB** (`27017`)
* **TimescaleDB** (`5433`)
* **Mintaka** (`9090`)
* **Alert Engine** (`8001` host, `8000` in container)
* **ROBIN Dashboard** (`5174`)

ROS/DDS Notes
~~~~~~~~~~~~~

Orion-LD and Vulcanexus containers run in host-network mode for DDS discovery.

Alert Engine reaches Orion through host-gateway mapping:

.. code-block:: yaml

   extra_hosts:
     - "orion-ld:host-gateway"

DDS to NGSI-LD Mapping
----------------------

``config-dds.json`` defines the direct mapping from ROS/DDS telemetry into Orion-LD.

Current mapping:

.. code-block:: json

   {
     "dds": {
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

The legacy HTTP NGSI bridge is not required in the current baseline.

Profile Configuration
---------------------

Each domain is defined by a YAML file in ``config/profiles/``.  Select the active
profile via the ``ROBIN_PROFILE`` environment variable:

.. code-block:: bash

   ROBIN_PROFILE=spray_coating docker compose up -d

Default: ``welding``.

A profile YAML contains:

* **vocabulary** - dashboard labels (process, speed, current, voltage, etc.)
* **fields** - telemetry field display names and units
* **ros2.topics** - ROS 2 topic names for geometry, process parameters, pose
* **skills** - robot capabilities (service/action names, types, descriptions)
* **ai** - feature order, model path, default tolerance, default mode
* **dds** - DDS bridge topic mapping

See ``config/profiles/welding.yaml`` for a complete example.

.. mermaid::

   graph TD
       subgraph profiles ["config/profiles/"]
           W["welding.yaml"]
           S["spray_coating.yaml"]
       end
       ENV["ROBIN_PROFILE\nenv var"] -->|selects| LOADER["ProfileLoader"]
       W --> LOADER
       S --> LOADER
       LOADER -->|"vocabulary\n+ skills"| ENDPOINT["GET /profile"]
       LOADER -->|"model_path"| AILOAD["Load AI\ncheckpoint"]
       LOADER -->|"feature_order\n+ tolerance"| DEV["Deviation\ndetection"]
       ENDPOINT -->|"runtime fetch"| DASH["Dashboard renders\ndomain labels"]

The dashboard fetches the active profile from the Alert Engine at runtime via
``GET /profile``.  No rebuild is needed to switch domains.

Per-Profile AI Models
~~~~~~~~~~~~~~~~~~~~~

Each profile specifies its model checkpoint in the ``ai.model_path`` field:

* ``data/models/welding/process_geometry_mlp.pt``
* ``data/models/spray_coating/process_geometry_mlp.pt``

Train new models with ``scripts/train_profile_model.py``.

Environment Variables
---------------------

Alert Engine:

* ``ROBIN_PROFILE``: active profile name (default: ``welding``)
* ``MINTAKA_URL``: Mintaka endpoint (default in compose: ``http://mintaka:8080``)
* ``NGSILD_TENANT``: tenant header for Orion/Mintaka requests

Dashboard (build-time, used as fallback if ``/profile`` is unreachable):

* ``VITE_ROBIN_API_URL``: Alert Engine base URL

Data Model Configuration
------------------------

NGSI-LD context includes:

* ``Process`` mapped to ``urn:robin:Process``
* ``Measurement`` mapped to ``urn:robin:Measurement``
* ``GeometryTarget`` mapped to ``urn:robin:GeometryTarget``
* ``AIRecommendation`` mapped to ``urn:robin:AIRecommendation``

Process entity example:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:Process:process_id",
     "type": "urn:robin:Process",
     "operationMode": {"type": "Property", "value": "parameter_driven"},
     "toleranceThreshold": {"type": "Property", "value": 10.0, "unitCode": "P1"}
   }

Geometry target example:

.. code-block:: json

   {
     "id": "urn:ngsi-ld:GeometryTarget:process_id",
     "type": "urn:robin:GeometryTarget",
     "targetHeight": {"type": "Property", "value": 5.0, "unitCode": "MMT"},
     "targetWidth": {"type": "Property", "value": 8.0, "unitCode": "MMT"}
   }

For operation modes (parameter-driven vs geometry-driven), see
:doc:`/user_guide/overview`.  For troubleshooting after configuration changes,
see :doc:`/reference/troubleshooting`.

