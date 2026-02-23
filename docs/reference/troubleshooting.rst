Troubleshooting Guide
=====================

This guide covers common issues in the current ROBIN stack.

Service Startup Issues
----------------------

FIWARE services not starting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Check runtime state:

.. code-block:: bash

   docker compose ps
   docker compose logs orion-ld
   docker compose logs mintaka

If needed:

.. code-block:: bash

   docker compose down
   docker compose up -d --build

Connection Issues
-----------------

Cannot reach Orion or Mintaka
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl http://localhost:1026/version
   curl 'http://localhost:9090/temporal/entities?limit=1'

Cannot reach Alert Engine
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   curl http://localhost:8001/health
   docker compose logs alert-processor

Data Model Issues
-----------------

Process entity not found
~~~~~~~~~~~~~~~~~~~~~~~~

Verify process entities in Orion:

.. code-block:: bash

   curl -H "NGSILD-Tenant: robin" \
        "http://localhost:1026/ngsi-ld/v1/entities?type=urn:robin:Process"

Measurements missing in API
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Check API endpoint:

.. code-block:: bash

   curl "http://localhost:8001/process/ros_bridge/measurements?last=20"

If empty:

1. verify that telemetry is being published in ROS 2
2. verify DDS mapping in ``config-dds.json``
3. check Orion and Mintaka logs

DDS Telemetry Issues
--------------------

No data from ROS 2 to Orion
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Confirm aggregator is running:

.. code-block:: bash

   ros2 topic echo /robin/telemetry

Confirm mapping configuration:

* DDS topic: ``rt/robin/telemetry``
* entity: ``urn:ngsi-ld:Process:ros_bridge``
* attribute: ``urn:robin:processTelemetry``

Note: legacy HTTP NGSI bridge is not part of the current baseline.

Frontend Issues
---------------

UI not loading
~~~~~~~~~~~~~~

.. code-block:: bash

   curl http://localhost:5174
   docker compose logs robin-dashboard

Wrong API target in UI
~~~~~~~~~~~~~~~~~~~~~~

Set ``VITE_ROBIN_API_URL`` at build/run time and rebuild frontend if needed.

Domain wording in UI
~~~~~~~~~~~~~~~~~~~~

Dashboard labels are loaded at runtime from the active profile YAML
(``config/profiles/<name>.yaml``).  Switch profiles with
``ROBIN_PROFILE=<name> docker compose up -d``.
See :doc:`configuration` for details.

Performance Issues
------------------

Stack feels slow
~~~~~~~~~~~~~~~~

.. code-block:: bash

   docker stats

For high-rate rosbag playback, reduce playback rate and verify Mintaka/Timescale health.

Getting Help
------------

If issues persist:

1. capture logs from Orion, Mintaka, and alert-processor
2. record the exact failing command/API call
3. include configuration snippets (without secrets) in your issue report

