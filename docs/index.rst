ROBIN Documentation
===================

**ROBIN: Open Process Intelligence Core**

.. image:: https://img.shields.io/badge/python-3.12+-blue.svg
   :target: https://www.python.org/downloads/
   :alt: Python Version

.. image:: https://img.shields.io/badge/ROS-2-blue.svg
   :target: https://docs.ros.org/en/jazzy/
   :alt: ROS 2

.. image:: https://img.shields.io/badge/FastAPI-0.116+-green.svg
   :target: https://fastapi.tiangolo.com/
   :alt: FastAPI

.. image:: https://img.shields.io/badge/FIWARE-enabled-orange.svg
   :target: https://www.fiware.org/
   :alt: FIWARE

Overview
--------

ROBIN provides **two reusable open-source modules** and a **documented integration pattern** for robotic process intelligence:

* **Module 1 - Process Intelligence API** (``robin/``): FastAPI service for process lifecycle management, deviation detection, and AI-assisted recommendations over FIWARE/NGSI-LD data.
* **Module 2 - Monitoring Dashboard** (``robin-dashboard/``): Configurable React operator dashboard for live monitoring, alerts, and AI model management.
* **Integration pattern - ROS 2 to FIWARE**: A telemetry aggregator node, message schema, and DDS mapping config that connect any ROS 2 robot to the FIWARE data layer via Orion-LD's built-in DDS bridge.

The components are domain-agnostic: they work for welding, spray coating, machining, or
any robotic manufacturing process.

.. mermaid::

   graph LR
       subgraph ros2 ["Integration Pattern"]
           SRC["Robot / Sensors"] --> AGG["Telemetry\nAggregator"]
           AGG --> TEL["/robin/telemetry"]
       end
       subgraph fiware ["FIWARE"]
           ORION["Orion-LD"] --> TSDB[("TimescaleDB")]
           TSDB --> MINTAKA["Mintaka"]
       end
       subgraph mod1 ["Module 1"]
           ALERT["Alert Engine\n+ AI Model"]
       end
       subgraph mod2 ["Module 2"]
           UI["ROBIN Dashboard"]
       end
       TEL -->|DDS| ORION
       ALERT --> ORION
       ALERT --> MINTAKA
       UI --> ALERT

Example Applications
--------------------

The repository includes demo profiles that compose the modules into working stacks
for different industrial domains:

* **Welding** (reference): ``python demo/profiles/welding_profile.py --mode both``
* **Spray Coating**: ``ROBIN_PROFILE=spray_coating docker compose up -d`` then
  ``python demo/profiles/spray_coating_profile.py --mode both``

Each demo waits for you to press **Start** from the dashboard before streaming
data. No module code is modified when switching domains. See
``demo/profiles/README.md`` for the full comparison.

For publication planning and ARISE alignment, see:

* ``../ARISE_PUBLICATION_ROADMAP.md``
* ``../arise/catalog-metadata.yaml``
* ``../current_stack_feb22.md``

Target Readiness Level
----------------------

**Current Status**: FIWARE data layer and DDS telemetry ingestion validated in lab setup.  
**Target Readiness Level**: TRL4, moving toward TRL6 with production-grade deployment and broader adapter coverage.

Documentation Structure
-----------------------

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   installation
   quickstart

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   user_guide/overview
   user_guide/dashboard
   user_guide/api
   user_guide/ai_models
   user_guide/profiles
   user_guide/demos

.. toctree::
   :maxdepth: 2
   :caption: Reference

   reference/configuration
   reference/troubleshooting

.. toctree::
   :maxdepth: 1
   :caption: Architecture & Operations

   DDS_BRIDGE_ARCHITECTURE
   RESET_DEMO_DATA

Community & Support
-------------------

* **GitHub**: `ROBIN Repository <https://github.com/Industry40Lab/open-robin>`_
* **Issues**: `Bug Reports & Feature Requests <https://github.com/Industry40Lab/open-robin/issues>`_
* **Discussions**: `Community Forum <https://github.com/Industry40Lab/open-robin/discussions>`_

License
-------

This project is licensed under the terms specified in the LICENSE file.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
