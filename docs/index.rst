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
* **Integration pattern - ROS 2 to FIWARE**: A normalized telemetry schema, demo publisher, optional aggregation path, and DDS mapping config that connect ROS 2 process data to the FIWARE data layer via Orion-LD's built-in DDS bridge.

The components are domain-agnostic: they work for welding, spray coating, machining, or
any robotic manufacturing process.

.. mermaid::

   graph LR
       subgraph ros2 ["Integration Pattern"]
           SRC["Robot / Sensors"] --> AGG["Telemetry\nPublisher / Aggregator"]
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

* **No-hardware hello world**: :doc:`quickstart` creates a process, ingests one mock
  measurement, reads it through the API, and requests an AI-assisted prediction
  without industrial hardware or private credentials.
* **Welding** (reference): ``python demo/profiles/welding_profile.py --mode both``
* **Spray Coating**: ``ROBIN_PROFILE=spray_coating docker compose up -d`` then
  ``python demo/profiles/spray_coating_profile.py --mode both``

No module code is modified when switching domains. See ``demo/profiles/README.md`` for the full
comparison.

**New here? Start with** :doc:`quickstart` - the minimal, no-hardware, copy-paste hello
world - then run the simulated welding demo in :doc:`user_guide/demos`.  The same modules
adapt to other robotized processes by swapping the profile configuration rather than the
module code (see :doc:`user_guide/profiles`).

For publication planning and ARISE alignment, see:

* ``../ARISE_PUBLICATION_ROADMAP.md``
* ``../arise/catalog-metadata.yaml``

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
   reference/fiware_ngsi_ld_dds_mapping
   reference/ros2_vulcanexus_interfaces
   reference/ros4hri_ros4ri_alignment
   reference/troubleshooting
   limitations

.. toctree::
   :maxdepth: 1
   :caption: Release

   release_notes/v0_1_0

.. toctree::
   :maxdepth: 1
   :caption: Architecture & Operations

   open_boundary
   evidence
   DDS_BRIDGE_ARCHITECTURE
   LAUNCH_AND_VERIFY_INTENTS
   ROS_BAG_PLAYBACK
   RESET_DEMO_DATA

Community & Support
-------------------

* **GitHub**: `ROBIN Repository <https://github.com/3D-Components/open-robin>`_
* **Issues**: `Bug Reports & Feature Requests <https://github.com/3D-Components/open-robin/issues>`_
* **Discussions**: `Community Forum <https://github.com/3D-Components/open-robin/discussions>`_
* **Support email**: info@3d-components.co
* **Maintainers**: Daniel Haas, Virgilio Gomez, Jayant Singh

License
-------

This project is licensed under AGPL-3.0-only as specified in the LICENSE file.
Third-party and template-origin notices are documented in ``THIRD_PARTY_LICENSES.md``.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
