Domain Profiles
===============

ROBIN is domain-agnostic.  The same stack works for welding, spray coating,
machining, or any robotic manufacturing process.  Switching domains is a
single environment variable - no code changes, no rebuild.

Switching Profiles
------------------

.. code-block:: bash

   ROBIN_PROFILE=spray_coating docker compose up -d

The Alert Engine loads the new profile at startup.  The dashboard fetches it at
runtime via ``GET /profile``, so labels, units, and AI config update
automatically.

Default profile: ``welding``.

Profile Comparison
------------------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Concept
     - Welding (default)
     - Spray Coating
   * - Process name
     - Weld
     - Coat Job
   * - Height metric
     - Bead Height (mm)
     - Coating Thickness (mm)
   * - Width metric
     - Bead Width (mm)
     - Coverage Width (mm)
   * - Speed
     - Wire Speed (m/min)
     - Line Speed (mm/s)
   * - Current / Flow
     - Current (A)
     - Flow Rate (ml/min)
   * - Voltage / Pressure
     - Voltage (V)
     - Pressure (bar)
   * - AI default mode
     - parameter_driven
     - geometry_driven

Canonical Profile Demos
-----------------------

Run the robust dual-mode demos directly from profile scripts:

.. code-block:: bash

   # Welding (default profile)
   python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2

   # Spray coating profile
   ROBIN_PROFILE=spray_coating docker compose up -d
   python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2

How Profiles Work
-----------------

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

Profile files live in ``config/profiles/`` and are plain YAML.  A profile
contains:

* **vocabulary** - dashboard labels (process, speed, current, voltage, etc.)
* **fields** - telemetry field display names and units
* **ros2.topics** - ROS 2 topic names for geometry, process parameters, pose
* **skills** - robot capabilities (service/action names, types, descriptions)
* **ai** - feature order, model path, default tolerance, default mode
* **dds** - DDS bridge topic mapping

See ``config/profiles/welding.yaml`` for a complete example.

Per-Profile AI Models
~~~~~~~~~~~~~~~~~~~~~

Each profile specifies its model checkpoint in the ``ai.model_path`` field:

* ``data/models/welding/process_geometry_mlp.pt``
* ``data/models/spray_coating/process_geometry_mlp.pt``

Train new models with:

.. code-block:: bash

   python scripts/train_profile_model.py

Creating a New Profile
----------------------

1. Copy an existing profile YAML:

   .. code-block:: bash

      cp config/profiles/welding.yaml config/profiles/machining.yaml

2. Edit vocabulary, fields, ROS 2 topics, skills, and AI config to match the
   new domain.

3. Train a model for the new profile:

   .. code-block:: bash

      ROBIN_PROFILE=machining python scripts/train_profile_model.py

4. Launch with the new profile:

   .. code-block:: bash

      ROBIN_PROFILE=machining docker compose up -d

See ``demo/profiles/README.md`` for a detailed comparison and step-by-step
instructions.
