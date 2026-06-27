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
   * - Primary AI inputs
     - Wire Feed Speed (m/min), Travel Speed (m/s), Arc Length Correction (mm)
     - Line Speed (mm/s), Flow Rate (ml/min), Nozzle Pressure (bar)
   * - Auxiliary measured telemetry
     - Wire Speed, Welding Current, Arc Voltage
     - Line Speed, Flow Rate, Nozzle Pressure
   * - AI default mode
     - parameter_driven
     - geometry_driven

Canonical Profile Demos
-----------------------

Run the robust dual-mode demos directly from profile scripts.  Each script
will create the process, configure AI expectations, then **wait for you to
press Start from the dashboard** before streaming data (pass ``--no-prompt``
to skip the wait):

.. code-block:: bash

   # Welding (default profile)
   python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2
   # -> select the process in the dashboard and press Start

   # Spray coating profile
   ROBIN_PROFILE=spray_coating docker compose up -d
   python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2
   # -> select the process in the dashboard and press Start

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

* **vocabulary** - dashboard labels for the profile's process and telemetry terms
* **fields** - telemetry field display names and units
* **ros2.topics** - ROS 2 topic names for geometry, process parameters, pose
* **skills** - robot capabilities (service/action names, types, descriptions)
* **ai** - feature order, model path, default tolerance, default mode,
  inverse bounds, and inverse-optimizer settings
* **dds** - DDS bridge topic mapping

See ``config/profiles/welding.yaml`` for a complete example.

Per-Profile AI Models
~~~~~~~~~~~~~~~~~~~~~

Each profile specifies its model checkpoint in the ``ai.model_path`` field.  The reference
**welding** profile points at a committed, trained checkpoint, so its AI path works out of the
box.  The **spray-coating** profile points at ``data/models/spray_coating/process_geometry_mlp.pt``,
which is **not committed** - so the spray demo runs the configuration-reuse and
deviation-against-target path rather than an AI-prediction path.

.. note::

   If a profile's ``ai.model_path`` is missing on disk, the engine falls back to whatever
   ``*.pt`` it can find, which may be another profile's model with different feature names.
   Train and commit a per-profile model to enable the AI path honestly:

   .. code-block:: bash

      poetry run python scripts/train_profile_model.py
      # writes data/models/<profile>/process_geometry_mlp.pt for each profile

   The same reuse procedure applies when running without a model: the deviation reference
   falls back to the target geometry instead of an AI prediction (as in the spray-coating
   profile below).

Creating a New Profile
----------------------

In short: copy ``config/profiles/welding.yaml``, edit the ``vocabulary``, ``fields``,
``ros2.topics``, ``skills``, ``ai``, and ``dds`` sections for your domain, optionally train a
per-profile model, and launch with ``ROBIN_PROFILE=<name> docker compose up -d``.

This page is the canonical reuse procedure - the configurable-vs-welding-specific boundary, the
welding -> spray-coating worked example, telemetry/topic/skill mapping, and the no-model path are
all covered above. See also ``demo/profiles/README.md`` for a side-by-side profile comparison.
