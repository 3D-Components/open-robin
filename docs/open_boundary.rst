Open Boundary and Demonstrator Scope
====================================

This page defines the publication boundary for ``open-robin``: an open
process-intelligence stack for robotized manufacturing processes.  The welding
cell is the reference demonstrator used to validate the stack, not the only
intended application.

The same core concepts can be reused for welding, spray coating, machining,
inspection, or other robotized processes where a process has measurable outputs,
operator decisions, telemetry history, and optional AI-assisted
recommendations.

Reusable Open Parts
-------------------

The following parts are in scope for the open ARISE module:

* **Process Intelligence API**: the FastAPI backend in ``robin/`` for process
  lifecycle management, measurements, deviation checks, history access, and
  AI-assisted recommendations.
* **ROBIN Dashboard**: the React dashboard in ``robin-dashboard/`` for operator
  monitoring, alerts, model trust information, history export, and profile-based
  vocabulary.
* **ROS 2-to-FIWARE integration pattern**: the open pattern for aggregating ROS
  2 telemetry, publishing a normalized telemetry message, and persisting it via
  FIWARE/NGSI-LD.
* **DDS Enabler configuration and mapping**: ``config-dds.json`` and the
  documented mapping from ROS 2/DDS telemetry into Orion-LD and temporal
  history.
* **Demo, mock, and profile-driven execution path**: public profile files,
  demo scripts, and sample payloads that let you exercise the stack
  without the original industrial cell.
* **Open demo AI model artifacts, when included**: small runtime-facing model
  artifacts, scalers, and benchmark summaries committed under ``data/`` and
  ``media/model-evidence/``.  These are included to demonstrate the model
  interface and runtime recommendation path, not to publish private production
  training data.

The public profiles demonstrate how domain-specific vocabulary and process
parameters map onto the same reusable process-intelligence interfaces.  The
welding profile is a reference profile; the spray-coating profile is included
to make the reuse path explicit.

Proprietary or Excluded Parts
-----------------------------

The following parts are intentionally outside the open publication boundary:

* proprietary ROBIN integrations into proprietary or customer-specific
  production software;
* private production profile configurations, customer parameters, credentials,
  secrets, or site-specific environment files;
* proprietary or offline AI model artifacts and training assets that are not
  committed to this repository;
* the proprietary/offline benchmark model reported with R2 about 0.96, which is
  D3 evidence only if referenced and is not an open reusable model artifact;
* private raw datasets, cleaned production datasets, notebook exports, and
  source-specific extraction or retraining pipelines;
* MIL-specific deployment details, internal network settings, cell credentials,
  and production deployment procedures;
* industrial safety validation, certification evidence, and hardware acceptance
  procedures beyond the scope of this open software module.

The repository may include small evidence files, screenshots, and benchmark
summaries so you can understand what was validated.  Those
evidence files do not imply that private datasets, proprietary models, or
site-specific deployment material are part of the open module.

Reference Demonstrator: Welding
-------------------------------

The welding setup is the reference demonstrator for ``open-robin``.  It shows
how the reusable module can be connected to a real robotized process, but these
parts are demonstrator-specific rather than required by the core module:

* UR10e robot integration and related launch/configuration files;
* Fronius TPS320i welding power-source integration;
* WAGO / OPC UA integration used around the welding cell;
* Garmo Garline profilometer setup and laser-profile acquisition;
* welding profile vocabulary, process parameters, tolerances, and UI labels;
* welding-trained AI model behavior and welding benchmark evidence;
* MIL cell layout, network setup, and deployment assumptions;
* welding-specific ROS bags, replay paths, and hardware drivers.

These assets are useful as an implementation example.  A different robotized
process should keep the reusable API, dashboard, profile contract, FIWARE data
model, and DDS integration pattern, then replace the domain profile, adapters,
topic mappings, labels, and process-specific model.

How to Reuse Beyond Welding
---------------------------

For a new domain, start from the reusable parts and treat the welding profile as an example:
define a domain profile under ``config/profiles/``, map process telemetry onto the reusable
measurement fields, adapt the ROS 2 / DDS topic mappings while keeping the NGSI-LD process
model stable, provide a domain-specific model through the documented interface (or run without
AI), and document any private datasets, credentials, or proprietary integrations as excluded.

The profile configuration path - including the configurable-vs-welding-specific boundary and the
worked welding -> spray-coating reuse example - is described in :doc:`user_guide/profiles`.

The included spray-coating profile is the first non-welding profile example.  It shows that the
module is configured through profiles and interfaces rather than hard-coded to welding; its
expected FIWARE/API entities are committed as illustrative evidence under
``media/api-responses/spray-coating/``.

Current Limitations
-------------------

The open module has been validated primarily through the welding demonstrator
and public demo/profile paths.  Broader industrial validation, additional
non-welding hardware adapters, production authentication/authorization,
multi-site deployment hardening, and safety certification are future work or
deployment-specific responsibilities.
