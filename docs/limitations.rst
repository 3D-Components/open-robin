Known Limitations
=================

This page documents the current limits of the ``open-robin`` module so they are
visible to users and adopters.  The repository provides reusable process
intelligence components and a reference integration pattern; it is not a
turnkey certified production deployment for every robotized cell.

Validation Scope
----------------

* The performed end-to-end validation evidence comes from one welding reference
  demonstrator, so current industrial validation should be treated as
  single-cell validation rather than broad multi-site validation.
* Non-welding reuse is demonstrated through the profile mechanism and a
  spray-coating profile, but broad non-welding industrial validation is still
  limited.
* The repository includes representative evidence, API examples, and demo
  artifacts.  Final acceptance for a new industrial process still requires
  local validation with that process, its sensors, and its operator workflow.

Hardware and Deployment Constraints
-----------------------------------

* Welding hardware adapters are demonstrator-specific examples.  They reference
  hardware such as UR10e, Fronius TPS320i, WAGO/OPC UA, and Garmo Garline
  components.
* Mechatronics Innovation Lab cell layout, network details, credentials, safety procedures, and
  production deployment settings are not part of the open module.
* ROS 2/DDS discovery behavior can differ between native Linux, Docker Desktop
  on macOS, and customer networks.  Linux remains the preferred environment for
  full ROS 2/Vulcanexus/DDS validation.
* The default Docker Compose stack is hardware-neutral and intended for demo
  quickstart and no-hardware demo paths.  The full physical demonstrator profile
  uses ``docker-compose.linux-gpu.override.yaml`` and was validated on the
  specific Linux/NVIDIA workstation and industrial cell setup available to the
  project team.  macOS and non-NVIDIA hosts should be treated as supported only
  for the portable core/demo stack unless separately validated.
* The Docker Compose stack is an integration example.  Production deployments
  should adapt networking, service discovery, persistent storage, secrets, and
  observability to the target site.

AI and Domain Model Scope
-------------------------

* The open model artifacts are included to exercise the runtime model interface
  and recommendation flow.  They are not a universal model for every robotized
  process.
* Welding-trained models and welding benchmark evidence are demonstrator
  artifacts.  A new domain should provide its own data, validation metrics, and
  model-governance process.
* Proprietary/offline model evidence is limited to the external benchmark
  artifact ``media/model-evidence/proprietary-offline-model-r2-096-scatter.png``,
  which reports R2 about 0.96 and is described in
  ``media/model-evidence/README.md``.  This is evidence for the demonstrator
  benchmark, not an open reusable model artifact.
* Private raw datasets, cleaned training datasets, production retraining
  pipelines, and customer-specific profile parameters are intentionally
  excluded.

Security and Production Hardening
---------------------------------

* The current local/demo stack does not provide production authentication,
  authorization, audit policy, secret rotation, or tenant isolation.  This keeps
  the demo simple and follows a data-minimization/privacy-by-design
  posture: the open demo should avoid collecting or storing user personal data
  unless a deployment explicitly adds the required governance and controls.
* Multi-tenant deployment is future work.  The current module can be adapted to
  tenant-aware infrastructure, but that design is outside the included demo
  stack.
* HTTPS termination, identity provider integration, role-based access control,
  and industrial network segmentation must be added by the deployment owner.

Safety and Certification
------------------------

* ``open-robin`` provides monitoring, deviation detection, and advisory
  recommendations.  It does not replace industrial safety systems, robot safety
  controllers, risk assessment, or certified emergency-stop behavior.
* The human operator remains the final authority for process decisions and can
  accept, reject, override, or adjust recommendations.
* Industrial safety certification and hardware acceptance testing are out of
  scope for this open repository.

Testing Gaps
------------

* Unit and API tests cover selected backend behavior, and the dashboard has
  first-party Vitest tests for shared utilities, API helpers, AI input
  normalization, and reusable UI primitives.
* The release-critical Python coverage scope is currently ``robin`` plus
  ``mlops``.  CI enforces a 90% line-coverage gate for that scope.
* Dashboard unit coverage is published under the Codecov flag
  ``dashboard-unit`` and has a 90% line-coverage gate for the scoped unit
  surface.  Full dashboard feature/page coverage and browser end-to-end tests
  remain future work.
* ROS 2 ``launch_testing`` coverage is future work and is not included in this
  release baseline.
* Hardware-in-the-loop testing automation is future work and is not included in
  this release baseline.

Recommended Use
---------------

Use the repository as:

* a reusable Process Intelligence API and dashboard module;
* a documented ROS 2-to-FIWARE/DDS integration pattern;
* a profile-driven demo environment for user and adopter evaluation;
* a reference implementation for adapting another robotized manufacturing
  process.

Do not treat the repository as:

* a drop-in certified controller for industrial safety;
* a publication of private datasets or proprietary production models;
* a complete production-authentication or multi-tenant deployment template;
* proof that every non-welding domain has already been validated.
