# Changelog

All notable changes to `open-robin` are documented in this file.

This project follows the spirit of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and uses semantic versioning for public release tags.

## 0.1.0 - 2026-06-30

Initial public release for `open-robin`, a reusable process
intelligence module for robotized manufacturing processes.

### Added

- Process Intelligence API in `robin/` for process lifecycle management,
  measurement access, deviation checks, AI-assisted recommendations, health
  checks, and FIWARE/NGSI-LD integration.
- ROBIN Dashboard in `robin-dashboard/` for live process monitoring, KPI
  visualization, deviation monitoring, AI trust information, process controls,
  and history/export workflows.
- ROS 2-to-FIWARE integration pattern in `vulcanexus_ws/`, including ROS 2
  intent/skill packages, normalized telemetry publishing, a hardware-agnostic
  synthetic demo, generic hardware-mode integration interfaces, launch
  documentation, and DDS/NGSI-LD mapping configuration.
- `config-dds.json` DDS Enabler mapping and FIWARE/NGSI-LD data-model reference
  for `Process`, `Measurement`, `GeometryTarget`, `Alert`, and
  `AIRecommendation` entities.
- No-hardware hello world in `docs/quickstart.rst`, covering API health,
  process creation, mock measurement ingestion, API read-back, dashboard
  visibility, and AI-assisted prediction.
- Canonical demo documentation and profile-driven examples for the welding
  reference profile and the spray-coating reuse profile, including
  `demo/profiles/welding_profile.py`, `demo/profiles/spray_coating_profile.py`,
  expected API/FIWARE behavior, dashboard state, and deviation/alert behavior.
- Profile adaptation guidance documenting configurable vocabulary, labels,
  fields, ROS 2 topics, skills, DDS mapping, AI model path, and what remains
  demonstrator-specific.
- Optional ROS 2 intent-driven simulation entry point in
  `demo/simulation-demo-ros2-live.sh` for exercising dashboard intents,
  supervisor/skill routing, synthetic telemetry, DDS/FIWARE persistence, and
  dashboard monitoring without industrial hardware.
- Evidence pack under `media/`, including architecture diagrams, dashboard and
  demonstrator screenshots, representative API responses, a CSV export example,
  and AI model evidence.
- Documentation pages for open/proprietary boundaries, ROS 2/Vulcanexus
  interfaces, ROS4HRI/ROS4RI alignment, FIWARE/NGSI-LD and DDS mapping,
  troubleshooting, limitations, and release readiness.
- Test and coverage gates for the release-critical Python scope and scoped
  dashboard unit-test surface.

### Changed

- Repository metadata, license references, package metadata, support contacts,
  and stale repository URLs were aligned with the public `3D-Components/open-robin`
  repository and AGPL-3.0-only license.
- Docker Compose defaults were kept hardware-neutral for demo and evaluation use, with
  Linux/NVIDIA physical demonstrator settings isolated in
  `docker-compose.linux-gpu.override.yaml`.
- Documentation now separates the minimal hello world from richer simulated demo
  workflows so users can choose the shortest reproducible path first.
- The canonical evaluation path is documented as no-hardware hello world first,
  then profile/mock demos; recorded ROS bag replay remains a supporting utility
  rather than a required clean-clone path.

### Documented limitations

- Performed end-to-end validation evidence comes from one welding reference
  demonstrator. The spray-coating profile demonstrates configuration reuse and
  deviation-against-target behavior, but broad non-welding industrial validation
  remains limited and no spray-coating AI model is committed.
- Physical hardware backends for UR10e, Fronius TPS320i, WAGO/OPC UA, Garmo
  Garline, and the MIL cell are demonstrator/deployment-specific. The open ROS 2
  workspace provides the synthetic no-hardware path and generic hardware-mode
  interfaces that adopters can implement for their own cells.
- The local/demo stack is not a production authentication, authorization,
  audit-policy, secret-rotation, or multi-tenant deployment template.
- Industrial safety certification, certified emergency-stop behavior, and
  hardware acceptance testing are outside the scope of this open repository.
- ROS 2 `launch_testing`, hardware-in-the-loop automation, browser end-to-end
  tests, and full cross-stack performance benchmarking are outside this release
  baseline.
