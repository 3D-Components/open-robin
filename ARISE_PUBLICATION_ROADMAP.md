# ARISE Open-Source Publication Roadmap

## Purpose

This document defines what will be published as reusable open-source modules from ROBIN, what remains domain-specific, and how publication will be staged for ARISE catalog onboarding.

Last reviewed: **June 22, 2026**.

Current release target: **v0.1.0 public release readiness**.

---

## Reusable Open-Source Core (Public Scope)

### 1) Process data and lifecycle core

- Path: `robin/`
- Includes:
  - NGSI-LD entity orchestration for `Process`, `Measurement`, `GeometryTarget`, `AIRecommendation`
  - process lifecycle APIs (create, stop, resume, status)
  - Mintaka-first measurement retrieval with Orion fallback

### 2) Alert/deviation logic

- Path: `robin/alert_engine.py`
- Includes:
  - deviation checks for parameter-driven and geometry-driven modes
  - operator-action API hooks
  - process target handling

### 3) AI model serving hooks

- Paths: `robin/ai/`, `scripts/train_geometry_mlp.py`
- Includes:
  - model loading/selection
  - prediction endpoint integration
  - baseline training pipeline on synthetic or real datasets

### 4) ROS 2 measurement pipeline (DDS-first)

- Paths:
  - `vulcanexus_ws/src/robin_interfaces/msg/ProcessTelemetry.msg`
  - `vulcanexus_ws/src/robin_telemetry/robin_telemetry/telemetry_aggregator_node.py`
  - `config-dds.json`
- Includes:
  - ROS topic aggregation to a single telemetry message
  - direct DDS mapping into Orion-LD (`urn:robin:processTelemetry`)
  - no legacy HTTP bridge requirement

### 5) Monitoring UI shell

- Path: `robin-dashboard/`
- Includes:
  - process selector + telemetry + deviation monitoring
  - configurable domain terms via `VITE_TERM_*` env vars
  - domain-agnostic defaults in the open-source profile

---

## Domain/Profile Layer (Not Core)

These domain-specific components are not part of the open release and are not mandatory
for reusing the core:

- welding hardware drivers and control sequencing (vendor / WAGO / OPC-UA specifics) —
  removed from the open module; adopters supply their own behind the skills' generic
  action/service interfaces
- welding-specific ROS bags and hardware launch configurations
- welding-only operator narratives and experiment scripts

The intended message: the same core can be reused for other robotic process domains by swapping profile adapters and topic mappings.

---

## Current Execution Status (as of June 22, 2026)

Completed in repository:

- DDS-first telemetry flow documented and active (`demo/simulation-demo-rosbag.sh`, `config-dds.json`)
- legacy HTTP NGSI bridge removed from active baseline and docs
- dashboard domain vocabulary fallback overrides documented (`VITE_TERM_*`)
- welding content reframed as **profile layer** in demo docs/scripts
- second non-welding profile added: `demo/profiles/spray_coating_profile.py`
- ARISE metadata file populated: `arise/catalog-metadata.yaml`
- CI extended to include Python tests and frontend build evidence (`.github/workflows/ci.yml`)
- evidence pack, screenshots, example responses, CSV export, and model evidence added under `media/`
- license, repository metadata, contact details, and third-party notices aligned with AGPL-3.0-only
- open/reusable boundary documented in `docs/open_boundary.rst`
- limitations and reviewer troubleshooting documented in `docs/limitations.rst` and `docs/reference/troubleshooting.rst`

Still pending before the final `v0.1.0` tag:

- reviewer quickstart and no-hardware hello world must be verified from a clean clone
- ROS 2 / Vulcanexus interface reference must be completed
- ROS4HRI / ROS4RI intent-skill-task-mission mapping must be completed
- FIWARE / NGSI-LD data model and DDS mapping reference must be completed
- changelog and release notes draft must be prepared
- final repository hygiene and clean-clone verification must pass

Execution checklist:

- `ARISE_PUBLICATION_CHECKLIST.md`

---

## Publication Milestones

### Milestone 1: Core scope and metadata cleanup
- **Status: completed for current `dev`**
- Completed:
  - legacy bridge docs removed or marked outside the active baseline
  - generic process-intelligence naming clarified across core docs
  - `ProcessTelemetry` DDS path documented as the current ROS 2 baseline
  - license, repository URL, maintainer, and catalog metadata aligned

### Milestone 2: Evidence and open boundary
- **Status: completed for current `dev`**
- Completed:
  - evidence index and media artifacts added
  - open/reusable, proprietary/excluded, and welding-specific boundaries documented
  - human-final-authority statement included in evidence docs
  - reviewer evidence wording kept public and module-oriented

### Milestone 3: Reviewer reproducibility and interface references
- **Status: in progress**
- Target before tagging:
  - no-hardware quickstart and hello world
  - basic demo path with expected outputs
  - ROS 2 / Vulcanexus interface reference
  - ROS4HRI / ROS4RI mapping
  - FIWARE / NGSI-LD and DDS mapping reference
  - troubleshooting and limitations pages kept aligned with the verified path

### Milestone 4: Release candidate
- **Status: pending**
- Target before tagging:
  - `CHANGELOG.md` and release notes draft
  - final repository hygiene pass
  - final clean-clone verification
  - approved commit hash for `v0.1.0`

---

## ARISE Alignment Checklist

### Standards and interoperability

- NGSI-LD data exchange through Orion-LD
- ROS 2 / DDS integration pattern for machine telemetry
- explicit, documented entity model and context usage

### Reusability

- generic `Process` core model
- domain vocabulary decoupled from UI via env config
- adapters and demos separated from core logic

### Openness

- open-source copyleft license (`AGPL-3.0-only`)
- public code + docs + reproducible startup path
- clear boundary of what is open vs what is proprietary

### Maintainability

- testable API modules
- scripted demos
- roadmap with dated milestones and ownership expectations

---

## Demo Packaging for ARISE

- keep welding scripts under `demo/` as the ROS 2 + DDS reference profile
- include non-welding synthetic profile under `demo/profiles/`
- keep profile-specific constants and labels isolated in profile scripts/config only
