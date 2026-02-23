# ARISE Open-Source Publication Roadmap

## Purpose

This document defines what will be published as reusable open-source modules from ROBIN, what remains domain-specific, and how publication will be staged for ARISE catalog onboarding.

Date of this roadmap: **February 22, 2026**.

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
  - `vulcanexus_ws/src/robin_core_data/scripts/telemetry_aggregator_node.py`
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

These components are kept as adapters or demos and are not mandatory for reusing the core:

- welding hardware drivers and control sequencing (`robin_hardware_fronius`, WAGO/OPC-UA specifics)
- welding-specific ROS bags and hardware launch configurations
- welding-only operator narratives and experiment scripts

The intended message: the same core can be reused for other robotic process domains by swapping profile adapters and topic mappings.

---

## Current Execution Status (as of February 22, 2026)

Completed in repository:

- DDS-first telemetry flow documented and active (`demo/simulation-demo-rosbag.sh`, `config-dds.json`)
- legacy HTTP NGSI bridge removed from active baseline and docs
- FAROS domain vocabulary overrides documented (`VITE_TERM_*`)
- welding content reframed as **profile layer** in demo docs/scripts
- second non-welding profile added: `demo/profiles/spray_coating_profile.py`
- ARISE metadata file populated: `arise/catalog-metadata.yaml`
- CI extended to include Python tests and frontend build evidence (`.github/workflows/ci.yml`)

Execution checklist:

- `ARISE_PUBLICATION_CHECKLIST.md`

---

## Publication Milestones (Concrete Dates)

### Milestone 1: Core freeze and cleanup
- **Target date: March 15, 2026**
- Deliverables:
  - remove/mark legacy bridge docs
  - confirm generic naming across core modules
  - finalize `ProcessTelemetry` DDS path documentation

### Milestone 2: Public module packaging
- **Target date: March 29, 2026**
- Deliverables:
  - versioned release candidate tag for reusable core
  - module-level READMEs for API, ROS pipeline, UI shell
  - minimal CI checks (lint, unit tests, smoke startup)

### Milestone 3: ARISE catalog submission package
- **Target date: April 12, 2026**
- Deliverables:
  - ARISE metadata file (`arise/catalog-metadata.yaml`)
  - architecture and dependency diagram
  - reproducible demo script and expected outputs

### Milestone 4: Public release v1
- **Target date: April 26, 2026**
- Deliverables:
  - public repository with release notes
  - changelog + compatibility statement
  - support channels and issue templates

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

- permissive license (`MIT`)
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
