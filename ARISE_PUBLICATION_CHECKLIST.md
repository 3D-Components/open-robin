# ARISE Publication Checklist

Last updated: **February 22, 2026**.

Use this checklist to finalize public release and ARISE catalog submission.

## 1. Open-Core Scope

- [x] Core modules are domain-agnostic (`Process`, `Measurement`, `GeometryTarget`, `AIRecommendation`).
- [x] ROS 2 ingestion path is DDS-first.
- [x] Legacy HTTP NGSI bridge is removed from active workflow.
- [x] Core-vs-profile boundary is documented in README and roadmap.

## 2. Reusable Component Evidence

- [x] Process API and lifecycle endpoints documented (`robin/`, `robin/alert_engine.py`).
- [x] Alert/deviation engine logic documented and test-covered.
- [x] ROS 2 telemetry schema + aggregator published:
  - [x] `vulcanexus_ws/src/robin_interfaces/msg/ProcessTelemetry.msg`
  - [x] `vulcanexus_ws/src/robin_core_data/scripts/telemetry_aggregator_node.py`
  - [x] `config-dds.json`
- [x] Monitoring UI shell published (`robin-dashboard/`).

## 3. Profile Demonstration Evidence

- [x] Welding reference profile demo is available under `demo/`.
- [x] Non-welding synthetic profile is available under `demo/profiles/`.
- [x] Demo docs use profile language (welding is not the core identity).

## 4. Publication Artifacts

- [x] Publication-ready root README (`README.md`).
- [x] ARISE roadmap with dates (`ARISE_PUBLICATION_ROADMAP.md`).
- [x] ARISE metadata file (`arise/catalog-metadata.yaml`).
- [x] Stack snapshot (`current_stack_feb22.md`).
- [x] Demo profile documentation (`demo/README.md`, `demo/profiles/README.md`).

## 5. Quality and CI Evidence

- [x] Backend tests pass locally (`poetry run pytest -q`).
- [x] Pre-commit checks pass locally (`poetry run pre-commit run --all-files`).
- [x] Frontend production build passes locally (`npm run build` in `robin-dashboard`).
- [x] CI pipeline includes both backend tests and frontend build (`.github/workflows/ci.yml`).

## 6. Release Execution (Pending Before Final Submission)

- [ ] Push final repository state to public GitHub repository.
- [ ] Verify GitHub Actions passes on default branch.
- [ ] Create release tag (recommended: `v0.1.0`) and release notes.
- [ ] Attach architecture diagram asset to release/catalog submission package.
- [ ] Submit ARISE catalog entry using:
  - [ ] repository URL
  - [ ] `arise/catalog-metadata.yaml`
  - [ ] roadmap and demo profile links

## 7. Final Consistency Check

- [ ] Validate all URLs in docs point to public repository.
- [ ] Confirm contact email in metadata is monitored.
- [ ] Confirm README quick-start steps run on a clean machine.
