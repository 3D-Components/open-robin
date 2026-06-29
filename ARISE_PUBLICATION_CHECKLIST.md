# ARISE Publication Checklist

Last updated: **June 23, 2026**.

Use this as the single release-readiness checklist for the `open-robin`
shareable HRI module submission.  It consolidates the repository audit findings
and the report-derived engineering handoff into one status tracker.

Final verification items should stay unchecked until they are validated from a
clean clone of the exact commit that will be tagged.

## 1. Repository Access and Release Metadata

- [x] Public repository URL is `https://github.com/3D-Components/open-robin`.
- [x] Repository owner/organisation is `3D-Components`.
- [x] GitHub issues are enabled.
- [x] Root `README.md` uses the official `3D-Components/open-robin` repository URL.
- [x] Root `LICENSE` exists.
- [x] License and metadata are aligned to AGPL-3.0-only.
- [x] Maintainer/contact information is visible in `README.md`.
- [ ] Submitted release tag exists and is stable: `v0.1.0`.
- [ ] GitHub release notes exist for `v0.1.0`.
- [ ] Final commit hash for `v0.1.0` is recorded and approved.

## 2. Legal, Ownership, and Repository Hygiene

- [x] Copyright owner is identified as 3D-Components AS.
- [x] Third-party and template-origin notices are documented in `THIRD_PARTY_LICENSES.md`.
- [x] Proprietary/excluded, demonstrator-specific, and reusable open parts are documented.
- [x] Confirm no private credentials, tokens, local paths, private datasets, or generated artifacts are committed.
- [x] Confirm any included sample/demo data and model artifacts are intentionally public and documented.
- [x] Confirm `.gitignore` prevents reintroducing generated/local artifacts.

## 3. Reviewer Entry Point and Repository Map

- [x] `README.md` introduces `open-robin` as a reusable process-intelligence module.
- [x] `README.md` lists the main open modules: Process Intelligence API, ROBIN Dashboard, and ROS 2-to-FIWARE integration pattern.
- [x] `README.md` explains that welding is a reference demonstrator, not the module identity.
- [x] `README.md` lists off-the-shelf capabilities: lifecycle management, measurement ingestion, deviation detection, alerts, AI-assisted recommendations, dashboard, model management, CSV export, NGSI-LD/FIWARE, DDS mapping, and profile-driven configuration.
- [x] `README.md` includes a repository map under "What's in this Repository".
- [x] `README.md` links to documentation, evidence, roadmap, checklist, license, issue tracker, and support contact.
- [ ] Confirm README and Read the Docs links are consistent after all release docs are merged.
- [ ] Confirm repository structure documentation is sufficient for reviewers, or add a dedicated repository-structure page if needed.

## 4. Open Core, Profile Layer, and Reuse

- [x] Core modules are domain-agnostic (`Process`, `Measurement`, `GeometryTarget`, `AIRecommendation`).
- [x] ROS 2 ingestion path is DDS-first.
- [x] Legacy HTTP NGSI bridge is removed from the active workflow.
- [x] Core-vs-profile boundary is documented in `README.md`, `ARISE_PUBLICATION_ROADMAP.md`, and `docs/open_boundary.rst`.
- [x] Welding reference profile demo is available under `demo/`.
- [x] Non-welding spray-coating profile is available under `demo/profiles/`.
- [x] Demo docs use profile language so welding is not presented as the core identity.
- [ ] Add or strengthen "how to adapt this to another robotized process" documentation.
- [ ] Confirm domain-switching demo evidence clearly supports reuse beyond welding.

## 5. Installation, Dependencies, Hello World, and Demo

- [x] Docker Compose path exists for FIWARE services, API, dashboard, and ROS 2/DDS integration example.
- [x] `docs/installation.rst` exists.
- [x] `docs/quickstart.rst` exists.
- [x] `demo/validate-setup.sh` exists as a setup validation helper.
- [x] Hardware-free demo/profile scripts are available.
- [ ] Installation docs list software dependencies separately.
- [ ] Installation docs list hardware dependencies separately.
- [ ] Installation docs list simulation/mock/recorded-data dependencies separately.
- [ ] Add a no-hardware hello world with exact commands, expected output, and pass/fail criteria.
- [ ] Verify `docker compose up -d` starts the required services from a clean clone.
- [ ] Verify `./demo/validate-setup.sh` passes from a clean clone.
- [ ] Verify `curl http://localhost:8001/health` returns a healthy API response.
- [ ] Verify dashboard is reachable at `http://localhost:5174`.
- [ ] Document the basic demo with commands, expected terminal output, expected dashboard state, alert behavior, and screenshots.
- [ ] Verify the welding profile demo from a clean clone.
- [ ] Verify the spray-coating profile demo from a clean clone.

## 6. Interface Evidence

- [x] Process API and lifecycle endpoints are documented (`robin/`, `robin/alert_engine.py`, docs API guide).
- [x] Alert/deviation engine logic is documented and test-covered.
- [x] ROS 2 telemetry schema, aggregator, and DDS mapping are published:
  - [x] `vulcanexus_ws/src/robin_interfaces/msg/ProcessTelemetry.msg`
  - [x] `vulcanexus_ws/src/robin_telemetry/robin_telemetry/telemetry_aggregator_node.py`
  - [x] `config-dds.json`
- [x] DDS-to-NGSI-LD architecture is documented in `docs/DDS_BRIDGE_ARCHITECTURE.md`.
- [x] Monitoring UI shell is published under `robin-dashboard/`.
- [ ] Add a central ROS 2 / Vulcanexus reference with nodes, topics, services, actions, messages, parameters, launch files, and QoS assumptions.
- [ ] Add a central FIWARE / NGSI-LD data model reference with entities, properties, relationships, contexts, payload examples, API usage, history, and export behavior.
- [ ] Explain and validate the complete flow: ROS 2 topic -> DDS Enabler -> Orion-LD -> Mintaka/history -> API/dashboard.
- [ ] Add explicit ROS4HRI / ROS4RI intent-skill-task-mission mapping or justified N/A notes for unsupported concepts.

## 7. Evidence and Media

- [x] Evidence index and artifacts exist under `media/`.
- [x] Documentation evidence page exists at `docs/evidence.rst`.
- [x] D3 demonstrator video link is included in the evidence page.
- [x] SUS/user-validation folder link is included in the evidence page.
- [x] Architecture/data-flow evidence is available or linked from `media/`.
- [x] Dashboard screenshots are available or linked from `media/`.
- [x] Demonstrator screenshots are available or linked from `media/`.
- [x] Example API responses are available under `media/api-responses/`.
- [x] CSV export example is available under `media/exports/`.
- [x] Open online model evidence and proprietary/offline benchmark evidence are separated under `media/model-evidence/`.
- [x] Human-final-authority statement is visible in evidence docs.
- [ ] Confirm media links work from both GitHub and the generated docs site after final merge.

## 8. Limitations, Safety, and Troubleshooting

- [x] Known limitations are documented in `docs/limitations.rst`.
- [x] Open/proprietary/commercial boundaries are documented in `docs/open_boundary.rst`.
- [x] Troubleshooting guide covers Docker not running.
- [x] Troubleshooting guide covers ports already in use.
- [x] Troubleshooting guide covers UID/GID/DISPLAY setup.
- [x] Troubleshooting guide covers Orion-LD unavailable.
- [x] Troubleshooting guide covers Mintaka unavailable.
- [x] Troubleshooting guide covers dashboard/API connectivity.
- [x] Troubleshooting guide covers no telemetry displayed.
- [x] Troubleshooting guide covers missing model files.
- [x] Troubleshooting guide covers DDS mapping not loading.
- [x] Limitations cover hardware-specific constraints.
- [x] Limitations cover single-cell validation.
- [x] Limitations cover welding model/domain limits.
- [x] Limitations cover production authentication/authorization hardening.
- [x] Limitations cover multi-tenant deployment as future work.
- [x] Limitations cover limited non-welding validation.
- [x] Limitations state industrial safety certification is out of scope.
- [x] Limitations cover ROS 2 `launch_testing`, hardware-in-the-loop, and end-to-end benchmarking gaps.

## 9. Tests, CI, and Quality Evidence

- [x] Backend tests are documented (`poetry run pytest -q`).
- [x] Frontend production build is documented (`npm run build` in `robin-dashboard`).
- [x] CI pipeline includes backend tests and frontend build (`.github/workflows/ci.yml`).
- [ ] Confirm backend tests pass on the final release branch.
- [ ] Confirm frontend build passes on the final release branch.
- [ ] Confirm docs build passes on the final release branch.
- [ ] Confirm known test gaps are documented and acceptable.

## 10. Release Notes, Roadmap, and Final Submission

- [x] Publication roadmap exists: `ARISE_PUBLICATION_ROADMAP.md`.
- [x] ARISE catalog metadata exists: `arise/catalog-metadata.yaml`.
- [x] Stack snapshot exists: `current_stack_feb22.md`.
- [ ] Add `CHANGELOG.md`.
- [ ] Add draft `v0.1.0` release notes with purpose, included modules, reviewer quickstart, known limitations, evidence links, open/proprietary boundary, license, and tag/date placeholder.
- [ ] Confirm roadmap does not overclaim unsupported future work.
- [ ] Create `v0.1.0` tag only after all critical readiness issues are closed and final verification is approved.
- [ ] Submit catalog entry using repository URL, metadata, roadmap, and evidence links.

## 11. Final Clean-Clone Verification

- [ ] Clone the repository from GitHub into a fresh directory.
- [ ] Check out the final branch/commit intended for `v0.1.0`.
- [ ] Run docs link checks if available.
- [ ] Run Docker Compose config validation.
- [ ] Run the reviewer quickstart.
- [ ] Run the no-hardware hello world.
- [ ] Run the canonical basic demo path.
- [ ] Run tests and local CI-equivalent checks where feasible.
- [ ] Verify Read the Docs public site works and corresponds to the submitted branch/tag.
- [ ] Verify no old repository URLs or license mismatches remain.
- [ ] Verify limitations and troubleshooting pages match the final verified quickstart/demo path.
