# ROBIN MVP - Technical Report (FIWARE slice, no ROS)

*This notebook is both a living technical logbook and the seed of the formal report for the FIWARE side. The goal is to make the MVP understandable end‑to‑end and to clarify what exists, why it exists, and what to do next. Read it as if a senior engineer were handing the project to a junior engineer.*

> Note: this logbook contains historical MVP notes. For the current open-source baseline, use `README.md`, `current_stack_feb22.md`, and `ARISE_PUBLICATION_ROADMAP.md`.

---

## 1) Executive overview

This MVP stands up the FIWARE half of our planned architecture for robotic process monitoring. Orion‑LD is the source of truth for process and quality entities; MongoDB backs Orion's storage. A lightweight Alert Engine (FastAPI) performs deviation checks and serves a simple browser dashboard built with vanilla HTML and Plotly. As per the roadmap, FAROS will be introduced as a configurable workspace that reuses the same Orion/Mintaka/Alert Engine APIs (see §13) and eventually replaces the ad‑hoc page for operator‑facing views.

A Python CLI (Typer) simulates sensors and AI by creating **Process**, **Measurement**, and **AIRecommendation** entities directly in Orion. TROE (Temporal Representation of Entities) automatically stores temporal data in TimescaleDB, and Mintaka provides temporal query capabilities so that historical windows are available for analytics and, later, FAROS charts. The current dashboard renders time‑series plots from measurements fetched via Mintaka's temporal API.

The RobTrack bridge is sketched so we can later swap our mock models for the proprietary forward/inverse models. Two operating modes are supported conceptually: **parameter‑driven** (we set process parameters and predict geometry) and **geometry‑driven** (we set target geometry and ask the model for parameters). The alerting loop is the same in both modes: compare an expected geometry (from either targets or predictions) against actual measurements, compute % deviation, and present the operator a small menu of next actions (manual tweak, ask AI again, fine‑tune, or reset).

---

## 2) Architecture, in plain language

**Context broker & storage.** Orion‑LD exposes an NGSI‑LD API for our domain entities. We run MongoDB because Orion uses it as its persistence layer. Entities are stored with typed Properties, Relationships, and timestamps, which lets us model processes, target dimensions, live measurements, and AI recommendations in a uniform graph. This keeps us vendor‑neutral and makes later integrations (subscriptions, northbound replication) predictable.

**Temporal storage path.** TROE (Temporal Representation of Entities) automatically captures historical measurements and recommendations in TimescaleDB whenever entities with temporal properties are updated in Orion-LD. Mintaka provides a temporal query API over this TimescaleDB storage, enabling charts and analytics to access historical data without manual subscriptions or configuration. FAROS widgets consume this temporal data via the Alert Engine's measurement endpoints, which internally query Mintaka's temporal API. The legacy dashboard also fetches measurement time-series via the same path and reads current state from Orion‑LD via the Alert Engine.

**Application layer.** The Alert Engine is a small FastAPI app. It serves a legacy dashboard HTML (retained for development/testing) and exposes a handful of JSON endpoints. The core endpoint takes a process context plus tolerance and returns whether a deviation exists and, if so, what to do about it. The engine includes simple mock functions for both forward (parameters → geometry) and inverse (geometry → parameters) modeling; these are placeholders for RobTrack.

**Operator UI.** FAROS is now the primary operator interface, providing a configurable workspace composed of custom widgets and operators. The workspace includes: a **Process ID Provider** operator for process selection, a **Timeseries Monitor** widget for live plotting of height/width/speed/current/voltage, a **Measurement KPI** widget for key metrics display, a **Deviation Alert** widget for real-time alerting, and **Process Controls** for start/stop operations. These widgets communicate via the Alert Engine's existing JSON endpoints and query Mintaka's temporal API for historical data. A legacy static HTML dashboard remains available for development and testing purposes.

**Developer tooling.** The Typer CLI is the glue that lets us demo without real robots: create a process, add targets, push measurements as if a profilometer had produced them, and store AI recommendations. It also has a status command to confirm Orion/Mintaka are reachable and a serve command to launch the FastAPI app with Uvicorn.

**Why these choices.** Orion‑LD + NGSI‑LD gives us structured, unit‑aware data with Relationships, which is a natural fit for process. TROE + TimescaleDB + Mintaka gives us automatic temporal storage with minimal setup and powerful time-series queries. FastAPI/Typer keeps the surface Pythonic and fast to iterate. FAROS provides a governed, reconfigurable operator console without changing backend APIs, allowing drag-and-drop dashboard composition from reusable widgets and operators.

---

## 3) Data model you can hold in your head

There are four core types, all in the tenant namespace `robin` and sharing a common `@context`.

* **Process** is the anchor entity. It records the operation mode (parameter‑driven or geometry‑driven) and the canonical process parameters (temperature, wireSpeed, voltage, current, travelSpeed). It also stores a `toleranceThreshold` so the operator can change sensitivity without redeploying code.
* **GeometryTarget** exists only for geometry‑driven runs. It hangs off a process via a Relationship and carries `targetHeight` and `targetWidth` in millimeters.
* **Measurement** is the live data from the line (or the simulator). Each Measurement references its Process and provides `measuredHeight` and `measuredWidth`, typically with `observedAt` timestamps.
* **AIRecommendation** stores either a forward prediction snapshot (when parameter‑driven) or an inverse suggestion for parameters (when geometry‑driven). We also keep a `confidence` and `modelVersion` for traceability.

Everything is discoverable with Orion’s NGSI‑LD endpoints and the tenant header set to `robin`. This is enough structure for the alert loop, subscriptions to history, and later joining with ROS topics.

---

## 4) What actually happens at runtime

**Parameter‑driven path.** The operator (or CLI) chooses process parameters; the engine uses the forward model to generate an expected geometry. Measurements come in, and we compare measured height/width against expected height/width. If either exceeds the tolerance percentage, we open an alert and offer next actions. The “AI retry” action calls the inverse path to propose a small parameter adjustment and stores it as a new AIRecommendation.

**Geometry‑driven path.** The operator sets target height/width on the process. The engine compares incoming measurements to those targets. If out of spec, the same alert menu shows up; “AI retry” asks the inverse model for parameters that are more likely to hit the target and stores that suggestion.

In both paths the comparison is the same: `deviation% = |measured − expected| / expected × 100`, guarded for zero expected values. The engine picks the larger of the two (height vs width) as the alert’s deviation percentage so the operator reacts to the worst offender.

---

## 5) Services and deployment notes

The compose file launches six containers: Orion‑LD, MongoDB, TimescaleDB, Mintaka, the **Alert Engine**, and **FAROS**. We pin AMD64 images so the stack behaves the same across dev laptops (including Apple Silicon). For Apple Silicon, we set `platform: linux/amd64` on images to ensure consistent behavior. Orion is exposed on `1026`, Mintaka on `9090`, TimescaleDB on `5433` (host port), the Alert Engine on `8000`, and FAROS on `8080`. The Alert Engine mounts a config directory and depends on Orion and Mintaka so it starts in the right order. FAROS uses TimescaleDB for its own metadata storage and connects to the Alert Engine for process data. TROE is enabled in Orion-LD to automatically store temporal data in TimescaleDB.

Once the stack is up, the CLI's `status` command is a quick way to sanity‑check that Orion and Mintaka respond. FAROS is accessible at `http://localhost:8080` and provides the primary operator interface through custom ROBIN widgets and operators.

---

## 6) The Alert Engine, demystified

Think of the Alert Engine as a thin adapter between the entity graph and the operator. It has a health endpoint (to verify the Orion connection), it serves the dashboard, and it exposes two actions: **check‑deviation** (the computation) and **operator‑action** (how we react). Under the hood it keeps a tiny library of “post‑alert” actions: manual parameter adjustment, ask AI for another suggestion, kick off a fine‑tune job, or reset the model/DOE.

The forward and inverse model functions are intentionally simple; they exist so we can wire up the UI and the persistence and later swap in RobTrack’s real APIs without touching the surrounding logic. One detail we’ve completed: the engine persists an **Alert** entity in Orion for audit (deviation %, expected vs measured payload, chosen action, timestamp). Cleanup excludes alerts so audits survive demo runs.

---

## 7) The dashboard, and how it talks to the backend

The FAROS workspace provides a professional operator interface with drag-and-drop configurability. The **Process ID Provider** operator manages process selection and creation, feeding process IDs to connected widgets. The **Timeseries Monitor** widget plots height/width/speed/current/voltage from measurement data via the Alert Engine's `/process/{id}/measurements` endpoint, which queries Mintaka's temporal API. The **Measurement KPI** widget displays current values and deviation percentages. The **Deviation Alert** widget polls `/check-deviation` and presents operator actions when thresholds are exceeded. The **Process Controls** widget manages process lifecycle (start/stop/resume) via the Alert Engine's process management endpoints.

In parameter‑driven mode, expected values come from the forward model; in geometry‑driven mode, expected values are taken from the process's **GeometryTarget** (fetched server‑side). FAROS widgets query Mintaka via the Alert Engine for temporal data, while direct Mintaka queries remain available for ML/batch analytics. The legacy HTML dashboard remains available at `/dashboard` for development and testing purposes.

---

## 8) The CLI, as the demo robot

The Typer CLI is straightforward but powerful for demos. It can create a **Process** (with either mode), attach a **GeometryTarget**, add one or more **Measurements**, and store an **AIRecommendation**. It prints friendly confirmations so you can see what it did. There’s also a `status` command that pings Orion and QuantumLeap to confirm connectivity, and a `serve` command that runs the FastAPI app with Uvicorn.

In practice, the demo flow is: create process → add target (if geometry‑driven) → push one or two measurements → add a recommendation → open FAROS at `http://localhost:8080` and configure widgets to monitor the process.

---

## 9) The RobTrack bridge, and how we’ll swap it in

The RobTrack bridge is a thin adapter that calls RobTrack endpoints for forward (parameters → geometry) and inverse (geometry → parameters) predictions. After getting a response, it translates that result into our **AIRecommendation** entity and publishes it to Orion. There’s also a hook for fine‑tuning: send a new datapoint and request a quick retrain. This is enough of a seam that we can replace the engine’s mock functions with real bridge calls and keep the rest of the system unchanged.

---

## 10) Status and what’s still missing (to close the loop)

**What's already done.** TROE (Temporal Representation of Entities) is enabled in Orion-LD and automatically stores temporal data in TimescaleDB whenever entities with temporal properties are updated. Mintaka provides temporal query capabilities over this data. The Alert Engine now persists **Alert** entities in Orion for audit, and cleanup excludes them so demo runs retain history. The dashboard has been aligned to the NGSI‑LD model and queries measurement time-series via Mintaka's temporal API for both KPIs/plots and the deviation check payload.

**What remains to finish the MVP.** Security for multi‑tenant deployments should be added beyond the tenant header (auth in front of Orion and the app once we leave demo scope). ROS integration should replace the CLI with ROS 2 publishers/subscribers that transform robot and sensor topics into **Measurement** entities and consume **AIRecommendation** and/or **Alert** streams to drive the robot UI. Additional FAROS widgets could be developed for AI recommendation display and advanced process analytics.

**FAROS implementation - now delivered.** FAROS has been implemented as the primary operator interface, replacing the need for the custom HTML dashboard in production. The current implementation includes: a **Process ID Provider** operator for process selection and management; a **Timeseries Monitor** widget for real-time plotting fed by Mintaka temporal queries; a **Measurement KPI** widget for key metrics display; a **Deviation Alert** widget connected to the Alert Engine's `/check-deviation` endpoint; and **Process Controls** for lifecycle management. All widgets communicate via the existing Alert Engine JSON APIs, maintaining backend compatibility while providing professional configurability.

---

## 11) How this maps to the implementation plan

The implementation plan calls for a closed loop between a robot (ROS), process/quality state, and an AI model that can both predict and propose. This MVP delivers that loop on the FIWARE side: entities, a context broker, a simple alert brain, and a professional operator interface. That's enough to prove the concept and to plug real data once ROS nodes are ready. The FAROS implementation provides configurable, multi‑view operator dashboards without touching backend APIs, completing the human-machine interface portion of the architecture.

---

## 12) Quick reference: how to run and validate

Bring the stack up with Docker Compose (TROE automatically stores temporal data in TimescaleDB when entities are updated). Then run `./demo/interactive-demo.sh` to validate the full loop. Use the CLI's `status` to check that Orion and Mintaka respond. Create one parameter‑driven process and one geometry‑driven process, add a couple of measurements to each, and post an AI recommendation. Open FAROS at `http://localhost:8080`, upload the custom ROBIN widgets and operators (from `wirecloud-widgets/` and `wirecloud-operators/` directories), and create a workspace to monitor your processes. Confirm the time-series plots update and alerts appear when you set strict tolerances. The legacy HTML dashboard remains available at `http://localhost:8000/dashboard` for development purposes. From there, iterate on the remaining to‑dos and then replace the mock models with calls to the RobTrack bridge.

---

## 13) The FAROS implementation (delivered and operational)

FAROS is FIWARE's mashup platform for assembling dashboards from reusable components, and it has been successfully implemented as the primary operator interface for this MVP. The implementation replaces the custom HTML/Plotly page with a configurable workspace composed of custom widgets (visual elements) and operators (data sources and transforms). Backend components remain unchanged: Orion‑LD is the context broker, Mintaka provides temporal query capabilities over TROE data stored in TimescaleDB, and the Alert Engine exposes `/check-deviation` and `/operator-action`.

**Custom components delivered:**

- **Process ID Provider** operator: Manages process selection and creation, providing process IDs to connected widgets via FAROS's wiring system
- **Timeseries Monitor** widget: Plots real-time height, width, speed, current, and voltage data by querying the Alert Engine's `/process/{id}/measurements` endpoint, which internally uses Mintaka's temporal API
- **Measurement KPI** widget: Displays current measurement values and computed deviation percentages in a clean metrics dashboard
- **Deviation Alert** widget: Polls the Alert Engine's `/check-deviation` endpoint and presents operator action buttons (manual adjust, AI retry, fine-tune, reset model) when thresholds are exceeded
- **Process Controls** widget: Manages process lifecycle through start/stop/resume operations via the Alert Engine's process management endpoints

**Architecture and wiring:** Each widget operates independently and communicates through FAROS's event-driven wiring system. The Process ID Provider operator broadcasts process IDs to all connected widgets, creating a cohesive monitoring experience. Widgets poll their respective Alert Engine endpoints at configurable intervals (typically 2-5 seconds) and update their displays in real-time. The Alert Engine serves as the integration layer, translating between FAROS's HTTP-based requests and the underlying FIWARE components (Orion-LD, Mintaka, TimescaleDB).

**Deployment and operations:** FAROS runs as a containerized service in `docker-compose.yaml`, accessible at `http://localhost:8080`. It uses TimescaleDB for its own metadata storage (workspace configurations, widget instances, user preferences) via a dedicated `wirecloud` database. Custom widgets and operators are packaged as `.wgt` files and can be uploaded through the FAROS catalog interface. The workspace is drag-and-drop configurable, allowing operators to customize their monitoring dashboards without code changes.

**Integration benefits achieved:** The FAROS implementation provides professional operator dashboards without changing backend APIs. Operators can create multiple workspaces for different processes, adjust widget layouts, and configure monitoring parameters through the UI. The system maintains full compatibility with the existing Alert Engine while adding governance, configurability, and multi-user support that would be difficult to achieve with static HTML pages.

---

### Closing note

This is the right skeleton: standards‑based data model, a thin server, and a professional configurable UI. The FAROS implementation completes the operator interface portion of the MVP. The next development phase should focus on the remaining items in §10 (security and ROS integration) so that swapping in the real ROS and RobTrack pieces is just integration work, not a redesign.
