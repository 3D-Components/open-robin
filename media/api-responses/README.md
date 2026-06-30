# Example API Responses

This folder contains representative ROBIN API response examples. The examples follow the current FastAPI response shapes in [`../../robin/alert_engine.py`](../../robin/alert_engine.py) and use the no-hardware demo process id `reviewer_demo_process`.

These files document the API contract so you can see the expected request/response shapes before wiring the modules into your own stack. Timestamps, latency values, and numeric process values are illustrative examples; you can refresh this folder with captured output from your own clean-clone run.

Request payload fixtures used by the longer curl examples are included in this folder so the commands can be copied directly from `media/api-responses/`. The Process Intelligence API is served on port **8001** (see `docker-compose.yaml` and the [quickstart](../../docs/quickstart.rst)).

No separate spray-coating API-response files are bundled in this release. The
non-welding reuse path is covered by the spray-coating profile and demo script
under [`../../demo/profiles/`](../../demo/profiles/).

## Response Index

| Flow step | Command | Example response |
| --- | --- | --- |
| Health check | `curl http://localhost:8001/health` | [health-response.json](health-response.json) |
| Create demo process | `curl -X POST http://localhost:8001/create-process -H 'Content-Type: application/json' -d @create-process-request.json` | [create-process-response.json](create-process-response.json) |
| Set target geometry | `curl -X POST http://localhost:8001/process/reviewer_demo_process/target -H 'Content-Type: application/json' -d '{"height": 3.2, "width": 7.0}'` | [set-target-response.json](set-target-response.json) |
| Read latest measurements | `curl 'http://localhost:8001/process/reviewer_demo_process/measurements?last=1'` | [process-measurements-response.json](process-measurements-response.json) |
| Check deviation | `curl -X POST http://localhost:8001/check-deviation -H 'Content-Type: application/json' -d @check-deviation-request.json` | [check-deviation-response.json](check-deviation-response.json) |
| Request AI recommendation | `curl -X POST http://localhost:8001/ai-recommendation -H 'Content-Type: application/json' -d @ai-recommendation-request.json` | [ai-recommendation-response.json](ai-recommendation-response.json) |
| Publish operator intent | `curl -X POST http://localhost:8001/intent -H 'Content-Type: application/json' -d '{"intent": "REQUEST_AI_RECOMMENDATION", "process_id": "reviewer_demo_process"}'` | [publish-intent-response.json](publish-intent-response.json) |

## Expected Demo Meaning

Together, these responses document the basic no-hardware API path covered by the quickstart:

1. API and FIWARE services are reachable through `/health`.
2. You can create a demo process.
3. A target geometry can be associated with the process.
4. Mock or recorded telemetry can be read back as process measurements.
5. Deviation detection returns the measured geometry, expected geometry, and recommended operator actions.
6. AI recommendation returns advisory parameters or predicted geometry.
7. Operator intent can be published for the ROS 2/FIWARE bridge path.

The human operator remains the final authority for any recommendation returned by these endpoints.
