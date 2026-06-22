# Example API Responses

This folder contains representative ROBIN API response examples for the evidence pack. The examples follow the current FastAPI response shapes in [`../../robin/alert_engine.py`](../../robin/alert_engine.py) and use the no-hardware reviewer process id `reviewer_demo_process`.

These files are intended as reviewer-facing evidence of the API contract. Timestamps, latency values, and numeric process values are illustrative examples; once the reviewer quickstart is finalized, this folder can be refreshed with captured output from a clean-clone run.

Request payload fixtures used by the longer curl examples are included in this folder so the commands can be copied directly from `media/api-responses/`.

## Response Index

| Flow step | Command | Example response |
| --- | --- | --- |
| Health check | `curl http://localhost:8000/health` | [health-response.json](health-response.json) |
| Create demo process | `curl -X POST http://localhost:8000/create-process -H 'Content-Type: application/json' -d @create-process-request.json` | [create-process-response.json](create-process-response.json) |
| Set target geometry | `curl -X POST http://localhost:8000/process/reviewer_demo_process/target -H 'Content-Type: application/json' -d '{"height": 3.2, "width": 7.0}'` | [set-target-response.json](set-target-response.json) |
| Read latest measurements | `curl 'http://localhost:8000/process/reviewer_demo_process/measurements?last=1'` | [process-measurements-response.json](process-measurements-response.json) |
| Check deviation | `curl -X POST http://localhost:8000/check-deviation -H 'Content-Type: application/json' -d @check-deviation-request.json` | [check-deviation-response.json](check-deviation-response.json) |
| Request AI recommendation | `curl -X POST http://localhost:8000/ai-recommendation -H 'Content-Type: application/json' -d @ai-recommendation-request.json` | [ai-recommendation-response.json](ai-recommendation-response.json) |
| Publish operator intent | `curl -X POST http://localhost:8000/intent -H 'Content-Type: application/json' -d '{"intent": "REQUEST_AI_RECOMMENDATION", "process_id": "reviewer_demo_process"}'` | [publish-intent-response.json](publish-intent-response.json) |

## Expected Demo Meaning

Together, these responses document the basic no-hardware API path expected by the reviewer quickstart:

1. API and FIWARE services are reachable through `/health`.
2. A reviewer can create a demo process.
3. A target geometry can be associated with the process.
4. Mock or recorded telemetry can be read back as process measurements.
5. Deviation detection returns the measured geometry, expected geometry, and recommended operator actions.
6. AI recommendation returns advisory parameters or predicted geometry.
7. Operator intent can be published for the ROS 2/FIWARE bridge path.

The human operator remains the final authority for any recommendation returned by these endpoints.
