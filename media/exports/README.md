# CSV Export Evidence

This folder contains a representative CSV export for the ROBIN Dashboard **History / Reports** tab.

The dashboard export is implemented in [`../../robin-dashboard/src/components/features/history/HistoryTab.tsx`](../../robin-dashboard/src/components/features/history/HistoryTab.tsx). It combines:

- raw process telemetry rows from `GET /process/{process_id}/measurements`;
- persisted warning/deviation rows from `GET /process/{process_id}/alerts`;
- the raw backend payload for each row in `raw_payload_json`.

The example file below mirrors the current UI export column set and filename pattern:

| File | Profile | Description |
| --- | --- | --- |
| [reviewer_demo_process-history-2026-06-16T12-02-00-000Z.csv](reviewer_demo_process-history-2026-06-16T12-02-00-000Z.csv) | Welding (reference) | Representative History / Reports export for a no-hardware demo process, including twelve telemetry rows and one warning row. |

## How to Generate This From The UI

1. Open the ROBIN Dashboard.
2. Go to **History / Reports**.
3. Select the process ID, for example `reviewer_demo_process`.
4. Click **Refresh**.
5. Click **Download CSV**.

The downloaded filename follows:

```text
<process_id>-history-<timestamp>.csv
```

## Notes

- This committed CSV is a representative example aligned with the dashboard export code, not a fresh clean-clone runtime capture. No separate spray-coating CSV export is bundled in this release; see [`../../docs/user_guide/profiles.rst`](../../docs/user_guide/profiles.rst) for the reuse path.
- After you run the quickstart/demo, replace or supplement it with a CSV downloaded from your own run.
- `record_type=measurement` rows are telemetry samples.
- `record_type=warning` rows are persisted deviation alerts.
- `raw_payload_json` preserves the backend payload used by the dashboard for traceability.
