# ROS Bag Samples

This folder contains small ROS bag samples that are intentionally committed for
the open demo and replay path.

## Included Samples

| Folder | Purpose |
|---|---|
| `correct_process_params/` | Compact replay sample for process telemetry and welding-related message types. |
| `bag_2026-03-16/` | Additional compact replay sample used for ROS 2/DDS integration checks. |

## Public Scope

These samples are release artifacts, not private production datasets. They are
included so reviewers can inspect representative ROS 2 message metadata and use
the documented replay path without access to the original industrial cell.

Do not commit large experiment bags, customer data, private cell recordings, or
site-specific bags here. Keep those in a private/local workspace and document
only the minimum sample needed for a public replay.
