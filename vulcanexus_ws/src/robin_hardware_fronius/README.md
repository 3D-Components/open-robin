# Fronius Welding Package (Example Domain Adapter)

> **Note:** This is an **example domain adapter**, not a reusable ROBIN module.
> It shows how to integrate a specific piece of industrial hardware (Fronius
> welding machine) with the ROBIN telemetry pipeline (Module 3).
> Use this as a reference when writing your own hardware adapter.

## Nodes

- `welding_data`: Reads real time data from the welding machine and publishes it. Communicates through OPC-UA
- `welding_controller`: Controls the welding operation like arc on/off etc. Stuff that is not part of "welding parameters". Communicates through RB FB/i interface card
- `welding_parameters`: Reads and writes welding parameters. Communicates through OPC-UA