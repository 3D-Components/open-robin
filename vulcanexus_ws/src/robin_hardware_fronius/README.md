# Fronius Welding Package

## Nodes

- `welding_data`: Reads real time data from the welding machine and publishes it. Communicates through OPC-UA
- `welding_controller`: Controls the welding operation like arc on/off etc. Stuff that is not part of "welding parameters". Communicates through RB FB/i interface card
- `welding_parameters`: Reads and writes welding parameters. Communicates through OPC-UA