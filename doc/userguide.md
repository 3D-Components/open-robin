# Process ID Provider Operator

## Description

This operator manages welding process IDs for the ROBIN system. It can automatically create new processes or accept manual process IDs.

## Configuration

### Preferences

- **Alert Engine URL**: URL of the ROBIN Alert Engine API (default: http://localhost:8000)
- **Auto Create Process**: Automatically create a new process on startup (default: true)
- **Operation Mode**: Process operation mode - Parameter Driven or Geometry Driven (default: parameter_driven)

### Wiring

#### Input Endpoints

- **Manual Process ID**: Manually set a process ID (text)

#### Output Endpoints

- **Process ID**: Current active process ID (text)
- **Process Status**: Process status information (JSON)

## Usage

1. Add the operator to your workspace
2. Configure the Alert Engine URL in preferences
3. The operator will automatically create a new process if auto_create_process is enabled
4. Connect the "Process ID" output to widgets that need process information
5. Optionally connect a manual process ID input to override the automatic process

## Behavior

- On startup, creates a new process with ID format: `wirecloud-{timestamp}`
- Sends the process ID to connected widgets
- Monitors process status and reports changes
- Can accept manual process IDs via input endpoint

## Integration

Connect this operator to:
- Measurement KPI widgets (Process ID input)
- Chart widgets (Process ID input)
- Any component that needs the current process ID