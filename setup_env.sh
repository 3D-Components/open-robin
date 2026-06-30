#!/bin/bash
set -e

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$SCRIPT_DIR"

# 1. Create .env file for Docker Compose
echo "Creating .env file with current user UID/GID..."
echo "UID=$(id -u)" > "$PROJECT_ROOT/.env"
echo "GID=$(id -g)" >> "$PROJECT_ROOT/.env"
echo "Created .env: UID=$(id -u), GID=$(id -g)"

# 2. Configure X11 permissions for GUI support
echo "Configuring X11 permissions..."
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker
    echo "X11 permissions granted to local docker users."
else
    echo "Warning: 'xhost' command not found. GUI applications may not work."
fi

echo "Environment setup complete!"
echo "You can now run: docker compose up -d"
