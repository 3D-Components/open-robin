#!/bin/bash
# Setup DDS temporal support for Mintaka
#
# This script creates a PostgreSQL trigger that auto-populates observedAt
# timestamps for data written by Orion-LD's DDS bridge.
#
# The trigger extracts the original sensor timestamp from the ROS message
# header.stamp field when available, falling back to DB insert time otherwise.
#
# Run this AFTER docker compose up when the stack is healthy.
#
# Usage: ./scripts/setup_dds_temporal.sh

set -e

echo "=== Setting up DDS temporal support for Mintaka ==="

# Wait for TimescaleDB to be ready
echo "Waiting for TimescaleDB..."
until docker exec fiware-timescaledb pg_isready -U orion -d orion > /dev/null 2>&1; do
    sleep 1
done

# Check if attributes table exists (created by Orion-LD TROE)
TABLE_EXISTS=$(docker exec fiware-timescaledb psql -U orion -d orion -tAc \
    "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'attributes');")

if [ "$TABLE_EXISTS" != "t" ]; then
    echo "⚠️  attributes table doesn't exist yet."
    echo "   Start Orion-LD with TROE enabled first, then re-run this script."
    exit 1
fi

# Create the trigger
echo "Creating observedAt trigger (uses ROS header.stamp when available)..."

# Step 1: Drop existing trigger
docker exec fiware-timescaledb psql -U orion -d orion -c \
    "DROP TRIGGER IF EXISTS trigger_set_observedat ON attributes;"

# Step 2: Create ROS-aware function
docker exec fiware-timescaledb psql -U orion -d orion -c "
CREATE OR REPLACE FUNCTION set_observedat_ros()
RETURNS TRIGGER AS \$\$
BEGIN
    IF NEW.observedat IS NULL THEN
        -- Try to extract timestamp from ROS message header.stamp
        IF NEW.compound IS NOT NULL 
           AND NEW.compound ? 'header' 
           AND (NEW.compound->'header') ? 'stamp'
           AND (NEW.compound->'header'->'stamp') ? 'sec' THEN
            NEW.observedat := to_timestamp(
                (NEW.compound->'header'->'stamp'->>'sec')::double precision + 
                COALESCE((NEW.compound->'header'->'stamp'->>'nanosec')::double precision, 0) / 1000000000.0
            );
        ELSE
            -- Fallback to DB insert time
            NEW.observedat := NEW.ts;
        END IF;
    END IF;
    RETURN NEW;
END;
\$\$ LANGUAGE plpgsql;
"

# Step 3: Create trigger
docker exec fiware-timescaledb psql -U orion -d orion -c "
CREATE TRIGGER trigger_set_observedat
    BEFORE INSERT ON attributes
    FOR EACH ROW
    EXECUTE FUNCTION set_observedat_ros();
"

# Verify
TRIGGER_FUNC=$(docker exec fiware-timescaledb psql -U orion -d orion -tAc \
    "SELECT p.proname FROM pg_trigger t JOIN pg_proc p ON t.tgfoid = p.oid WHERE t.tgname = 'trigger_set_observedat';")

if [ "$TRIGGER_FUNC" = "set_observedat_ros" ]; then
    echo "✅ DDS temporal support enabled!"
    echo ""
    echo "The DDS bridge will now work with Mintaka temporal queries."
    echo "- ROS messages with header.stamp → uses original sensor timestamp"
    echo "- Other data → uses DB insert timestamp"
else
    echo "❌ Failed to create trigger (got function: $TRIGGER_FUNC)"
    exit 1
fi
